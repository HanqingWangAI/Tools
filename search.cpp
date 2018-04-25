#include "cmaes.h"
#include "core/types.h"
#include "core/maths.h"
#include "core/platform.h"
#include "core/mesh.h"
#include "core/voxelize.h"
#include "core/sdf.h"
#include "core/pfm.h"
#include "core/tga.h"
#include "core/perlin.h"
#include "core/convex.h"
#include "core/cloth.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <stack>
#include <queue>
#include "core/octree.h"


using namespace libcmaes;

int penalty = 1;
int eval_count = 0;

std::string temp_obj_path = "./temp/objs";
std::string temp_val_path = "./temp/vals";

const float threshold = 0.5f;

const float eps = 1e-6;

Mesh cube, tetra;

bool IntersectTriangle(Vec3 orig, Vec3 dir,
    Vec3 v0, Vec3 v1, Vec3 v2)
{	
    float*t,*u,*v;
	float a,b,c;
	t = &a;
	u = &b;
	v = &c;
    // E1
    Vec3 E1 = v1 - v0;

    // E2
    Vec3 E2 = v2 - v0;

    // P
    Vec3 P = Cross(dir,E2);

    // determinant
    float det = Dot(E1,P);

    // keep det > 0, modify T accordingly
    Vec3 T;
    if( det >0 )
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if( det < 0.0001f )
        return false;

    // Calculate u and make sure u <= 1
    *u = Dot(T,P);
    if( *u < 0.0f || *u > det )
        return false;

    // Q
    Vec3 Q = Cross(T,E1);

    // Calculate v and make sure u + v <= 1
    *v = Dot(dir,Q);
    if( *v < 0.0f || *u + *v > det )
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    *t = Dot(E2,Q);

    float fInvDet = 1.0f / det;
    *t *= fInvDet;
    *u *= fInvDet;
    *v *= fInvDet;

    return true;
}

bool Intersect(Vec3 a0, Vec3 b0, Vec3 c0,
	Vec3 a1,Vec3 b1,Vec3 c1)
{
	Vec3 t1[3] = {a0,b0,c0},t2[3] = {a1,b1,c1}, x0, y0;
	int cnt = 0;
	//find the same vertex
	for(int i = 0;i < 3;i++){
		for(int j = 0;j < 3;j++){
			if(!(t1[i] != t2[j])) {
				cnt++;
				// find other two vertices
				if(i != 0) x0 = t1[0];
				else x0 = t1[1];
				if(i != 2) y0 = t1[2];
				else y0 = t1[1];
			}
		}
	}
	// if triangles share an edge
	if(cnt >= 2) return false;
	// if triangles share a vertex
	if(cnt == 1) return IntersectTriangle(x0,y0-x0,a1,b1,c1)&&IntersectTriangle(y0,x0-y0,a1,b1,c1);

	if(IntersectTriangle(a0,b0-a0,a1,b1,c1)&&IntersectTriangle(b0,a0-b0,a1,b1,c1)) return true;
	if(IntersectTriangle(a0,c0-a0,a1,b1,c1)&&IntersectTriangle(c0,a0-c0,a1,b1,c1)) return true;
	if(IntersectTriangle(c0,b0-c0,a1,b1,c1)&&IntersectTriangle(b0,c0-b0,a1,b1,c1)) return true;
	return false;
}

inline float Mod(Vec3 v){
	return sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
}

Vec3 CalcNormal(Vec3 v0, Vec3 v1, Vec3 v2){
	Vec3 a,b;
	a = v1-v0;
	b = v2-v1;
	Vec3 temp = Cross(a,b);
	return temp /= Mod(temp);
}


Mesh* generate(const double *array, int dim){
    const double *cubes = array;
    
    Mesh *mesh = new Mesh();
    int num_points = 0;
    for(int i = 0; i < dim;i++){
        for(int j = 0;j < dim;j++){
            for(int k = 0;k < dim;k++){
                Point3 translation = Point3(i,j,k);
    
                if(array[i*dim*dim+j*dim+k]<threshold) continue;
                for (int l = 0; l < cube.m_positions.size(); l++){
                    Point3 temp = cube.m_positions[l];
                    temp = temp + translation;
                    mesh->m_positions.push_back(temp);
                    mesh->m_normals.push_back(Vec3(0,0,0));
                }

                for (int l = 0; l < cube.m_indices.size(); l++){
                    mesh->m_indices.push_back(cube.m_indices[l]+num_points);
                }

                num_points += 8;
            }
        }
    }
    
    return mesh;
}

FitFunc fsphere = [](const double *x, const int N){
	double res = 0.0;
	int id = eval_count++;
	Mesh *mesh;
    int dim = 10;
	mesh = generate(x, dim);
	
	
	char full_obj_path[100],full_val_path[100],cmd[200];
	sprintf(full_obj_path,"%s/%d.obj",temp_obj_path.c_str(),id);
	sprintf(full_val_path,"%s/%d",temp_val_path.c_str(),id);
	sprintf(cmd,"./NvFlexDemoReleaseCUDA_x64 %s %s",full_obj_path,full_val_path);
	ExportToObj(full_obj_path, mesh);
	delete mesh;
    //printf("%s\n%s\n",full_obj_path,cmd);
	system(cmd);
	//printf("OK\n");

	FILE* fp = fopen(full_val_path,"r");
	int val;
	
	fscanf(fp,"%d",&val);
	fclose(fp);
	res -= val;
	printf("Score is %d, evaluation is %f\n",val, res);
	fp = fopen("scores.txt","a");
	fprintf(fp,"%d %f\n",id, res);
	fclose(fp);
	
	return res;
};

void init(){
    printf("Init start!\n");
    cube.m_positions.push_back(Point3(0,0,0));
    cube.m_positions.push_back(Point3(1,0,0));
    cube.m_positions.push_back(Point3(1,1,0));
    cube.m_positions.push_back(Point3(0,1,0));
    cube.m_positions.push_back(Point3(0,0,1));
    cube.m_positions.push_back(Point3(1,0,1));
    cube.m_positions.push_back(Point3(1,1,1));
    cube.m_positions.push_back(Point3(0,1,1));

    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    cube.m_normals.push_back(Vec3(0,0,0));
    
    cube.m_indices.push_back(3);
    cube.m_indices.push_back(1);
    cube.m_indices.push_back(0);

    cube.m_indices.push_back(3);
    cube.m_indices.push_back(2);
    cube.m_indices.push_back(1);

    cube.m_indices.push_back(4);
    cube.m_indices.push_back(3);
    cube.m_indices.push_back(0);

    cube.m_indices.push_back(7);
    cube.m_indices.push_back(3);
    cube.m_indices.push_back(4);

    cube.m_indices.push_back(6);
    cube.m_indices.push_back(2);
    cube.m_indices.push_back(3);

    cube.m_indices.push_back(7);
    cube.m_indices.push_back(6);
    cube.m_indices.push_back(3);

    cube.m_indices.push_back(1);
    cube.m_indices.push_back(4);
    cube.m_indices.push_back(0);

    cube.m_indices.push_back(1);
    cube.m_indices.push_back(5);
    cube.m_indices.push_back(4);

    cube.m_indices.push_back(6);
    cube.m_indices.push_back(7);
    cube.m_indices.push_back(4);

    cube.m_indices.push_back(5);
    cube.m_indices.push_back(6);
    cube.m_indices.push_back(4);

    cube.m_indices.push_back(1);
    cube.m_indices.push_back(6);
    cube.m_indices.push_back(5);

    cube.m_indices.push_back(2);
    cube.m_indices.push_back(6);
    cube.m_indices.push_back(1);
    

    
    // cube.Normalize();
    // printf("before normalize\n");

    // tetra.m_positions.push_back(Point3(1,0,0));
    // tetra.m_positions.push_back(Point3(1,1,1));
    // tetra.m_positions.push_back(Point3(0,0,1));
    // tetra.m_positions.push_back(Point3(0,1,0));

    // tetra.m_normals.push_back(Vec3(0,0,0));
    // tetra.m_normals.push_back(Vec3(0,0,0));
    // tetra.m_normals.push_back(Vec3(0,0,0));
    // tetra.m_normals.push_back(Vec3(0,0,0));


    // tetra.m_indices.push_back(0);
    // tetra.m_indices.push_back(1);
    // tetra.m_indices.push_back(2);

    // tetra.m_indices.push_back(3);
    // tetra.m_indices.push_back(1);
    // tetra.m_indices.push_back(0);

    // tetra.m_indices.push_back(2);
    // tetra.m_indices.push_back(1);
    // tetra.m_indices.push_back(3);

    // tetra.m_indices.push_back(2);
    // tetra.m_indices.push_back(3);
    // tetra.m_indices.push_back(0);

    // tetra.Normalize();
     
    
    
    
    
    
    
    
    
    
    
}

Mesh* ToMesh(OctTree& tree){
    Mesh* mesh = new Mesh();
    Point3 translation = Point3(-0.5,-0.5,-0.5);
    stack<TreeNode*> st;
    st.push(tree.root);
    int num_points = 0;
    float x,y,z,mx,my,mz;
    x = y = z = 100000;
    mx = my = mz = -100000;
    while(!st.empty()){
        TreeNode* node = st.top();
        st.pop();
        bool flag = true;
        for(int i = 0;i < 8;i++)
            if(node->son[i] != NULL){
                flag = false;
                break;
            }

        if(flag){
            float length = node->cube[1] - node->cube[0];
            Point3 ori = Point3(node->cube[0],node->cube[2],node->cube[4]);
            for (int l = 0; l < cube.m_positions.size(); l++){
                Point3 temp = cube.m_positions[l]*length;
                temp = temp + ori + translation;
                x = min(x,temp.x);
                mx = max(mx,temp.x);
                y = min(y,temp.y);
                my = max(my,temp.y);
                z = min(z,temp.z);
                mz = max(mz,temp.z);

                mesh->m_positions.push_back(temp);
                mesh->m_normals.push_back(Vec3(0,0,0));
            }

            for (int l = 0; l < cube.m_indices.size(); l++){
                mesh->m_indices.push_back(cube.m_indices[l]+num_points);
            }

            num_points += 8;
        }
        else{
            for(int i = 0;i < 8;i++){
                if(node->son[i] != NULL){
                    st.push(node->son[i]);
                }
            }
        }
    }
    // printf("%f %f %f %f %f %f\n",x,y,z,mx,my,mz);
    return mesh;
}


struct node{
    float val;
    TreeNode *fa;
    int pos;
    node(){}
    node(TreeNode* f, int p, float v):fa(f),val(v),pos(p){}
    bool operator < (const node &b) const {
        int da,db;
        da = fa->depth;
        db = b.fa->depth;
        if(da == db)
            return val < b.val;
        return da > db;
    }
};

void test_octree(){
    // init();
    // OctTree tree;
    // TreeNode* fa = tree.root;
    // for(int i = 0;i < 7;i++){
    //     tree.AddNode(fa,0,1);
    //     fa = tree.AddNode(fa,3,1);
    // }
    // Mesh* mesh = ToMesh(tree);
    // char * filename="hahaha.obj";
    // ExportToObj(filename, mesh);
    // delete mesh;
    // OctTree copy_tree;
    // copy_tree = tree.Copy();
    // mesh = ToMesh(copy_tree);
    // ExportToObj("copy.obj",mesh);
    // delete mesh;
    
    std::priority_queue<node> pq;
    pq.push(node(NULL,1,0));
    pq.push(node(NULL,1,1));
    pq.push(node(NULL,1,2));
    pq.push(node(NULL,1,3));
    while(!pq.empty()){
        printf("%f\n",pq.top().val);
        pq.pop();
    }

}


int main_func(int argc, char *argv[]){
    init();
    printf("Initialize finished!\n");
	int dim = 10;
	
	eval_count = 0;

	// std::vector<double> x0(dim,10.0);
	std::vector<double> x0;

	for(int i = 0;i < dim; i++){
		for(int j = 0;j < dim; j++){
            for(int k = 0;k < dim; k++){
                x0.push_back(0.5);
            }
        }
	}

	double sigma = 0.025;
	
	CMAParameters<> cmaparams(x0, sigma);
	CMASolutions cmasols = cmaes<>(fsphere, cmaparams);
	std::cout << "best solution: "<< cmasols << std::endl;
	std::cout << "optimization took" << cmasols.elapsed_time() / 1000.0 << " seconds\n";
	return cmasols.run_status();
}

float min_score = 0;
int max_depth = 6;

float Evaluation(OctTree& tree){
    int id = eval_count++;
    Mesh *mesh;
    mesh = ToMesh(tree);
    char full_obj_path[100],full_val_path[100],cmd[200];
	sprintf(full_obj_path,"%s/%d.obj",temp_obj_path.c_str(),id);
	sprintf(full_val_path,"%s/%d",temp_val_path.c_str(),id);
	sprintf(cmd,"./NvFlexDemoReleaseCUDA_x64 %s %s",full_obj_path,full_val_path);
	ExportToObj(full_obj_path, mesh);
    
    delete mesh;

    system(cmd);

    FILE* fp = fopen(full_val_path,"r");
	int val;
	
	fscanf(fp,"%d",&val);
	fclose(fp);
    printf("%d ",val);
    
    return val;
}

int mode_index[12][4] = {
    {0,1,2,4},{0,1,3,5},{1,4,5,7},{0,4,5,6},
    {1,2,3,7},{0,2,3,6},{2,4,6,7},{3,5,6,7},
    {0,1,2,3},{4,5,6,7},{0,2,4,6},{1,3,5,7}
    };

std::vector<TreeNode*> addMode(OctTree &tree, TreeNode* fa, int mode, float val){
    std::vector<TreeNode*> res;
    if(mode==12){
        printf("Delete Node ");
        tree.DeleteNode(fa);
    }
    else{
        for(int i = 0;i < 4;i++){
            res.push_back(tree.AddNode(fa,mode_index[mode][i],val));
        }
    }
    return res;
}

void search(){
    init();
    OctTree tree;
    std::priority_queue<node> pq;
    // std::vector<node> pq;

    for(int i = 0;i < 12;i++)
        pq.push(node(tree.root,i,1));
    
    int loop = 0;
    printf("before optimize!\n");

    while(!pq.empty() && loop < 10000){
        // sort(pq.begin(),pq.end());
        std::priority_queue<node> tpq = pq;
        while(!tpq.empty()){
            node tnode = tpq.top();
            tpq.pop();
            printf("val %f mod %d depth %d\n",tnode.val,tnode.pos,tnode.fa->depth+1);
        } 
        // printf("\n");
        node top = pq.top();
        pq.pop();
        TreeNode* fa = top.fa;
        float val;
        val = top.val - fa->val;

        printf("value %f base %f depth %d mode %d\n",val,fa->val,fa->depth+1,top.pos);
        if(val > min_score){
            // fa = tree.AddNode(fa,top.pos,top.val);
            std::vector<TreeNode*> newv = addMode(tree,fa,top.pos,top.val);
            printf("Added.\n");
            for(int j = 0;j < newv.size();j++){
                fa = newv[j];
                if(fa->depth < max_depth){
                    float best_score = -10000;
                    int best_ans;
                    for(int i = 0;i < 13;i++){
                        
                        OctTree tmp_tree = tree.Copy();
                        // tmp_tree.AddNode(fa,i,0);
                        addMode(tmp_tree,fa,i,0);
                        float score = Evaluation(tmp_tree);
                        // printf("before delete\n");
                        tmp_tree.DeleteNode(tmp_tree.root);
                        if(best_score<score){
                            best_score = score;
                            best_ans = i;
                        }
                        // printf("after delete\n");
                    }
                    pq.push(node(fa,best_ans,best_score));
                }
            }
        }
        else{// The second chance to expand this vertex
            OctTree tmp_tree = tree.Copy();
            // tmp_tree.AddNode(fa,top.pos,top.val);
            addMode(tmp_tree,fa,top.pos,top.val);
            float score = Evaluation(tmp_tree);
            tmp_tree.DeleteNode(tmp_tree.root);

            if(score > min_score)
                pq.push(node(fa,top.pos,score));
            
        }
        printf("\n");
        loop++;
    }
}

int main(int argc, char *argv[]){
    search();
    // test_octree();
	// main_func(argc,argv);
	// test();
	// TestIntersect();
}
