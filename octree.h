
#pragma once

#include <iostream>
#include <cstring>
#include <algorithm>
#include <vector>
#include <assert.h>
#include <stack>

#include "mesh.h"

using namespace std;


class TreeNode{
    public:
        TreeNode(){
            leaf = true;
            for(int i = 0;i < 8;i++) son[i] = NULL;
            fa = NULL;
            depth = 0;
            val = 0;
        }

        ~TreeNode(){
            
        }

        float cube[6];
        float val;
        TreeNode *fa;
        TreeNode *son[8];
        bool leaf = true;
        int depth = 0;
        int id = 0;
        
};

class OctTree{
    public:
        OctTree(){
            root = new TreeNode();
            for(int i = 0;i < 3;i++){
                root->cube[i*2] = 0.0;
                root->cube[i*2+1] = 1.0;
            }
            root->depth = 0;
        }

        ~OctTree(){
            // DeleteNode(root);
        }

        OctTree Copy(){
            OctTree res;
            stack<TreeNode*> st,stn;
            st.push(root);
            stn.push(res.root);
            while(!st.empty()){
                TreeNode* tn = st.top(), *tnn = stn.top();
                st.pop();
                stn.pop();

                for(int i = 0;i < 8;i++){
                    TreeNode* s = tn->son[i];
                    if(s != NULL){
                        st.push(s);
                        s = res.AddNode(tnn,i,s->val);
                        stn.push(s);
                    }
                }
            }
            return res;
        }

        TreeNode* root;

        TreeNode* AddNode(TreeNode *fa, int po, float val){
            assert(po < 8 && po >= 0);
            assert(fa != NULL);
            fa->leaf = false;
            if(fa->son[po] != NULL) {
                // fa->son[po]->val = val;
                printf("OK");
                return fa->son[po];
            }

            TreeNode* tmp = new TreeNode();
            tmp->val = val;
            tmp->id = po;
            fa->son[po] = tmp;
            tmp->fa = fa;
            tmp->depth = fa->depth + 1;
            for(int i = 0;i < 3;i++){
                if(po&(1<<i)){
                    tmp->cube[2*i+1] = fa->cube[2*i+1];
                    tmp->cube[2*i] = (fa->cube[2*i]+fa->cube[2*i+1])/2;
                }
                else{
                    tmp->cube[2*i+1] = (fa->cube[2*i]+fa->cube[2*i+1])/2;
                    tmp->cube[2*i] = fa->cube[2*i];
                }
            }
            return tmp;
        }

        void DeleteNode(TreeNode *a){
            if(a == NULL) return;
            for(int i = 0;i < 8;i++){
                DeleteNode(a->son[i]);
            }
            int id = a->id;
            if(a->fa != NULL)
                a->fa->son[id] = NULL;
            delete a;
        }
        
        
};


