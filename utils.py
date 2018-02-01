import math
import numpy as np
ZERO = -1e-9

class Point(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y

class Vector(object):

    def __init__(self, A ,B):
        self.start_point = A
        self.end_point = B
        self.x = B.x - A.x
        self.y = B.y - A.y
        self.mod = math.sqrt(self.x*self.x+self.y*self.y)


# orthogonal vector, y=-1 for right direction x = b/a, y = 1 for left direction, x = -b/a

def negative(vector):
    return Vector(vector.end_point, vector.start_point)

def cross_product(vectorA, vectorB):
    return vectorA.x * vectorB.y - vectorB.x * vectorA.y

def inner_product(vectorA, vectorB):
    return vectorA.x*vectorB.x + vectorA.y*vectorB.y

def dis(A, B):
    v = Vector(A, B)
    return v.mod

def is_intersected(A, B, C, D):
    AC = Vector(A, C)
    AD = Vector(A, D)
    BC = Vector(B, C)
    BD = Vector(B, D)
    CA = negative(AC)
    CB = negative(BC)
    DA = negative(AD)
    DB = negative(BD)

    return (cross_product(AC, AD) * cross_product(BC, BD) <= ZERO) \
        and (cross_product(CA, CB) * cross_product(DA, DB) <= ZERO)

def check(points):
    num = len(points)
    flag = False
    for i in range(num):
        A = Point(points[i, 0], points[i, 1])
        B = Point(points[(i+1) % num, 0], points[(i+1) % num, 1])

        for j in range(i+2,num):
            C = Point(points[j,0], points[j,1])
            D = Point(points[(j + 1) % num, 0], points[(j + 1) % num, 1])
            if is_intersected(A,B,C,D):
                flag = True
                # print((A.x,A.y),(B.x,B.y),(C.x,C.y),(D.x,D.y))

                break
        if flag:
            break

    return not flag

def check_intersection(A, B):
    num_B = B.shape[0]

    for c in A:
        pre = 0
        mark = True
        for i in range(num_B):
            a = B[i]
            b = B[(i+1)%num_B]
            vc = Vector(Point(a[0],a[1]),Point(c[0],c[1]))
            vb = Vector(Point(a[0],a[1]),Point(b[0],b[1]))
            res = cross_product(vc,vb)
            # print c,i,pre,res
            if pre != 0:
                if pre * res < 0:
                    # print c,i
                    mark = False
                    break
            if res < 0:
                pre = -1
            if res > 0:
                pre = 1
        if mark:
            return True
    return False

def norm(x):
    length = math.sqrt(x.x*x.x+x.y*x.y)
    return Vector(Point(0, 0), Point(x.x/length, x.y/length))

def get_left(x):
    a = x.x
    b = x.y
    if math.fabs(a) < math.fabs(ZERO):
        res = Vector(Point(0,0), Point(-1, 0))
    else:
        res = norm(Vector(Point(0, 0), Point(-b / a, 1)))
    if cross_product(res,x) > 0:
        res = negative(res)
    return res

def get_right(x):
    a = x.x
    b = x.y
    if math.fabs(a) < math.fabs(ZERO):
        res = Vector(Point(0, 0), Point(-1, 0))
    else:
        res = norm(Vector(Point(0, 0), Point(-b / a, 1)))
    if cross_product(res, x) < 0:
        res = negative(res)
    return res

# xx = [0,0,2,0,2,-2,1,2]
# print check(xx)
# xx = [0,0,2,0,2,-2,0,-2]
# print check(xx)
# A = Point(1,0)
# B = Point(1,1)
# C = Point(0,0)
# D = Point(0,1)
# print is_intersected(A,B,C,D)
# print is_intersected(A,D,B,C)

def round(x,n):
    res = np.round(x).astype(np.int32)
    while res < 0:
        res += n
    res %= n
    return res
