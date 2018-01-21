#encoding=utf-8
import os
import math
import numpy as np
import lcmaes
from utils import *

INF = 1000000000
num_func = 8
num_affo = 10

x = [-3, 0,
     -3, 2,
     -1, 2,
      1, 2,
      3, 2,
      3, 0,
      1, 0,
     -1, 0,
     -1,-2,
     -1,-4,
     -1,-6,
     -1,-8,
      1,-8,
      1,-6,
      1,-4,
      1,-2,
      7,  #sampled function base
      3,  #sampled fulcrum base
      5   #sampled affordance base
     ]
olambda = 10 # lambda is a reserved keyword in python, using olambda instead.
seed = 0 # 0 for seed auto-generated within the lib.
sigma = 0.1
p = lcmaes.make_simple_parameters(x,sigma,olambda,seed)



# orthogonal vector, y=-1 for right direction x = b/a, y = 1 for left direction, x = -b/a

def negative(vector):
    return Vector(vector.end_point, vector.start_point)

def cross_product(vectorA, vectorB):
    return vectorA.x * vectorB.y - vectorB.x * vectorA.y

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
                print((A.x,A.y),(B.x,B.y),(C.x,C.y),(D.x,D.y))

                break
        if flag:
            break

    return not flag

def norm(x):
    length = math.sqrt(x.x*x.x+x.y*x.y)
    return Vector(Point(0, 0), Point(x.x/length, x.y/length))


def get_left(x):
    a = x.x
    b = x.y
    if math.fabs(a) < math.fabs(ZERO):
        return Vector(Point(0,0), Point(-1, 0))

    return norm(Vector(Point(0, 0), Point(-b / a, 1)))

def get_right(x):
    a = x.x
    b = x.y
    if math.fabs(a) < math.fabs(ZERO):
        return Vector(Point(0, 0), Point(1, 0))

    return norm(Vector(Point(0, 0), Point(b / a, -1)))

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
    res = np.round(x)
    while res < 0:
        res += n
    res %= n
    return res

def objfunc(x,n):
    global num_func, num_affo

    res = 0
    func_part = np.array(x[:num_func*2]).reshape([-1, 2])
    affo_part = np.array(x[num_func*2:-3]).reshape([-1, 2])
    sample_points = x[-3:]

    if not check(func_part):
        res += INF
    if not check(affo_part):
        res += INF

    func_idx = round(sample_points[0], num_func)
    fulc_idx = round(sample_points[1], num_func)
    affo_idx = round(sample_points[2], num_affo)

    if func_idx == num_func - 2:
        res += INF
    if affo_idx == 0:
        res += INF

    mid_func = (func_part[func_idx] + func_part[(func_idx+1) % num_func]) / 2
    mid_affo = (affo_part[affo_idx] + affo_part[(affo_idx+1) % num_affo]) / 2
    mid_fulc = func_part[fulc_idx]

    vec_func = Vector(Point(func_part[func_idx][0], func_part[func_idx][1]),
                      Point(func_part[(func_idx+1) % num_func][0], func_part[(func_idx+1) % num_func][1]))
    vec_affo = Vector(Point(affo_part[affo_idx][0], affo_part[affo_idx][1]),
                      Point(affo_part[(affo_idx+1) % num_affo][0], affo_part[(affo_idx+1) % num_affo][1]))

    b = get_left(vec_func)
    a = get_left(vec_affo)


    lb = Vector(Point(mid_func[0], mid_func[1]), Point(mid_fulc[0], mid_fulc[1]))
    la = Vector(Point(mid_affo[0], mid_affo[1]), Point(mid_fulc[0], mid_fulc[1]))

    fa_fb = cross_product(b,lb)/(cross_product(a,la)-ZERO)
    print(fa_fb)



    return res

objfunc = lcmaes.fitfunc_pbf.from_callable(objfunc);

# pass the function and parameter to cmaes, run optimization and collect solution object.
cmasols = lcmaes.pcmaes(objfunc,p)

# collect and inspect results
bcand = cmasols.best_candidate()
bx = lcmaes.get_candidate_x(bcand)
print "best x=",bx

# print fitfunc(x,3)


# #
# print "distribution mean=",lcmaes.get_solution_xmean(cmasols)
# cov = lcmaes.get_solution_cov(cmasols) # numpy array
# # print "cov=",cov
# # print "elapsed time=",cmasols.elapsed_time(),"ms"
