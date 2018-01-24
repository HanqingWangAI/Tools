#encoding=utf-8
import os
import math
import numpy as np
import cma
from utils import *

INF = 1e12
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
sigma = 0.1


def reg(x):
    res = 0
    num = x.shape[0]
    for i in range(num):
        v1 = Vector(Point(x[i,0],x[i,1]),Point(x[(i+1)%num,0],x[(i+1)%num,1]))
        v2 = Vector(Point(x[(i+1)%num,0],x[(i+1)%num,1]),Point(x[(i+2)%num,0],x[(i+2)%num,1]))
        l1 = get_left(v1)
        l2 = get_left(v2)
        res += inner_product(l1,l2)

    return res

def area(x):
    res = 0
    num = x.shape[0]
    for i in range(num):
        res += x[i,0]*x[(i+1)%num,1]-x[(i+1)%num,0]*x[i,1]
    res /= 2
    if res < 0:
        res = -res

    return res

def reg_dis(x):
    num = x.shape[0]
    res = []
    for i in range(num):
        for j in range(i+1,num):
            res.append(dis(Point(x[i,0],x[i,1]),Point(x[j,0],x[j,1])))
    res = np.min(res)
    return res

def objfunc(x):
    global num_func, num_affo

    res = 0
    func_part = np.array(x[:num_func*2]).reshape([-1, 2])
    affo_part = np.array(x[(num_func-2)*2:-3]).reshape([-1, 2])
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
    # print(fa_fb)
    if fa_fb < 0:
        res += INF
    else:
        res -= fa_fb

    dis = reg_dis(np.reshape(x[:32],[-1,2]))


    res += reg(func_part)
    res += reg(affo_part)
    res += 0.1*area(affo_part)
    res += 0.1*area(func_part)
    res += (np.maximum(np.max(func_part), np.max(affo_part)) - np.minimum(np.min(func_part),
                                                                          np.min(affo_part))) / dis
    if dis < 0.01:
        res += INF
    return res

def main():
    es = cma.CMAEvolutionStrategy(x,sigma,{'verb_disp':100})
    es.optimize(objfunc)

if __name__ == '__main__':
    main()
