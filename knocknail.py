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
      6   #sampled affordance base
     ]
sigma = 0.5


def reg(x):
    res = 0
    num = x.shape[0]
    for i in range(num):
        v1 = Vector(Point(x[i,0],x[i,1]),Point(x[(i+1)%num,0],x[(i+1)%num,1]))
        v2 = Vector(Point(x[(i+1)%num,0],x[(i+1)%num,1]),Point(x[(i+2)%num,0],x[(i+2)%num,1]))
        l1 = get_left(v1)
        l2 = get_left(v2)
        res += inner_product(l1,l2)

    res *= -1
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
    res_min = np.min(res)
    res_max = np.max(res)
    return res_min, res_max

def objfunc(x):
    global num_func, num_affo
    data_term = 0
    reg_term = 0
    func_part = np.array(x[:num_func*2]).reshape([-1, 2])
    affo_part = np.array(x[(num_func-2)*2:-3]).reshape([-1, 2])
    sample_points = x[-3:]

    if not check(func_part):
        data_term += INF
    if not check(affo_part):
        data_term += INF
    if check_intersection(func_part,affo_part) or check_intersection(affo_part,func_part):
        data_term += INF

    func_idx = round(sample_points[0], num_func)
    fulc_idx = round(sample_points[1], num_func)
    affo_idx = round(sample_points[2], num_affo)

    if func_idx == num_func - 2:
        data_term += INF
    if affo_idx == 0:
        data_term += INF


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

    fa_fb = cross_product(b, lb)  / ((cross_product(a, la) - ZERO) * np.fabs(inner_product(b, get_left(lb))))
    # print(fa_fb)
    if fa_fb < 0:
        data_term += INF
    else:
        data_term += fa_fb

    dis_min, dis_max = reg_dis(np.reshape(x[:32], [-1, 2]))


    reg_term += 0.1*reg(func_part)
    reg_term += 0.1*reg(affo_part)
    reg_term += 0.1*area(affo_part)
    reg_term += 0.1*area(func_part)
    reg_term += (np.maximum(np.max(func_part), np.max(affo_part)) - np.minimum(np.min(func_part),
                                                                          np.min(affo_part))) / dis_min
    if dis_min < 1:
        data_term += INF
    if dis_max >20:
        data_term += INF
    res = data_term + np.fabs(data_term)*reg_term/(np.fabs(data_term)+np.fabs(reg_term))
    return math.log(res)

def main():
    es = cma.CMAEvolutionStrategy(x,sigma,{'verb_disp':100})
    es.optimize(objfunc)

def test_obj(x):
    global num_func, num_affo
    reg_part = 0
    area_part = 0
    ratio_part = 0
    res = 0
    func_part = np.array(x[:num_func * 2]).reshape([-1, 2])
    affo_part = np.array(x[(num_func - 2) * 2:-3]).reshape([-1, 2])
    sample_points = x[-3:]

    if not check(func_part):
        res += INF
    if not check(affo_part):
        res += INF
    # print check_intersection(func_part,affo_part) and check_intersection(affo_part,func_part)


    func_idx = round(sample_points[0], num_func)
    fulc_idx = round(sample_points[1], num_func)
    affo_idx = round(sample_points[2], num_affo)

    if func_idx == num_func - 2:
        res += INF
    if affo_idx == 0:
        res += INF

    mid_func = (func_part[func_idx] + func_part[(func_idx + 1) % num_func]) / 2
    mid_affo = (affo_part[affo_idx] + affo_part[(affo_idx + 1) % num_affo]) / 2
    mid_fulc = func_part[fulc_idx]

    vec_func = Vector(Point(func_part[func_idx][0], func_part[func_idx][1]),
                      Point(func_part[(func_idx + 1) % num_func][0], func_part[(func_idx + 1) % num_func][1]))
    vec_affo = Vector(Point(affo_part[affo_idx][0], affo_part[affo_idx][1]),
                      Point(affo_part[(affo_idx + 1) % num_affo][0], affo_part[(affo_idx + 1) % num_affo][1]))
    # print vec_func.x,vec_func.y
    b = get_left(vec_func)
    a = get_left(vec_affo)
    # print ('b',(b.x,b.y),'a',(a.x,a.y))

    lb = Vector(Point(mid_func[0], mid_func[1]), Point(mid_fulc[0], mid_fulc[1]))
    la = Vector(Point(mid_affo[0], mid_affo[1]), Point(mid_fulc[0], mid_fulc[1]))

    fa_fb = cross_product(b, lb) / (cross_product(a, la) - ZERO)
    ratio_part += fa_fb
    # print(fa_fb)
    if fa_fb < 0:
        res += INF
    else:
        res += fa_fb

    dis_min, dis_max = reg_dis(np.reshape(x[:32], [-1, 2]))

    reg_part += reg(func_part)
    reg_part += reg(affo_part)
    area_part += area(affo_part)
    area_part += area(func_part)
    res += (np.maximum(np.max(func_part), np.max(affo_part)) - np.minimum(np.min(func_part),
                                                                          np.min(affo_part))) / dis_min
    if dis_min < 0.1:
        res += INF
    return ratio_part, area_part, reg_part, dis_min, dis_max, check_intersection(func_part,affo_part) or check_intersection(affo_part, func_part)

if __name__ == '__main__':
    main()
    # print test_obj(x)
