import matplotlib.pyplot as plt
import numpy as np
from utils import *

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

def plot(x):
    global num_affo, num_func

    func_part = np.array(x[:num_func * 2]).reshape([-1, 2])
    affo_part = np.array(x[(num_func-2) * 2:-3]).reshape([-1, 2])
    sample_points = x[-3:]
    print(func_part.shape,affo_part.shape)

    func_idx = round(sample_points[0], num_func)
    fulc_idx = round(sample_points[1], num_func)
    affo_idx = round(sample_points[2], num_affo)

    fig, ax = plt.subplots()
    for i in range(num_func):
        v1 = func_part[i]
        v2 = func_part[(i+1)%num_func]
        ax.plot([v1[0],v2[0]],[v1[1],v2[1]],color='r')

    for i in range(num_affo):
        v1 = affo_part[i]
        v2 = affo_part[(i+1)%num_affo]
        ax.plot([v1[0],v2[0]],[v1[1],v2[1]],color='b')

    plt.xlim((-10,10))
    plt.ylim((-10,10))
    plt.show()


plot(x)