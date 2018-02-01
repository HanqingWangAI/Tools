import matplotlib.pyplot as plt
import numpy as np
from utils import *
import imageio
from PIL import Image

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
    parts = np.array(x[:32]).reshape([-1, 2])
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



    ax.plot([func_part[func_idx, 0], func_part[(func_idx + 1) % num_func, 0]],
            [func_part[func_idx, 1], func_part[(func_idx + 1) % num_func, 1]], color='g')

    ax.plot([affo_part[affo_idx, 0], affo_part[(affo_idx + 1) % num_affo, 0]],
            [affo_part[affo_idx, 1], affo_part[(affo_idx + 1) % num_affo, 1]], color='black')

    ax.plot(parts[:, 0], parts[:, 1], '.')

    ax.plot([func_part[fulc_idx,0]],[func_part[fulc_idx,1]],'o')

    plt.xlim((-10,10))
    plt.ylim((-10,10))
    # plt.show()
    plt.savefig('temp.png',format='png',dpi=100)
    plt.close()




def main():
    from knocknail import test_obj
    from knocknail import objfunc
    path = 'outcmaesxrecentbest.dat'
    imgs = []
    y_axis = []
    with open(path,'r') as fp:
        xx = fp.readline()
        while xx != '':
            # print(xx)
            contents = xx.split()
            xx = fp.readline()
            if contents[0] == '%':
                continue
            contents = contents[5:]
            contents = [float(i) for i in contents]
            # plot(contents)
            # img = np.asarray(Image.open('temp.png'))
            # imgs.append(img)
            # y_axis.append(test_obj(contents))
            ratio, area, reg,_,_,_ = test_obj(contents)
            y_axis.append([objfunc(contents),ratio,area,reg])
    # imageio.mimsave('new.gif', imgs, duration=0.05)
    for x in y_axis:
        print x
    y_axis = np.array(y_axis)
    fig,ax = plt.subplots()
    x_axis = range(len(y_axis))
    # plt.plot(x_axis,y_axis[:,0],label='energy')
    plt.plot(x_axis,y_axis[:,1],label='proportion')
    # plt.plot(x_axis,y_axis[:,2],label='area')
    # plt.plot(x_axis,y_axis[:,3],label='smooth')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
    # plot(x)