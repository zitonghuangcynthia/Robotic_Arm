from math import *
import numpy as np




def get_mid_pos(l1, th1):
  x = l1*cos(th1)
  y = l1*sin(th1)
  return x,y

def get_eff_pos(l1, l2, th1, th2):
  x = l1*cos(th1) + l2*cos(th1+th2)
  y = l1*sin(th1) + l2*sin(th1+th2)
  return x,y

def Jacobian(l1, l2, th1, th2):
  J11 = -l1*sin(th1) - l2*sin(th1+th2)
  J21 = l1*cos(th1) + l2*cos(th1+th2)
  J12 = - l2*sin(th1+th2)
  J22 = l2*cos(th1+th2)
  J = np.array([[J11, J12], [J21, J22]])
  return J

def get_next_angles_transpose(l1, l2, th1, th2, tx, ty):
  t = np.array([tx, ty])
  x2, y2 = get_eff_pos(l1, l2, th1, th2)
  mypos = np.array([x2, y2])
  J = Jacobian(l1, l2, th1, th2)
  angle_diff = 0.002*J.transpose().dot(t - mypos)
  ret1 = th1 + angle_diff[0]
  ret2 = th2 + angle_diff[1]
  return ret1, ret2

def get_next_angles_inverse(l1, l2, th1, th2, tx, ty):
  t = np.array([tx, ty])
  x2, y2 = get_eff_pos(l1, l2, th1, th2)
  mypos = np.array([x2, y2])
  J = Jacobian(l1, l2, th1, th2)
  angle_diff = np.linalg.inv(J).dot(t - mypos)
  ret1 = th1 + angle_diff[0]
  ret2 = th2 + angle_diff[1]
  print("the difference in angle is", angle_diff)
  return ret1, ret2


def get_next_angles(th1, th2):
  th1 = th1 + 0.4*(np.random.random() - 0.5)
  th2 = th2 + 0.4*(np.random.random() - 0.5)
  return th1, th2

import matplotlib.pylab as plt
import pandas as pd

import time
from IPython import display

l1 = 50
l2 = 50
th1 = 0.1*(pi/180)
th2 = 0.1*(pi/180)


tx = 75
ty = 70
for i in range(10):
        #print("th1 is", th1)
        x1, y1 = get_mid_pos(l1, th1)
        x2, y2 = get_eff_pos(l1, l2, th1, th2)

        if abs(x2-tx)+abs(y2-ty) < 0.01:
          break

        display.clear_output(wait=True)
        plt.cla()

        #th1, th2 = get_next_angles(th1, th2)
        th1, th2 = get_next_angles_inverse(l1, l2, th1, th2, tx, ty)

        plt.plot([0, x1, x2], [0, y1, y2])
        plt.scatter([0, x1], [0, y1], s = 200)
        plt.scatter(x2, y2, s = 200, marker = '>')

        plt.plot(tx, ty, marker = 'x')

        plt.xlim([-1, 300])
        plt.ylim([-1, 300])
        plt.savefig(f'frame_tttt{i}.png')

        #display.display(plt.gcf())

        #time.sleep(0.3)

