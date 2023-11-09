from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import time
from IPython import display
import matplotlib.pylab as plt
import pandas as pd
from robot_arm import RobotArm
from matplotlib.animation import FuncAnimation


import time


def get_mid_pos(l1, th1, origin):
    x = origin[0] + l1 * cos(th1)
    y = origin[1] +l1 * sin(th1)
    return x, y

def get_eff_pos(l1, l2, th1, th2, origin):
    x =  origin[0]+l1 * cos(th1) + l2 * cos(th1 + th2)
    y =  origin[1]+l1 * sin(th1) + l2 * sin(th1 + th2)
    return x, y

def Jacobian(l1, l2, th1, th2):
  J11 = -l1*sin(th1) - l2*sin(th1+th2)
  J21 = l1*cos(th1) + l2*cos(th1+th2)
  J12 = - l2*sin(th1+th2)
  J22 = l2*cos(th1+th2)
  J = np.array([[J11, J12], [J21, J22]])
  return J

def get_next_angles_transpose(l1, l2, th1, th2, tx, ty):
  t = np.array([tx, ty])
  x2, y2 = get_eff_pos(l1, l2, th1, th2, midpoint_origin)
  mypos = np.array([x2, y2])
  J = Jacobian(l1, l2, th1, th2)
  angle_diff = 0.002*J.transpose().dot(t - mypos)
  ret1 = th1 + angle_diff[0]
  ret2 = th2 + angle_diff[1]
  return ret1, ret2

def get_next_angles_inverse(l1, l2, th1, th2, tx, ty):
  t = np.array([tx, ty])
  x2, y2 = get_eff_pos(l1, l2, th1, th2, midpoint_origin)
  mypos = np.array([x2, y2])
  J = Jacobian(l1, l2, th1, th2)
  #print("J is ", J)
  angle_diff = np.linalg.inv(J).dot(t - mypos)
  ret1 = th1 + angle_diff[0]
  ret2 = th2 + angle_diff[1]
  print("this angles' inverse diff is ", angle_diff)
  return ret1, ret2, J

def get_next_angles(th1, th2):
  th1 = th1 + 0.4*(np.random.random() - 0.5)
  th2 = th2 + 0.4*(np.random.random() - 0.5)
  print("this angle is ", th1, th2)
  return th1, th2

def velocity(l1, l2, th1, th2, tx, ty):
    t = np.array([tx, ty])
    x2, y2 = get_eff_pos(l1, l2, th1, th2)
    mypos = np.array([x2, y2])
    J = Jacobian(l1, l2, th1, th2)
    angular_velocity = J.dot(t - mypos)
    print("angular v is ", angular_velocity)
    return angular_velocity

def acceleration(l1, l2, th1, th2, tx, ty):
    t = np.array([tx, ty])
    x2, y2 = get_eff_pos(l1, l2, th1, th2)
    mypos = np.array([x2, y2])
    J = Jacobian(l1, l2, th1, th2)
    J_dot = np.gradient(J, axis=1)
    angular_acceleration = J_dot.dot(t - mypos)
    print("acceleration is ", acceleration)
    return angular_acceleration



fig = plt.figure()
ax = fig.subplots()
ARM_COLOR = (10/255, 180/255, 200/255, 0.8)

DISPLAY_WIDTH = 300
DISPLAY_HEIGHT = 300

ax.set_xlim(0, DISPLAY_WIDTH)
ax.set_ylim(0, DISPLAY_HEIGHT)
ax.set_aspect('equal', adjustable='box')

midpoint_origin = np.array([DISPLAY_WIDTH / 2, 0])
robot_arm = RobotArm(np.array([DISPLAY_WIDTH / 2, DISPLAY_HEIGHT]), ax)
robot_arm.set_angular_velocity(2, 0)
robot_arm.update(1)
robot_arm.draw()



#angular_velocity = velocity(l1, l2, th1, th2, tx, ty)
#angular_acceleration = acceleration(l1, l2, th1, th2, tx, ty)

l1 = 50
l2 = 50
th1 = 0.63589064565 * (pi / 180)
th2 =  2.25176423609* (pi / 180)

tx = 225
ty = 70

midpoint_origin = np.array([DISPLAY_WIDTH / 2, 0])

start_time = time.time()

for i in range(10):
    x1, y1 = get_mid_pos(l1, th1, midpoint_origin)
    x2, y2 = get_eff_pos(l1, l2, th1, th2, midpoint_origin)
    #print("end points 1 are", x1, y1)
    #print("end points 2 are", x2, y2)

    if abs(x2 - tx) + abs(y2 - ty) < 0.01:
        #ani.event_source.stop()  # Stop the animation when the condition is met
        #return
        break

    ax.cla()  # clean up the ax but not the fig
    ax.set_xlim(0, DISPLAY_WIDTH)
    ax.set_ylim(0, DISPLAY_HEIGHT)
    ax.set_aspect('equal', adjustable='box')
    plt.plot(tx, (300-ty), marker = 'x')
    #th1, th2 = get_next_angles(l1, l2, th1, th2, tx, ty)
    #print("th1 is", th1)
    prev_th1 = th1
    prev_th2 = th2
    th1, th2, J = get_next_angles_inverse(l1, l2, th1, th2, tx, ty)
    # Specify the target velocities for each joint
    # Specify the target position for the end effector
    target_position = np.array([tx, ty])
    current_position = np.array([x2, y2])
    #print("current loc is", current_position)
    #print("target loc is", target_position)

    # Your code to update joint angles goes here
    # Measure the time elapsed for this iteration
    # Measure the time elapsed for this iteration

    # Your code to calculate joint velocity goes here
    '''
    theta_current = np.array([th1, th2])
    theta_previous = np.array([prev_th1, prev_th2])
    # Calculate time elapsed
    delta_t = time.time() - start_time
    joint_velocity = (theta_current - theta_previous) / delta_t
    theta_dot = np.array([joint_velocity[0], joint_velocity[1]])


    # Calculate angular velocity using the Jacobian
    angular_velocity = J.dot(theta_dot)
    print("angular velocity is ", angular_velocity)
    '''

    delta_t = time.time() - start_time
    
    target_velocity = J.dot(target_position - current_position)

    J_inv = np.linalg.pinv(J)

    angular_velocity = J_inv.dot(target_velocity)
    joint1_velocity = angular_velocity[0]
    joint2_velocity = angular_velocity[1]   
    

    # Call the set_angular_velocity_from_jacobian method
    robot_arm.set_angular_velocity(angular_velocity[0], 0)
    robot_arm.update(1)
    robot_arm.set_angular_velocity(angular_velocity[1], 1)
    robot_arm.update(delta_t)
    robot_arm.draw()
    plt.savefig(f'frame_{i}.png')

'''
ani = FuncAnimation(fig, update, fargs=(l1, l2, th1, th2, tx, ty), repeat=False, interval=100)  # 绘制动画
plt.show()
'''