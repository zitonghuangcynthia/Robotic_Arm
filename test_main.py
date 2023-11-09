import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from robot_arm import RobotArm
from matplotlib.animation import FuncAnimation




'''
robot_arm = RobotArm(np.array([DISPLAY_WIDTH / 2, DISPLAY_HEIGHT]), ax)
#robot_arm.set_angular_velocity(2, 1)
#if robot_arm.rotating[1]:
    #robot_arm.set_angular_velocity(0, 1)
#robot_arm.update(1)
robot_arm.draw()
plt.show()
'''

#robot_arm = RobotArm(np.array([DISPLAY_WIDTH / 2, DISPLAY_HEIGHT]), ax)
'''
robot_arm.set_angular_velocity(2, 0)
if robot_arm.rotating[1]:
    robot_arm.set_angular_velocity(0, 0)
robot_arm.update(1)
robot_arm.draw()
robot_arm.set_angular_velocity(2, 0)
if robot_arm.rotating[1]:
    robot_arm.set_angular_velocity(0, 0)
robot_arm.update(1)
robot_arm.draw()
robot_arm.set_angular_velocity(2, 0)
if robot_arm.rotating[1]:
    robot_arm.set_angular_velocity(0, 0)
robot_arm.update(1)
robot_arm.draw()
if robot_arm.rotating[1]:
    robot_arm.set_angular_velocity(0, 0)
robot_arm.update(1)
robot_arm.draw()
if robot_arm.rotating[1]:
    robot_arm.set_angular_velocity(0, 0)
robot_arm.update(1)
robot_arm.draw()
if robot_arm.rotating[1]:
    robot_arm.set_angular_velocity(0, 0)
robot_arm.update(1)
robot_arm.draw()

plt.show()
'''
fig = plt.figure()
ax = fig.subplots()
ARM_COLOR = (10/255, 180/255, 200/255, 0.8)
DISPLAY_WIDTH = 300
DISPLAY_HEIGHT = 300

ax.set_xlim(0, DISPLAY_WIDTH)
ax.set_ylim(0, DISPLAY_HEIGHT)
ax.set_aspect('equal', adjustable='box')

robot_arm = RobotArm(np.array([DISPLAY_WIDTH / 2, DISPLAY_HEIGHT]), ax)

'''
robot_arm.set_angular_velocity(2, 0)
robot_arm.update(1)
robot_arm.draw()
plt.savefig(f'frame_{0}.png')  # 保存每一帧图像为单独的文件

plt.clf()
ax.set_xlim(0, DISPLAY_WIDTH)
ax.set_ylim(0, DISPLAY_HEIGHT)
ax.set_aspect('equal', adjustable='box')
robot_arm.set_angular_velocity(2, 0)
robot_arm.update(1)
robot_arm.draw()
plt.savefig(f'frame_{1}.png')  # 保存每一帧图像为单独的文件
'''
def update(h):
    #for i in range(h):
    print("change")
    robot_arm.set_angular_velocity(2, 0)
    ax.cla()  
    ax.set_xlim(0, DISPLAY_WIDTH)
    ax.set_ylim(0, DISPLAY_HEIGHT)
    ax.set_aspect('equal', adjustable='box')

    robot_arm.update(1)
    robot_arm.draw()  
    plt.savefig(f'frame_{h}.png')  




ani=FuncAnimation(fig,update,frames=1, repeat=False, interval=100) #绘制动画
plt.show()

