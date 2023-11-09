import numpy as np

from robot_parts import BasePlate, Claw
from robot_joint import RobotJoint

import numpy as np
import matplotlib.pyplot as plt

START_ROTATIONS = [-30,-60]

class RobotArm:
    def __init__(self, pos : np.ndarray([0,0]), ax=None):
        self.ax = ax if ax is not None else plt.gca()
        #print("build base")
        self.base = BasePlate(pos, ax=self.ax)
        self.joints = []
        #print("build joint1")
        self.joints.append(RobotJoint(ax=self.ax, parent=self.base, name="First Joint", pos=[0,0]))
        #print("build joint2")
        self.joints.append(RobotJoint(ax=self.ax, parent=self.joints[0].children[-1], name="Second Joint", pos=self.joints[0].edge))
        #print("build claw")
        self.claw = Claw(ax=self.ax, parent=self.joints[1].children[-1], name="Claw", pos=self.joints[1].edge)
        self.num_joints = len(self.joints)
        self.rotating = [False] * self.num_joints
        for i in range(self.num_joints):
            self.joints[i].rotate(START_ROTATIONS[i])

    def move_ip(self, x: float = 0, y: float = 0) -> None:
        self.state[0][0] += x; self.state[0][1] += y
    
    def set_angular_velocity(self, angular_vel:float, joint:int=0):
        #self.ax.patches = []
        if angular_vel:
            self.rotating[joint] = True
        self.joints[joint].set_angular_vel(angular_vel)
        # pass

    def rotate(self, rotation : float, joint : int = 0):
        # pass
        self.joints[joint].rotate(rotation)
        # self.claw.rotate(-rotation)
        # self.base.rotate(rotation)
    
    def update(self, delta_t : float) -> None:
        for i in range(self.num_joints):
            self.joints[i].update(delta_t)
            #print(f"joint[{i}]:\n{self.joints[i].state}")

    def draw(self):
        #print("draw base") 
        #print("draw base")
        self.base.draw(self.ax)
        #print("draw joint 1")
        self.joints[0].draw(self.ax)
        #print("draw joint 2")
        self.joints[1].draw(self.ax)
        #print("draw claw")
        self.claw.draw(self.ax)

    def set_angular_velocity_from_jacobian(self, J, target_velocity, joint=0):
        """
        Set angular velocity based on the Jacobian matrix.
        
        Parameters:
            J (numpy.ndarray): Jacobian matrix.
            target_velocity (numpy.ndarray): Target angular velocity.
            joint (int): Index of the joint for which the velocity is set.
        """
        self.rotating[joint] = True
        angular_velocity = np.linalg.pinv(J).dot(target_velocity)
        self.joints[joint].set_angular_vel(angular_velocity[0])



