import numpy as np
import math
import matplotlib.pyplot as plt
from robot_parts import BasePlate, BasePart, Arm, Motor


from utils import find_nearest, find_lowest, find_highest, check_points

class RobotJoint:
    '''
    Consists of a motor and arm. The origin of rotation is around the center of the motor.
    pos : the position of the center of the motor
    parent : A BasePart that this joint is connected to this object's motor. Is connected to the edge of the arm
    '''
        
    def __init__(self, parent, name : str = "unamed_joint", pos : np.array = np.array([0,0]), ax=None) -> None:
        self.ax = ax if ax is not None else plt.gca()
        self.name = name
        self.parent = parent
        self.state = np.array([0,0,0], dtype=float) # [pos, vel, accel, [rotation, angular rotation, angular acceleration]]
        #print("This is motor!!!")
        self.motor = Motor(pos=pos, ax=self.ax, parent=self.parent)
        self.arm = Arm(ax=self.ax, parent=self.motor)
        self.children = [self.motor, self.arm] # The last index is the edge
        self.edge = self.arm.get_endpoint()


    def set_angular_vel(self, ang_vel : float = 0) -> None:
        self.state[1] = ang_vel

    def rotate(self, degrees : float = 0.0) -> None: # TODO: Complete Rotation
        # Rotates an object relative to its current relative position
        self.state[0] = degrees

    def update(self, delta_t : float):
        self.state[1] += delta_t * self.state[2]
        self.state[0] += delta_t * self.state[1]
        self.motor.rotate(self.state[0])
        # pass

    def draw(self, ax=None):
        # draw.line(screen, (0,0,0), self.pos, )
        self.ax = ax if ax is not None else plt.gca()
        #print("draw arm")
        self.arm.draw(self.ax)
        #print("draw motor")
        self.motor.draw(self.ax)
        