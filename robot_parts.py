import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.patches import Polygon
from matplotlib.transforms import Affine2D
from utils import check_points, find_highest, find_lowest

class BasePart:
    def __init__(self, shape : np.ndarray, color : np.ndarray, pos : np.array = np.array([0,0]), origin=np.array([0, 0]), rotation: float=0, parent=None, ax=None):
        self.ax = ax if ax is not None else plt.gca()
        self.color = color
        self.state = np.array([[pos[0],pos[1]], [0,0], [0,0]], dtype=float) # [pos, vel, accel]
        #print("state is ", self.state)
        self.shape = shape
        self.rotation = rotation
        self.origin = origin
        self.abs_origin : np.ndarray
        self.parent = parent
        self.boundary = [self.ax.get_xlim()[1], self.ax.get_ylim()[1]]

        self.num_points = self.shape.shape[0]

        self.points = np.ndarray(shape=(self.num_points,2 ),dtype=float)
        self.update_points()
        check_points(self.points, self.state, self.boundary)

    def rotate_ip(self, val : float = 0.0, degrees : bool = True) -> bool:
        # Returns True if rotations 
        self.rotation += math.radians(val) if degrees else val
        self.rotation %= 2 * math.pi
    
    def rotate(self, val : float = 0.0, degrees : bool = True) -> bool:
        # Returns True if rotations 
        self.rotation = math.radians(val) if degrees else val
        self.rotation %= 2 * math.pi

    def move_ip(self, x: float = 0, y: float = 0) -> None:
        self.state[0][0] += x; self.state[0][1] += y

    def move(self, x: float = 0, y: float = 0) -> None:
        self.state[0][0] = x; self.state[0][1] = y

    def update_points(self) -> None:
        #print("before update ", self.abs_origin)
        shape = np.array([[point[0], point[1], 1] for point in self.shape])
        #reflection_x = np.array([[1, 0], [0, -1]])
        #print("shape is ", shape)
        origin = np.hstack((self.origin, 1))
        #print("origin is", origin)
        transf_mat = self.get_transf_mat()
        #print("trans mat is", transf_mat)
        for i in range(self.num_points):
            self.points[i] = np.matmul(shape[i], transf_mat)[0:2]
            #print("before update points", self.points[i])
            #self.points[i] = np.matmul(reflection_x, self.points[i])
            #print("number of points", self.num_points)
            print("after update points", self.points[i])
        self.abs_origin = np.matmul(origin, transf_mat)[0:2]

    def draw(self, ax) -> None:
        self.update_points()
        polygon = Polygon(self.points, closed=True, color=self.color)
        ax.add_patch(polygon)
    

    def get_corners(self) -> np.array:
        return self.points

    def show_corners(self, ax) -> None:
        for i in range(self.num_points):
            ax.plot([self.points[i][0], self.abs_origin[0]], [self.points[i][1], self.abs_origin[1]], color='red')

    def add_child(self, part, origin) -> np.array:
        self.children.append([part, origin])
        part.parent(self)

    def get_transf_mat(self)->np.ndarray:
        parent_transf_mat = np.eye(3) if self.parent == None else self.parent.get_transf_mat()
        trans_transf_mat = np.array([[1, 0, 0], [0,1,0], [self.state[0][0], self.state[0][1], 1]])
        #print("state is ", self.state)
        parent_transf_mat = np.matmul(trans_transf_mat, parent_transf_mat)
        #print("par tran mat is", parent_transf_mat)
        cur_transf_mat = np.array([[math.cos(self.rotation), -math.sin(self.rotation), 0],
                         [math.sin(self.rotation), math.cos(self.rotation), 0,],
                         [self.origin[0] * math.cos(self.rotation) + self.origin[1] * math.sin(self.rotation),
                          -self.origin[0] * math.sin(self.rotation) + self.origin[1] * math.cos(self.rotation), 1
                          ]])
        # self.abs_origin = np.matmul()
        # print(f"Parent:\n{parent_transf_mat}\nCur_trans:\n{cur_transf_mat}\nrotation:{self.rotation}")
        #print("par tran mat is", parent_transf_mat)
        return np.matmul(cur_transf_mat, parent_transf_mat)


class Motor(BasePart):
    def __init__(self, pos : np.ndarray = np.array([0,0]), parent=None, ax=None):
        self.ax = ax if ax is not None else plt.gca()
        self.parent=parent
        self.radius = 10
        self.center = None
        self.shape = np.array([[self.radius, 0], [0, self.radius], [-self.radius, 0], [0, -self.radius]])
        self.color = (0.4, 0.4, 0.1, 1.0)
        print("this is motor")
        super().__init__(pos=pos, shape=self.shape, color=self.color, parent=self.parent, ax=self.ax)

    def update_points(self) -> None:
        shape = np.array([[point[0], point[1], 1] for point in self.shape])
        center = np.hstack((self.origin, 1))
        transf_mat = self.get_transf_mat()
        #print("state is ", self.state)
        self.center = np.matmul(center, transf_mat)[0:2]
        for i in range(self.num_points):
            self.points[i] = np.matmul(shape[i], transf_mat)[0:2]

    def draw(self, ax) -> None:
        self.update_points()
        #print("center is ", self.center)
        circle = patches.Circle(self.center, self.radius, color=self.color[:3], alpha=self.color[3])
        ax.add_patch(circle)


class Arm(BasePart):
    def __init__(self, parent=None, ax=None):
        self.ax = ax if ax is not None else plt.gca()
        self.parent=parent
        self.shape = np.array([[10, 25], [-10, 25], [-10, -25], [10, -25]])
        self.color = (0.8, 0.4, 0.4, 0.4)
        pos = [0, -25]
        origin = [0, 0]
        print("this is arm")
        super().__init__(pos=pos, shape=self.shape, color=self.color, parent=self.parent, origin=origin, ax=self.ax)

    def get_endpoint(self) -> np.ndarray:
        return (self.shape[3] + self.shape[2]) / 2

class BasePlate(BasePart):
    def __init__(self, pos : np.ndarray([0,0]), ax=None):
        self.ax = ax if ax is not None else plt.gca()
        self.shape = np.array([[50, 25], [-50, 25], [-25, 0], [25, 0]])
        origin = np.array([0,0])
        # self.connector = 
        self.color = np.array([0.4, 0.4, 0.4, 0.4])
        super().__init__(origin=origin, pos=pos, shape=self.shape, color=self.color, ax=self.ax, parent=None)

class Claw(BasePart):
    def __init__(self, pos: np.ndarray([0,0]), name="", parent=None, ax=None):
        self.ax = ax if ax is not None else plt.gca()
        self.parent=parent
        self.name = name
        self.shape = np.array([[15, 25], [-15, 25], [-25, -10], [-7, -25], [-15, -7],
                              [0, 15], [15, -7], [7, -25], [25, -10]]) # Max Size is 50,50
        self.color = (0.8, 0.4, 0.4, 0.4)
        origin = np.array([0,-25])
        super().__init__(origin=pos, pos=origin, shape=self.shape, color=self.color,ax=self.ax, parent=self.parent)