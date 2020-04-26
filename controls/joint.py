from abc import ABC, abstractmethod
from sympy import *

t = Symbol('t')


class Joint(ABC):

    @abstractmethod
    def __init__(self, parent, child, xy):
        self.name = parent.name + '_' + child.name + '_joint'
        self.parent = parent
        self.child = child

        fx, fy, tau = symbols('fx_%s, fy_%s, tau_%s' %
                              (self.name, self.name, self.name))

        self.fx = fx
        self.fy = fy
        self.tau = tau

        self.link1_offset_x, self.link1_offset_y = self.parent.xy_offset(xy)
        self.link2_offset_x, self.link2_offset_y = self.child.xy_offset(xy)
            # (dist1 - self.parent.length / 2) * sympy.cos(self.parent.theta(t))
        # self.link1_offset_y = (dist1 - self.parent.length / 2) * sympy.sin(self.parent.theta(t))
        # self.link2_offset_x = (dist2 - self.child.length / 2) * sympy.cos(self.child.theta(t))
        # self.link2_offset_y = (dist2 - self.child.length / 2) * sympy.sin(self.child.theta(t))

        self.mat_symbol1 = Symbol('parent_mat_%s' % self.name, commutative=False)
        self.mat_symbol2 = Symbol('child_mat_%s' % self.name, commutative=False)

        self._mat1 = None
        self.mat2 = None


class Hinge2D(Joint):

    def __init__(self, parent, child, dist1, dist2):
        super().__init__(parent, child, (0, 0))
        self.dist1 = dist1
        self.dist2 = dist2

        self.link1_offset_x = (dist1 - self.parent.length / 2) * cos(self.parent.theta(t))
        self.link1_offset_y = (dist1 - self.parent.length / 2) * sin(self.parent.theta(t))
        self.link2_offset_x = (dist2 - self.child.length / 2) * cos(self.child.theta(t))
        self.link2_offset_y = (dist2 - self.child.length / 2) * sin(self.child.theta(t))

        self.mat1 = Matrix([[1., 0., cos(self.parent.theta(t) + pi/2) / (self.parent.length / 2)],
                                  [0., 1., sin(self.parent.theta(t) + pi/2) / (self.parent.length / 2)],
                                  [-self.link1_offset_y, self.link1_offset_x, 0.25]])
        self.mat2 = Matrix([[-1., 0., -cos(self.child.theta(t) + pi/2) / (self.child.length / 2)],
                                  [0., -1., -sin(self.child.theta(t) + pi/2) / (self.child.length / 2)],
                                  [self.link2_offset_y, -self.link2_offset_x, -0.25]])
