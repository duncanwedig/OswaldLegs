from abc import ABC, abstractmethod
from sympy import symbols, Symbol, Function, Matrix
from controls.joint import Hinge2D

t = Symbol('t')


class Link(ABC):

    @abstractmethod
    def __init__(self, name: str):
        self.name = name
        # self.length = Symbol('l_' + name)
        self.m = Symbol('m_' + name)
        self.I = Symbol('I_' + name)
        self.theta_z = Function('theta_z_' + name)
        self.x, self.y = symbols('x_%s, y_%s' % (name, name), cls=Function)
        self.tau_x_expr = 0
        self.tau_y_expr = 0
        self.tau_z_expr = 0
        self.fx_expr = 0
        self.fy_expr = 0
        self.fz_expr = 0
        self.joints = []
        self.is_parent = []

        self.dtdt = Symbol('dtdt_%s' % self.name, commutative=False)
        self.dtdt_vec = None

    @abstractmethod
    def add_joint(self, other_link, *args, **kwargs):
        pass

    def add_force(self, fx=0, fy=0, fz=0, tau_x=0, tau_y=0, tau_z=0):
        self.fx_expr += fx
        self.fy_expr += fy
        self.fz_expr += fz
        self.tau_x_expr += tau_x
        self.tau_y_expr += tau_y
        self.tau_z_expr += tau_z

    def __repr__(self):
        return self.name

    def xy_offset(self, xy):
        return xy[0] - self.x(t), xy[0] - self.y(t)


class Rod2D(Link):

    def __init__(self, name: str):
        super().__init__(name)
        self.length = Symbol('l_' + name)
        self.I = self.m / 12 * self.length ** 2

        self.tau_expr = self.tau_z_expr
        self.theta = self.theta_z

        self.dtdt_vec = Matrix([self.m * self.x(t).diff(t, t) - self.fx_expr,
                               self.m * self.y(t).diff(t, t) - self.fy_expr,
                               self.I * self.theta(t).diff(t, t) - self.tau_expr])

    def add_joint(self, other_link, **kwargs):
        try:
            joint = kwargs['cls'](self, other_link, kwargs)
        except KeyError:
            dist = kwargs['dist']
            if dist is None:
                dist = self.length
            joint = Hinge2D(self, other_link, dist, 0)
        self.joints.append(joint)
        self.is_parent.append(True)

        other_link.joints.append(joint)
        other_link.is_parent.append(False)

        return joint

    def add_force(self, fx=0, fy=0, tau_z=0):
        return super().add_force(fx, fy, tau_z=tau_z)
