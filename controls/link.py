import sympy


t = sympy.symbols('t')


class Link:

    def __init__(self, name: str, length: float):
        self.name = name
        self.length = length
        self.x1, self.y1, self.x2, self.y2 \
            = sympy.symbols('x_' + name + '1, y_' + name + '1, x_' + name + '2, y_' + name + '2', cls=sympy.Function)
        self.theta = sympy.symbols('theta_' + name, cls=sympy.Function)
        self.net_torque = sympy.symbols('tau_' + name, cls=sympy.Function)
        self.tau_expr = 0
        self.net_force = sympy.symbols('F_' + name, cls=sympy.Function)
        self.f_expr = 0
        sympy.Eq(self.net_torque(t), self.theta(t).diff(t).diff(t))

    def add_joint(self, link1):
        sympy.Eq(link1.x2(t), self.x1(t))
        sympy.Eq(link1.y2(t), self.y1(t))



