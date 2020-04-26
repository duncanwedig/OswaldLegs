import sympy
from sympy import *
import networkx as nx
import matplotlib.pyplot as plt
import cProfile
import pickle
from controls.joint import Hinge2D
from controls.link import Rod2D

t = Symbol('t')


class Multibody:

    def __init__(self):
        self.links = []
        self.joints = []
        self.mat = sympy.Matrix()
        self.solved = None
        self.f_symbols = []
        self.u = None
        self.u_symbols = []
        self.u_mats = []

    def add_link(self, link):
        self.links.append(link)

    def connect_links(self, parent, child, dist1=None):
        joint = parent.add_joint(child, dist=dist1)
        self.joints.append(joint)
        return joint

    @property
    def _joint_matrix_list(self):
        system = []
        for link in self.links:
            row = []
            for joint in self.joints:
                if link == joint.parent:
                    row.append(joint.mat_symbol1)
                elif link == joint.child:
                    row.append(-joint.mat_symbol2)
                else:
                    row.append(0)
            system.append(row + [link.dtdt])
        return system

    def _expand_solved_symbols(self):
        substitutions = []
        for joint in self.joints:
            substitutions.append((joint.mat_symbol1.name, str(joint.mat1)))
            substitutions.append((joint.mat_symbol2.name, str(joint.mat2)))
        for link in self.links:
            for i, u in enumerate(self.u):
                substitutions.append((link.dtdt.name + '/' + u.name, u.name + '**(-1)*' + link.dtdt.name))
            substitutions.append((link.dtdt.name, str(link.dtdt_vec)))
            substitutions.append((link.x.name, 'Function("%s")' % link.x.name))
            substitutions.append((link.y.name, 'Function("%s")' % link.y.name))
            substitutions.append((link.theta.name, 'Function("%s")' % link.theta.name))
            substitutions.append((link.length.name, 'Symbol("%s")' % link.length.name))
            substitutions.append((link.m.name, 'Symbol("%s")' % link.m.name))
        substitutions.append(('(t)', '(Symbol("t"))'))
        for i, u in enumerate(self.u):
            # substitutions.append((u.name + '**(-1)', str(self.u_mats[i]) + ').T'))
            substitutions.append((u.name, str(self.u_mats[i])))
            link_name = u.name[2:]
            substitutions.append(('ux_' + link_name, 'symbols("ux_%s")' % link_name))
            substitutions.append(('uy_' + link_name, 'symbols("uy_%s")' % link_name))
            substitutions.append(('utau_' + link_name, 'symbols("utau_%s")' % link_name))
        # substitutions.append(('**(-1)', '.inv()'))
        solved_strs = [str(row.expand()) for row in self.solved]
        print(substitutions)
        for i, _ in enumerate(solved_strs):
            for sub in substitutions:
                solved_strs[i] = solved_strs[i].replace(*sub)
        print(solved_strs)
        self.solved = [eval(string) for string in solved_strs]

    def solve_force(self, *args):
        """
        :param args: the link(s) to apply an input force to, this force can later be substituted using sympy
        :return: the solved set of differential equations expressed as a function of the input forces
        """
        assert len(self.links) <= len(self.joints)+1, 'All links must be jointed together!'
        # if link_applied_to is not None:
        self.u_symbols = []
        fj_symbols = [(joint.fx, joint.fy, joint.tau) for joint in self.joints]
        for forces in fj_symbols:
            self.f_symbols += [*forces]
        self.u = []
        for i, link in enumerate(args):
            self.u_symbols += [sympy.symbols('ux_%s, uy_%s, utau_%s' %
                                             (link.name, link.name, link.name))]
            self.u_mats.append(diag(*self.u_symbols[i]))

            self.u.append(sympy.Symbol('u_%s' % link.name))

        system = self._joint_matrix_list
        # if link_applied_to is not None:
        for i, link_applied_to in enumerate(args):
            for j, link in enumerate(self.links):
                system[j].insert(0, self.u[i] if link == link_applied_to else 0)

        system = sympy.Matrix(system)
        A, b = system[:, :-1], system[:, -1]
        print(A)
        self.solved = (A**-1) * b
        print(self.solved)
        self._expand_solved_symbols()
        return self.solved


if __name__ == "__main__":
    # link0 = Link('link0', 7)
    link1 = Rod2D('link1')
    # link1.add_force(0, -9.8)
    # link1.add_force(-u(t) * sympy.sin(link1.theta(t)), u(t) * sympy.cos(link1.theta(t)))
    link2 = Rod2D('link2')
    # link2.add_force(0, -9.8)
    link3 = Rod2D('link3')
    # link3.add_force(0, -9.8)
    # link4 = Link('link4', 18)
    # link5 = Link('link5', 18)
    # link3 = Link('link3', 4)
    # j0 = link0.add_joint(0, link1)
    # j1 = link1.add_joint(link2, 5)
    # j2 = link2.add_joint(link3)
    testing = Multibody()
    testing.add_link(link1)
    testing.add_link(link2)
    testing.add_link(link3)
    # testing.add_link(link4)
    # testing.add_link(link5)
    j1 = testing.connect_links(link1, link2)
    j2 = testing.connect_links(link2, link3)
    # testing.connect_links(link3, link4)
    # testing.connect_links(link4, link5)
    # testing.connect_links(link2, link5)
    # link2.add_joint(5, link3)
    print('howdy')
    # sympy.pprint(link1.fx_expr())
    solved00 = testing.solve_force(link1)
    pprint(solved00)
    # pprint(solve(Eq(solved00[0, 0], 1), symbols('ux_link1')))

    # sympy.pprint(j1.solve_fx_parent())
    # sympy.pprint(link1.theta(t))
    print('hi')
    # sympy.pprint(j2.solve_fx_parent())
    # sympy.pprint(link3.fx_expr)
