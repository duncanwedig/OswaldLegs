import sympy
from simulation import model
from simulation.geometry import *
from sympy import symbols, Matrix, pprint, solve, lambdify, solveset
from sympy.physics.mechanics import *
from inversekinematics.iksolvers import analytic_2link
import pickle
import time

t = sympy.symbols('''t''')
# u1, u2 = sympy.symbols('''u1, u2''')
#
theta1, theta2, theta3, theta4, theta5, theta6, theta7 \
    = sympy.symbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7')
# thetad1, thetad2, thetad3, thetad4, thetad5, thetad6, thetad7 \
#     = sympy.symbols('thetad1, thetad2, thetad3, thetad4, thetad5, thetad6, thetad7')
#
# cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, cx5, cy5, cx6, cy6 \
#     = sympy.symbols('cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, cx5, cy5, cx6, cy6', cls=sympy.Function)
#
# fx1, fy1, fx2, fy2, fx3, fy3, fx4, fy4, fx5, fy5, fx6, fy6, tau1, tau2, tau3, tau4, tau5, tau6 \
#     = sympy.symbols('Fx1, Fy1, Fx2, Fy2, Fx3, Fy3, Fx4, Fy4, Fx5, Fy5, Fx6, Fy6, \
#                      tau1, tau2, tau3, tau4, tau5, tau6', cls=sympy.Function)
#
# jx1, jy1, jx2, jy2, jx3, jy3, jx4, jy4, jx5, jy5, jx6, jy6, jx7, jy7, jx8, jy8 \
#     = sympy.symbols('jx1, jy1, jx2, jy2, jx3, jy3, jx4, jy4, jx5, jy5, jx6, jy6, jx7, jy7, jx8, jy8',
#                     cls=sympy.Function)


def link(x1, y1, dist1, angle1, x2, y2, dist2, angle2):
    sympy.Eq(x1(t) + dist1 * sympy.cos(angle1(t)),
             x2(t) + dist2 * sympy.cos(angle2(t)))
    sympy.Eq(y1(t) + dist1 * sympy.sin(angle1(t)),
             y2(t) + dist2 * sympy.sin(angle2(t)))


#
# u1 = theta1(t).diff(t).diff(t)
#
# # theta4(t) is actually added to theta5(t) and theta6(t)
# u2 = theta4(t).diff(t).diff(t)

# theta1(t): recip
# theta2(t): connector
# theta3(t): thigh
# theta4(t): shin
# theta5(t): ankle
# theta6(t): offset for chain bar
# shin angle: theta2 + theta6

# joint 1: recip mount
# joint 2: recip connector
# joint 3: connector thigh
# joint 4: thigh mount
# joint 5: thigh shin
# joint 6: thigh 4bar
# joint 7: shin ankle
# joint 8: 4bar ankle

q1, q2, q3, q4, q5, q6 = qs = dynamicsymbols('q1:7')
q1d, q2d, q3d, q4d, q5d, q6d = dynamicsymbols('q1:7', 1)
q1dd, q2dd, q3dd, q4dd, q5dd, q6dd = dynamicsymbols('q1:7', 2)
v1, v2, v3, v4, v5, v6 = vs = dynamicsymbols('v1:7')
v1d, v2d, v3d, v4d, v5d, v6d = dynamicsymbols('v1:7', 1)
x, y = dynamicsymbols('x y')
xd, yd = dynamicsymbols('x y', 1)
vx, vy = dynamicsymbols('vx vy')
u1, u2 = dynamicsymbols('u1 u2')
u1d, u2d = dynamicsymbols('u1 u2', 1)
l1, l2, l3, l4, l5, l6, l7 = symbols('l1:8')
m1, m2, m3, m4, m5, m6, m7 = symbols('m1:8')
# I1, I2, I3, I4, I5, I6 = symbols('I1 I2 I3 I4 I5 I6', cls=Dyadic)
m, g = symbols('m g')

N = ReferenceFrame('N')
F1 = N.orientnew('F1', 'Axis', [q1, N.z])
F2 = N.orientnew('F2', 'Axis', [q2, N.z])
F3 = N.orientnew('F3', 'Axis', [q3, N.z])
F4 = N.orientnew('F4', 'Axis', [q4, N.z])
F5 = N.orientnew('F5', 'Axis', [q5, N.z])
F6 = N.orientnew('F6', 'Axis', [q6, N.z])
F7 = N.orientnew('F7', 'Axis', [q6 - 0, N.z])

# F1.set_ang_vel(N, q1d * N.z)
# F2.set_ang_vel(N, q2d * N.z)
# F3.set_ang_vel(N, q3d * N.z)
# F4.set_ang_vel(N, q4d * N.z)
# F5.set_ang_vel(N, q5d * N.z)
# F6.set_ang_vel(N, q6d * N.z)
# F7.set_ang_vel(N, q6d * N.z)

F1.set_ang_vel(N, v1 * N.z)
F2.set_ang_vel(N, v2 * N.z)
F3.set_ang_vel(N, v3 * N.z)
F4.set_ang_vel(N, v4 * N.z)
F5.set_ang_vel(N, v5 * N.z)
F6.set_ang_vel(N, v6 * N.z)
F7.set_ang_vel(N, v6 * N.z)

O = Point('O')
J1 = O.locatenew('J1', recip_joint_xz[0] * N.x + recip_joint_xz[1] * N.y)
J2 = J1.locatenew('J2', l1 * F1.x)
J3 = J2.locatenew('J3', l2 * F2.x)
J4 = J1.locatenew('J4', fusion_thigh_recip_offset[0] * N.x + fusion_thigh_recip_offset[1] * N.y)
_J3 = J4.locatenew('_J3', thigh_dist_to_connector * F3.x)
J5 = J4.locatenew('J5', thigh_link_length * F3.x)
J6 = J4.locatenew('J6', thigh_dist_to_4bar * F3.x)
J8 = J6.locatenew('J8', four_bar_link_length * F5.x)
J7 = J8.locatenew('J7', ankle_link_length1 * F6.x)
_J7 = J5.locatenew('_J7', shin_link_length * F4.x)
foot_point = J7.locatenew('foot_point', ankle_link_length2 * F7.x)
# x = foot_point.pos_from(O).express(N).dot(N.x)
# y = foot_point.pos_from(O).express(N).dot(N.y)

recip_center = J1.locatenew('recip_center', l1/2 * F1.x)
connector_center = J2.locatenew('connector_center', l2/2 * F2.x)
thigh_center = J4.locatenew('thigh_center', l3/2 * F3.x)
shin_center = J5.locatenew('shin_center', l4/2 * F4.x)
four_bar_center = J6.locatenew('four_bar_center', l5/2 * F5.x)
ankle_center1 = J8.locatenew('ankle_center1', l6/2 * F6.x)
ankle_center2 = J8.locatenew('ankle_center2', l7/2 * F7.x)

recip = RigidBody('recip', recip_center, F1, m1, (outer(F1.x, F1.x), recip_center))
connector = RigidBody('connector', connector_center, F2, m2, (outer(F2.x, F2.x), connector_center))
thigh = RigidBody('thigh', thigh_center, F3, m3, (outer(F3.x, F3.x), thigh_center))
shin = RigidBody('shin', shin_center, F4, m4, (outer(F4.x, F4.x), shin_center))
four_bar = RigidBody('four_bar', four_bar_center, F5, m5, (outer(F5.x, F5.x), four_bar_center))
ankle1 = RigidBody('ankle1', ankle_center1, F6, m6, (outer(F6.x, F6.x), ankle_center1))
ankle2 = RigidBody('ankle2', ankle_center2, F7, m7, (outer(F7.x, F7.x), ankle_center2))

O.set_vel(N, Vector(0))
J1.set_vel(N, Vector(0))
recip_center.v2pt_theory(J1, N, F1)
J2.v2pt_theory(J1, N, F1)
connector_center.v2pt_theory(J2, N, F2)
J4.set_vel(N, Vector(0))
J3.v2pt_theory(J2, N, F2)
_J3.v2pt_theory(J4, N, F3)
thigh_center.v2pt_theory(J4, N, F3)
J5.v2pt_theory(J4, N, F3)
J6.v2pt_theory(J4, N, F3)
shin_center.v2pt_theory(J5, N, F4)
four_bar_center.v2pt_theory(J6, N, F5)
J8.v2pt_theory(J6, N, F5)
J7.v2pt_theory(J8, N, F6)
_J7.v2pt_theory(J5, N, F4)
ankle_center1.v2pt_theory(J7, N, F6)


# recip_center_vel = recip_center.pos_from(O).dt(N)
# recip_center_vel.simplify()
# recip_center.set_vel(N, recip_center_vel)
# connector_center_vel = connector_center.pos_from(O).dt(N)
# connector_center_vel.simplify()
# connector_center.set_vel(N, connector_center_vel)
# thigh_center_vel = thigh_center.pos_from(O).dt(N)
# thigh_center_vel.simplify()
# thigh_center.set_vel(N, thigh_center_vel)
# shin_center_vel = shin_center.pos_from(O).dt(N)
# shin_center_vel.simplify()
# shin_center.set_vel(N, shin_center_vel)
# four_bar_center_vel = four_bar_center.pos_from(O).dt(N)
# four_bar_center_vel.simplify()
# four_bar_center.set_vel(N, four_bar_center_vel)
# ankle_center1_vel = ankle_center1.pos_from(O).dt(N)
# ankle_center1_vel.simplify()
# ankle_center1.set_vel(N, ankle_center1_vel)
# ankle_center2_vel = ankle_center2.pos_from(O).dt(N)
# ankle_center2_vel.simplify()
# ankle_center2.set_vel(N, ankle_center2_vel)
# foot_point_vel = foot_point.pos_from(O).dt(N)
# foot_point_vel.simplify()
# foot_point.set_vel(N, foot_point_vel)
#
# J3_vel = J3.pos_from(O).dt(N)
# J3_vel.simplify()
# J3.set_vel(N, J3_vel)
# _J3_vel = _J3.pos_from(O).dt(N)
# _J3_vel.simplify()
# _J3.set_vel(N, _J3_vel)
# J7_vel = J7.pos_from(O).dt(N)
# J7_vel.simplify()
# J7.set_vel(N, J7_vel)
# _J7_vel = _J7.pos_from(O).dt(N)
# _J7_vel.simplify()
# _J7.set_vel(N, _J7_vel)

f_c = Matrix([(J3.pos_from(O) - _J3.pos_from(O)).magnitude(),
              (J7.pos_from(O) - _J7.pos_from(O)).magnitude()])
fv_c = Matrix([(J3.vel(N) - _J3.vel(N)).magnitude(),
               (J7.vel(N) - _J7.vel(N)).magnitude()])

# constraints = Matrix([(J3.pos_from(O) - _J3.pos_from(O)).magnitude(),
#                       (J7.pos_from(O) - _J7.pos_from(O)).magnitude()])
# # T = sum([body.kinetic_energy(N) for body in (recip, connector, thigh, shin, four_bar, ankle1, ankle2)])
# # U = m1 * g * recip.masscenter.pos_from(O).dot(N.y) + \
# #     m2 * g * connector.masscenter.pos_from(O).dot(N.y) + \
# #     m3 * g * thigh.masscenter.pos_from(O).dot(N.y) + \
# #     m4 * g * shin.masscenter.pos_from(O).dot(N.y) + \
# #     m5 * g * four_bar.masscenter.pos_from(O).dot(N.y) + \
# #     m6 * g * four_bar.masscenter.pos_from(O).dot(N.y)
# # L = T - U
# L = sum([Lagrangian(N, body) for body in (recip, connector, thigh, shin, four_bar, ankle1, ankle2)])
#
# recip_f_mag = u1 / (recip_link_length/2)
# LM = LagrangesMethod(L, [q1, q2, q3, q4, q5, q6],
#                      forcelist=[(recip_center, recip_f_mag * F1.y)],
#                      bodies=[recip, connector, thigh, shin, four_bar, ankle1, ankle2],
#                      frame=N,
#                      hol_coneqs=constraints)
# lag_eqs = LM.form_lagranges_equations()
# print(lag_eqs)
# operating_point = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6,
#                    q1d: thetad1, q2d: thetad2, q3d: thetad3, q4d: thetad4, q5d: thetad5, q6d: thetad6,
#                    q1d.diff(t): thetad1.diff(t), q2d.diff(t): thetad2.diff(t), q3d.diff(t): thetad3.diff(t),
#                    q4d.diff(t): thetad4.diff(t), q5d.diff(t): thetad5.diff(t), q6d.diff(t): thetad6.diff(t)}
# lam_op = LM.solve_multipliers(operating_point)
# # print(lam_op)
# operating_point.update(lam_op)
# solved_eqs = []
# for symbol, eq in zip(dynamicsymbols('q1:7', 2), lag_eqs):
#     last_time = time.time()
#     solved_eqs += [msubs(solve(eq, symbol)[0], operating_point)]
#     print(symbol)
#     print(time.time() - last_time)
# # print(solved_eqs)
# # print(solved_eqs[0])
# acc = solved_eqs[0]
# # print(operating_point)
# # acc.simplify()
# print(acc)

# pickle.dump(solved_eqs, open('model.pkl', 'wb'), protocol=pickle.HIGHEST_PROTOCOL)
'''
kd = [q1d - v1, q2d - v2, q3d - v3, q4d - v4, q5d - v5, q6d - v6]
recip_f_mag = u1 / (recip_link_length/2)
FL = [(recip_center, m1 * g * N.y + recip_f_mag * F1.y),
      (connector_center, m2 * g * N.y),
      (thigh_center, m3 * g * N.y),
      (shin_center, m4 * g * N.y),
      (four_bar_center, m5 * g * N.y),
      (ankle_center1, m6 * g * N.y)]
BL = [recip, connector, thigh, shin, four_bar, ankle1]

KM = KanesMethod(N, q_ind=[q1, q2, q4, q5], u_ind=[v1, v2, v4, v5], kd_eqs=kd,
                 configuration_constraints=f_c, velocity_constraints=fv_c,
                 q_dependent=[q3, q6], u_dependent=[v3, v6])

fr, frstar = KM.kanes_equations(BL, FL)
kdd = KM.kindiffdict()
print(kdd)
# mm = KM.mass_matrix_full
# fo = KM.forcing_full
# qudots = mm.inv() * fo
# qudots = qudots.subs(kdd)
# qudots.simplify()
# mechanics_printing()
# mprint(qudots)
print("hi")
# hi = fo.subs(kdd)
# hi.simplify()
hi = KM.rhs()
hi = hi.subs(kdd)
print('hi1.5')
# qudots = mm.inv() * fo
# qudots = qudots.subs(kdd)
print('hi2')
print(*vs)
print(*qs)
# mprint(qudots)
# hi.simplify()
print('hi3')
mprint(hi[6])
# pickle.Pickler(open('model.pkl', 'wb'), protocol=pickle.HIGHEST_PROTOCOL).dump(hi)
print('hi4')
numerical_func = lambdify(qs + vs, hi, 'numpy')
print('hi5')
pickle.Pickler(open('model.pkl', 'wb'), protocol=pickle.HIGHEST_PROTOCOL).dump(numerical_func)
print('hi6')

# q1, q2 = dynamicsymbols('q1 q2')
# q1d, q2d = dynamicsymbols('q1 q2', 1)
# v1, v2 = dynamicsymbols('v1 v2')
# v1d, v2d = dynamicsymbols('v1 v2', 1)
# l, m, g = symbols('l m g')
#
# N = ReferenceFrame('N')
# A = N.orientnew('A', 'Axis', [q1, N.z])
# B = N.orientnew('B', 'Axis', [q2, N.z])
#
# A.set_ang_vel(N, v1 * N.z)
# B.set_ang_vel(N, v2 * N.z)
#
# O = Point('O')
# P = O.locatenew('P', l * A.x)
# R = P.locatenew('R', l * B.x)
#
# O.set_vel(N, 0)
# P.v2pt_theory(O, N, A)
# R.v2pt_theory(P, N, B)
#
# ParP = Particle('ParP', P, m)
# ParR = Particle('ParR', R, m)
#
# kd = [q1d - v1, q2d - v2]
# FL = [(P, m * g * N.x), (R, m * g * N.x)]
# BL = [ParP, ParR]
#
# KM = KanesMethod(N, q_ind=[q1, q2], u_ind=[v1, v2], kd_eqs=kd)
#
# fr, frstar = KM.kanes_equations(BL, FL)
# kdd = KM.kindiffdict()
# mm = KM.mass_matrix_full
# fo = KM.forcing_full
# # qudots = mm.inv() * fo
# # qudots = qudots.subs(kdd)
# # qudots.simplify()
# mechanics_printing()
# # mprint(qudots)
# print("hi")
# # hi = fo.subs(kdd)
# # hi.simplify()
# hi = KM.rhs()
# hi.simplify()
# mprint(hi)
'''


def draw_legs():
    pass


# def legs_inertia(recip_angle, chain_bar_offset):
#     recip_c = utils.raytrace_line((recip_joint_xz[0], recip_joint_xz[2]), recip_angle, recip_link_length/2)
#     recip_connector = utils.raytrace_line((recip_joint_xz[0], recip_joint_xz[2]), recip_angle, recip_link_length)
#     thigh_angle, connector_angle = analytic_2link(thigh_dist_to_connector, connector_link_length,
#                                                   thigh_joint_xz[0], thigh_joint_xz[2], *recip_connector)


if __name__ == "__main__":
    pass


