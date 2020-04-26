import sympy
from simulation import model
from simulation.geometry import *
from sympy import symbols, Matrix, pprint, solve
from sympy.physics.mechanics import *

t = sympy.symbols('''t''')
u1, u2 = sympy.symbols('''u1, u2''', cls=sympy.Function)

theta1, theta2, theta3, theta4, theta5, theta6, theta7 \
    = sympy.symbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7')
thetad1, thetad2, thetad3, thetad4, thetad5, thetad6, thetad7 \
    = sympy.symbols('thetad1, thetad2, thetad3, thetad4, thetad5, thetad6, thetad7')

cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, cx5, cy5, cx6, cy6 \
    = sympy.symbols('cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, cx5, cy5, cx6, cy6', cls=sympy.Function)

fx1, fy1, fx2, fy2, fx3, fy3, fx4, fy4, fx5, fy5, fx6, fy6, tau1, tau2, tau3, tau4, tau5, tau6 \
    = sympy.symbols('Fx1, Fy1, Fx2, Fy2, Fx3, Fy3, Fx4, Fy4, Fx5, Fy5, Fx6, Fy6, \
                     tau1, tau2, tau3, tau4, tau5, tau6', cls=sympy.Function)

jx1, jy1, jx2, jy2, jx3, jy3, jx4, jy4, jx5, jy5, jx6, jy6, jx7, jy7, jx8, jy8 \
    = sympy.symbols('jx1, jy1, jx2, jy2, jx3, jy3, jx4, jy4, jx5, jy5, jx6, jy6, jx7, jy7, jx8, jy8',
                    cls=sympy.Function)


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

q1, q2, q3, q4, q5, q6 = dynamicsymbols('q1:7')
q1d, q2d, q3d, q4d, q5d, q6d = dynamicsymbols('q1:7', 1)
q1dd, q2dd, q3dd, q4dd, q5dd, q6dd = dynamicsymbols('q1:7', 2)
v1, v2, v3, v4, v5, v6 = dynamicsymbols('v1:7')
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

F1.set_ang_vel(N, q1d * N.z)
F2.set_ang_vel(N, q2d * N.z)
F3.set_ang_vel(N, q3d * N.z)
F4.set_ang_vel(N, q4d * N.z)
F5.set_ang_vel(N, q5d * N.z)
F6.set_ang_vel(N, q6d * N.z)
F7.set_ang_vel(N, q6d * N.z)

O = Point('O')
J1 = O.locatenew('J1', recip_joint_xz[0] * N.x + recip_joint_xz[1] * N.y)
J2 = J1.locatenew('J2', l1 * F1.x)
J3 = J2.locatenew('J3', l2 * F2.x)
J4 = J1.locatenew('J4', fusion_thigh_recip_offset[0] * N.x + fusion_thigh_recip_offset[1] * N.y)
_J3 = J4.locatenew('_J3', thigh_dist_to_connector * F3.x)
J5 = J4.locatenew('J5', thigh_link_length * F3.x)
J6 = J4.locatenew('J6', thigh_dist_to_4bar * F3.x)
J7 = J5.locatenew('J7', shin_link_length * F4.x)
J8 = J6.locatenew('J8', four_bar_link_length * F5.x)
_J7 = J8.locatenew('_J7', ankle_link_length1 * F6.x)
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
recip_center.v2pt_theory(O, N, F1)
connector_center.v2pt_theory(O, N, F2)
thigh_center.v2pt_theory(O, N, F3)
shin_center.v2pt_theory(O, N, F4)
four_bar_center.v2pt_theory(O, N, F5)
ankle_center1.v2pt_theory(O, N, F6)
ankle_center2.v2pt_theory(O, N, F7)
foot_point.v2pt_theory(O, N, F7)
J3.v2pt_theory(O, N, F3)
_J3.v2pt_theory(O, N, F3)
J7.v2pt_theory(O, N, F6)
_J7.v2pt_theory(O, N, F6)


# f_c = Matrix([(J3.pos_from(O) - _J3.pos_from(O)).magnitude()])
# fv_c = Matrix([(J3.vel(N) - _J3.vel(N)).magnitude()])
f_c = Matrix([(J3.pos_from(O) - _J3.pos_from(O)).magnitude(),
              (J7.pos_from(O) - _J7.pos_from(O)).magnitude()])
fv_c = Matrix([(J3.vel(N) - _J3.vel(N)).magnitude(),
               (J7.vel(N) - _J7.vel(N)).magnitude()])

constraints = Matrix([(J3.pos_from(O) - _J3.pos_from(O)).magnitude(),
                      (J7.pos_from(O) - _J7.pos_from(O)).magnitude()])
# T = sum([body.kinetic_energy(N) for body in (recip, connector, thigh, shin, four_bar, ankle1, ankle2)])
# U = m1 * g * recip.masscenter.pos_from(O).dot(N.y) + \
#     m2 * g * connector.masscenter.pos_from(O).dot(N.y) + \
#     m3 * g * thigh.masscenter.pos_from(O).dot(N.y) + \
#     m4 * g * shin.masscenter.pos_from(O).dot(N.y) + \
#     m5 * g * four_bar.masscenter.pos_from(O).dot(N.y) + \
#     m6 * g * four_bar.masscenter.pos_from(O).dot(N.y)
# L = T - U
L = sum([Lagrangian(N, body) for body in (recip, connector, thigh, shin, four_bar, ankle1, ankle2)])

recip_f_mag = u1 / (recip_link_length/2)
LM = LagrangesMethod(L, [q1, q2, q3, q4, q5, q6],
                     forcelist=[(recip_center, recip_f_mag * F1.y)],
                     bodies=[recip, connector, thigh, shin, four_bar, ankle1, ankle2],
                     frame=N,
                     hol_coneqs=constraints)
lag_eqs = LM.form_lagranges_equations()
operating_point = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6,
                   q1d: thetad1, q2d: thetad2, q3d: thetad3, q4d: thetad4, q5d: thetad5, q6d: thetad6,
                   q1d.diff(t): thetad1.diff(t), q2d.diff(t): thetad2.diff(t), q3d.diff(t): thetad3.diff(t),
                   q4d.diff(t): thetad4.diff(t), q5d.diff(t): thetad5.diff(t), q6d.diff(t): thetad6.diff(t)}
lam_op = LM.solve_multipliers(operating_point)
print(lam_op)
operating_point.update(lam_op)
solved_eqs = []
for i, eq in enumerate(lag_eqs):
    solved_eqs += solve(eq, (q1dd, q2dd, q3dd, q4dd, q5dd, q6dd)[i])
print(solved_eqs)
print(solved_eqs[0])
acc = solved_eqs[0]
print(operating_point)
acc = acc.subs(operating_point)
# acc.simplify()
print(acc)


# f_c = Matrix([])
# fv_c = Matrix([vx - foot_point.vel(N).dot(N.x),
#               vy - foot_point.vel(N).dot(N.y)])

# kd = [q1d - v1, q2d - v2, q3d - v3, q4d - v4]
# FL = [(recip_center, m1 * g * N.y + recip_f_mag * F1.y),
#       (connector_center, m2 * g * N.y),
#       (thigh_center, m3 * g * N.y),
#       (shin_center, m4 * g * N.y)]
# BL = [recip, connector, thigh, shin]
# kd = [q1d - v1, q2d - v2, q3d - v3, q4d - v4, q5d - v5, q6d - v6]
# recip_f_mag = u1 / (recip_link_length/2)
# FL = [(recip_center, m1 * g * N.y + recip_f_mag * F1.y),
#       (connector_center, m2 * g * N.y),
#       (thigh_center, m3 * g * N.y),
#       (shin_center, m4 * g * N.y),
#       (four_bar_center, m5 * g * N.y),
#       (ankle_center1, m6 * g * N.y)]
# BL = [recip, connector, thigh, shin, four_bar, ankle1]

# KM = KanesMethod(N, q_ind=[q1, q2, q4], u_ind=[v1, v2, v4], kd_eqs=kd,
#                  configuration_constraints=f_c, velocity_constraints=fv_c, q_dependent=[q3], u_dependent=[v3])
# KM = KanesMethod(N, q_ind=[q1, q2, q5, q4], u_ind=[v1, v2, v5, v4], kd_eqs=kd,
#                  configuration_constraints=f_c, velocity_constraints=fv_c, q_dependent=[q3, q6], u_dependent=[v3, v6])

# fr, frstar = KM.kanes_equations(BL, FL)
# kdd = KM.kindiffdict()
# mm = KM.mass_matrix_full
# fo = KM.forcing_full
# qudots = mm.inv() * fo
# qudots = qudots.subs(kdd)
# qudots.simplify()
# mechanics_printing()
# mprint(qudots)
# hi = fo.subs(kdd)
# hi.simplify()
# mprint(hi)



