import sympy
from simulation import geometry, model
import pychrono as chrono

t, u1, u2 \
    = sympy.symbols('''
    t, u1, u2
    ''')
theta1, theta2, theta3, theta4, theta5, theta6, theta7 \
    = sympy.symbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7', cls=sympy.Function)

cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, cx5, cy5, cx6, cy6 \
    = sympy.symbols('cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4, cx5, cy5, cx6, cy6', cls=sympy.Function)


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
# theta5(t): 4bar
# theta6(t): ankle
# theta7(t): offset for chain bar

# recip mount joint
sympy.Eq(cx1(t) - geometry.recip_link_length/2 * sympy.cos(theta1(t)), geometry.recip_joint_xz[0])
sympy.Eq(cy1(t) - geometry.recip_link_length/2 * sympy.sin(theta1(t)), geometry.recip_joint_xz[2])

# recip connector joint
link(cx1, cy1, geometry.recip_link_length/2, theta1, cx2, cy2, -geometry.connector_link_length/2, theta2)

# thigh mount joint
sympy.Eq(cx3(t) - geometry.thigh_link_length/2 * sympy.cos(theta3(t)), geometry.thigh_joint_xz[0])
sympy.Eq(cy3(t) - geometry.thigh_link_length/2 * sympy.sin(theta3(t)), geometry.thigh_joint_xz[2])

# thigh connector joint
link(cx2, cy2, geometry.connector_link_length/2, theta2,
     cx3, cy3, geometry.thigh_dist_to_connector - geometry.thigh_link_length/2, theta3)

# thigh shin joint
link(cx3, cy3, geometry.thigh_link_length/2, theta3, cx4, cy4, -geometry.shin_link_length/2, theta4(t) + theta7(t))



# rcx = geometry.recip_joint_xz[0] + geometry.recip_link_length * sympy.cos(theta1(t))
# rcy = geometry.recip_joint_xz[1] + geometry.recip_link_length * sympy.sin(theta1(t))
#
# cx2 = rcx + geometry.connector_link_length/2 * sympy.cos(theta2(t))
# cy2 = rcy + geometry.connector_link_length/2 * sympy.sin(theta2(t))
# ctx = rcx + geometry.connector_link_length * sympy.cos(theta2(t))
# cty = rcy + geometry.connector_link_length * sympy.sin(theta2(t))
# sympy.Eq(ctx, geometry.thigh_joint_xz[0] + geometry.thigh_dist_to_connector * sympy.cos(theta3(t)))
# sympy.Eq(cty, geometry.thigh_joint_xz[1] + geometry.thigh_dist_to_connector * sympy.sin(theta3(t)))
#
# cx3 = geometry.thigh_joint_xz[0] + geometry.thigh_link_length/2 * sympy.cos(theta3(t))
# cy3 = geometry.thigh_joint_xz[1] + geometry.thigh_link_length/2 * sympy.sin(theta3(t))
# tsx = geometry.thigh_joint_xz[0] + geometry.thigh_link_length * sympy.cos(theta3(t))
# tsy = geometry.thigh_joint_xz[1] + geometry.thigh_link_length * sympy.sin(theta3(t))
# t4x = geometry.thigh_joint_xz[0] + geometry.thigh_dist_to_4bar * sympy.cos(theta3(t))
# t4y = geometry.thigh_joint_xz[1] + geometry.thigh_dist_to_4bar * sympy.sin(theta3(t))
#
# sympy.Eq(theta2(t), theta5(t) + theta7(t))
# cx4 = tsx + geometry.shin_link_length/2 * sympy.cos(theta4(t) + theta7(t))
# cy4 = tsy + geometry.shin_link_length/2 * sympy.sin(theta4(t) + theta7(t))
# sax = tsx + geometry.shin_link_length * sympy.cos(theta4(t) + theta7(t))
# say = tsy + geometry.shin_link_length * sympy.sin(theta4(t) + theta7(t))
#
# cx5 = t4x + geometry.four_bar_link_length/2 * sympy.cos(theta5(t) + theta7(t))
# cy5 = t4y + geometry.four_bar_link_length/2 * sympy.sin(theta5(t) + theta7(t))
# fax = t4x + geometry.four_bar_link_length * sympy.cos(theta5(t) + theta7(t))
# fay = t4y + geometry.four_bar_link_length * sympy.sin(theta5(t) + theta7(t))
#
# sympy.Eq(sax, fax + geometry.ankle_link_length1 * sympy.cos(theta6(t)))
# sympy.Eq(say, fay + geometry.ankle_link_length1 * sympy.sin(theta6(t)))
# cx6 = fax + geometry.ankle_link_length1/2 * sympy.cos(theta6(t))
# cy6 = fay + geometry.ankle_link_length1/2 * sympy.sin(theta6(t))
#
# v1, v2, v3, v4, v5, v6, v7 = (n(t).diff(t) for n in (theta1, theta2, theta3, theta4, theta5, theta6, theta7))
# a1, a2, a3, a4, a5, a6, a7 = (n.diff(t) for n in (v1, v2, v3, v4, v5, v6, v7))
#
# I1 = 1
# tau_net1 = I1 * a1
# sympy.pprint(tau_net1)
# print(type(theta1))

