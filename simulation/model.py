from simulation import geometry
import xml.etree.ElementTree as ET
import numpy as np
import string
import pychrono as chrono


def string_array(array: np.ndarray):
    return ' '.join(array.astype(str))


def swap_yz(array: np.ndarray):
    result = np.array([0, 0, 0], np.double)
    result[0], result[2], result[1] = array
    return result


brick_material = chrono.ChMaterialSurfaceNSC()
brick_material.SetFriction(0.5)
brick_material.SetDampingF(0.2)
brick_material.SetCompliance (0.0000001)
brick_material.SetComplianceT(0.0000001)


def make_box(size, pos, fixed=False, collide=True, euler_angles=(0, 0, 0), color=(1., 0, 1.)):
    body = chrono.ChBody()
    body.SetBodyFixed(fixed)
    body.SetPos(chrono.ChVectorD(*(swap_yz(pos))))
    body.SetRot(chrono.Q_from_Euler123(chrono.ChVectorD(*euler_angles)))

    body.GetCollisionModel().ClearModel()
    # swap_yz(size)
    body.GetCollisionModel().AddBox(size[0], size[2], size[1]) # must set half sizes
    body.GetCollisionModel().BuildModel()
    body.SetCollide(collide)

    mboxasset = chrono.ChBoxShape()
    mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(*(swap_yz(size)))
    mboxasset.SetColor(chrono.ChColor(color[0], color[1], color[2]))
    body.AddAsset(mboxasset)
    # system.Add(body)
    return body


def make_mesh(filename, pos, fixed=False, collide=True, euler_angles=(0, 0, 0), color=(1., 1., 1.)):
    body = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile(filename),  # mesh filename
                                 7000,  # density kg/m^3
                                 True,  # automatically compute mass and inertia
                                 True,  # visualize?>
                                 0.001,
                                 collide  # collide?
                                 )  # use mesh for collision?
    body.SetBodyFixed(fixed)
    body.SetPos(chrono.ChVectorD(*swap_yz(pos)))
    body.SetRot(chrono.Q_from_Euler123(chrono.ChVectorD(*euler_angles)))
    return body


def make_joint(body1, body2, joint_pos):
    joint = chrono.ChLinkRevolute()
    joint_frame = chrono.ChFrameD(chrono.ChVectorD(*swap_yz(joint_pos)))
    joint.Initialize(body1, body2, joint_frame)
    return joint


def make_model(system):
    # write_xml()

    # Make ground body
    ground = make_box([1, 1, 10], [0, 0, -10], fixed=True, color=(1., 0.5, 0))
    system.Add(ground)

    # Make main body
    main_body = make_box(geometry.body_size, geometry.body_center, fixed=False)
    system.Add(main_body)

    right_recip = make_box(geometry.recip_link_size, geometry.right_recip_link_center, fixed=False,
                          euler_angles=geometry.recip_link_euler)
    system.Add(right_recip)

    right_connector = make_box(geometry.connector_link_size, geometry.right_connector_link_center, fixed=False,
                              euler_angles=geometry.connector_link_euler)
    system.Add(right_connector)

    right_thigh = make_box(geometry.thigh_link_size, geometry.right_thigh_link_center, fixed=False,
                          euler_angles=geometry.thigh_link_euler)
    system.Add(right_thigh)

    right_shin = make_box(geometry.shin_link_size, geometry.right_shin_link_center, fixed=False,
                         euler_angles=geometry.shin_link_euler)
    system.Add(right_shin)

    right_4bar = make_box(geometry.four_bar_link_size, geometry.right_4bar_link_center, fixed=False,
                         euler_angles=geometry.four_bar_link_euler)
    system.Add(right_4bar)

    right_ankle1 = make_box(geometry.ankle_link_size1, geometry.right_ankle_link_center1, fixed=False, collide=False,
                           euler_angles=geometry.ankle_link_euler1)
    system.Add(right_ankle1)

    right_ankle2 = make_box(geometry.ankle_link_size2, geometry.right_ankle_link_center2, fixed=False,
                           euler_angles=geometry.ankle_link_euler2)
    system.Add(right_ankle2)

    right_foot = make_mesh('foot.stl', [0, 0, 0], fixed=True, collide=False)
    system.Add(right_foot)

    # Setting up right joints

    right_recip_connector_joint = make_joint(right_recip, right_connector, geometry.right_recip_link_pos2)
    system.Add(right_recip_connector_joint)

    right_thigh_mount = make_joint(main_body, right_thigh, geometry.right_thigh_link_pos1)
    system.Add(right_thigh_mount)

    right_thigh_connector_joint = make_joint(right_connector, right_thigh, geometry.right_connector_link_pos2)
    system.Add(right_thigh_connector_joint)

    right_thigh_shin_joint = make_joint(right_thigh, right_shin, geometry.right_shin_link_pos1)
    system.Add(right_thigh_shin_joint)

    right_thigh_4bar_joint = make_joint(right_thigh, right_4bar, geometry.right_4bar_link_pos1)
    system.Add(right_thigh_4bar_joint)

    right_shin_ankle_joint1 = make_joint(right_shin, right_ankle1, geometry.right_shin_link_pos2)
    system.Add(right_shin_ankle_joint1)

    right_4bar_ankle_joint1 = make_joint(right_4bar, right_ankle1, geometry.right_4bar_link_pos2)
    system.Add(right_4bar_ankle_joint1)

    right_shin_ankle_joint2 = make_joint(right_shin, right_ankle2, geometry.right_shin_link_pos2)
    system.Add(right_shin_ankle_joint2)

    right_4bar_ankle_joint2 = make_joint(right_4bar, right_ankle2, geometry.right_4bar_link_pos2)
    system.Add(right_4bar_ankle_joint2)

    right_recip_joint = make_joint(main_body, right_recip, geometry.right_recip_link_pos1)
    system.Add(right_recip_joint)

    '''
        Now do it again, but for the left side!
    '''

    left_recip = make_box(geometry.recip_link_size, geometry.left_recip_link_center, fixed=False,
                          euler_angles=geometry.recip_link_euler)
    system.Add(left_recip)

    left_connector = make_box(geometry.connector_link_size, geometry.left_connector_link_center, fixed=False,
                              euler_angles=geometry.connector_link_euler)
    system.Add(left_connector)

    left_thigh = make_box(geometry.thigh_link_size, geometry.left_thigh_link_center, fixed=False,
                          euler_angles=geometry.thigh_link_euler)
    system.Add(left_thigh)

    left_shin = make_box(geometry.shin_link_size, geometry.left_shin_link_center, fixed=False,
                         euler_angles=geometry.shin_link_euler)
    system.Add(left_shin)

    left_4bar = make_box(geometry.four_bar_link_size, geometry.left_4bar_link_center, fixed=False,
                         euler_angles=geometry.four_bar_link_euler)
    system.Add(left_4bar)

    left_ankle1 = make_box(geometry.ankle_link_size1, geometry.left_ankle_link_center1, fixed=False, collide=False,
                           euler_angles=geometry.ankle_link_euler1)
    system.Add(left_ankle1)

    left_ankle2 = make_box(geometry.ankle_link_size2, geometry.left_ankle_link_center2, fixed=False,
                           euler_angles=geometry.ankle_link_euler2)
    system.Add(left_ankle2)

    # Setting up left joints

    left_recip_connector_joint = make_joint(left_recip, left_connector, geometry.left_recip_link_pos2)
    system.Add(left_recip_connector_joint)

    left_thigh_mount = make_joint(main_body, left_thigh, geometry.left_thigh_link_pos1)
    system.Add(left_thigh_mount)

    left_thigh_connector_joint = make_joint(left_connector, left_thigh, geometry.left_connector_link_pos2)
    system.Add(left_thigh_connector_joint)

    left_thigh_shin_joint = make_joint(left_thigh, left_shin, geometry.left_shin_link_pos1)
    system.Add(left_thigh_shin_joint)

    left_thigh_4bar_joint = make_joint(left_thigh, left_4bar, geometry.left_4bar_link_pos1)
    system.Add(left_thigh_4bar_joint)

    left_shin_ankle_joint1 = make_joint(left_shin, left_ankle1, geometry.left_shin_link_pos2)
    system.Add(left_shin_ankle_joint1)

    left_4bar_ankle_joint1 = make_joint(left_4bar, left_ankle1, geometry.left_4bar_link_pos2)
    system.Add(left_4bar_ankle_joint1)

    left_shin_ankle_joint2 = make_joint(left_shin, left_ankle2, geometry.left_shin_link_pos2)
    system.Add(left_shin_ankle_joint2)

    left_4bar_ankle_joint2 = make_joint(left_4bar, left_ankle2, geometry.left_4bar_link_pos2)
    system.Add(left_4bar_ankle_joint2)

    left_recip_joint = make_joint(main_body, left_recip, geometry.left_recip_link_pos1)
    system.Add(left_recip_joint)


if __name__ == '__main__':
    system = chrono.ChSystemNSC()
    make_model(system)
