import numpy as np
import utils
import math
import pychrono as chrono

# Units are in m and kg
# Model masses
recip_mass = 0.1
connector_mass = 0.1
thigh_mass = 0.1

# Model geometries
body_center = np.array([0, 0, 0.4], np.double)
body_size = np.array([0.075, 0.1, 0.09], np.double)

# Geometries in CAD
fusion_body_corner = np.array([-0.0219, 0, 0.32625], np.double)

fusion_recip_joint_xz = np.array([0.0381, 0, 0.40005], np.double)
fusion_recip_link_y = np.array([0, -0.005, 0], np.double)
fusion_recip_thickness = 0.015

fusion_recip_connector_xz = np.array([0.059101, 0, 0.36826], np.double)
fusion_connector_link_y = np.array([0, -0.024, 0], np.double)
fusion_connector_thickness = 0.015

fusion_thigh_connector_xz = np.array([0.128997, 0, 0.414524], np.double)
fusion_thigh_joint_xz = np.array([0.0635, 0, 0.47625], np.double)
fusion_thigh_link_y = np.array([0, -0.004, 0], np.double)
fusion_thigh_thickness = 0.015

fusion_thigh_shin_xz = np.array([0.183651, 0, 0.363017], np.double)
fusion_shin_link_y = np.array([0, -0.022, 0], np.double)
fusion_shin_thickness = 0.015

fusion_thigh_4bar_xz = np.array([0.155924, 0, 0.389148], np.double)
fusion_4bar_link_y = np.array([0, -0.022, 0], np.double)
# fusion_4bar_thickness = 0.008
fusion_4bar_thickness = fusion_shin_thickness

fusion_shin_ankle_xz = np.array([0.12899, 0, 0.193827], np.double)
fusion_4bar_ankle_xz = np.array([0.101263, 0, 0.219958], np.double)
fusion_ankle_y = np.array([0, 0.000764, 0], np.double)
fusion_ankle_thickness = 0.019764/2

fusion_ankle_toe_xz = np.array([0.155949, 0, 0.030943], np.double)

fusion_foot_left_offset = np.array([0.0381, 0.0127, 0.0381], np.double)
fusion_foot_right_offset = np.array([0.0381, -0.0127, 0.0381], np.double)
fusion_thigh_recip_offset = fusion_thigh_joint_xz - fusion_recip_joint_xz

# Setting up the origin as seen in fusion, used to find global positions of other things later
fusion_body_corner[1] -= body_size[1]
body_lower_left_back = body_center - body_size
fusion_origin = body_lower_left_back - fusion_body_corner

# JOINT SETUP
# -------------------------------------------------------------------------------------------
# Recip joint setup
recip_joint_xz = fusion_recip_joint_xz + fusion_origin

# Recip-connector joint setup
recip_connector_xz = fusion_recip_connector_xz + fusion_origin

# Connector-thigh joint setup
thigh_connector_xz = fusion_thigh_connector_xz + fusion_origin

# Thigh mount setup
thigh_joint_xz = fusion_thigh_joint_xz + fusion_origin

# Thigh-shin joint setup
thigh_shin_xz = fusion_thigh_shin_xz + fusion_origin

# Thigh-4bar joint setup
thigh_4bar_xz = fusion_thigh_4bar_xz + fusion_origin

# Shin-ankle joint setup
shin_ankle_xz = fusion_shin_ankle_xz + fusion_origin

# Ankle-4bar joint setup
ankle_4bar_xz = fusion_4bar_ankle_xz + fusion_origin

# Ankle-toe joint setup
ankle_toe_xz = fusion_ankle_toe_xz + fusion_origin

# LINK SETUP
# -------------------------------------------------------------------------------------------
# Recip link setup
recip_link_y = np.array([0, fusion_recip_link_y[1] - fusion_recip_thickness/2 - body_size[1], 0])

left_recip_link_pos1 = recip_joint_xz - recip_link_y
left_recip_link_pos2 = recip_connector_xz - recip_link_y
left_recip_link_center = (left_recip_link_pos1 + left_recip_link_pos2)/2

right_recip_link_pos1 = recip_joint_xz + recip_link_y
right_recip_link_pos2 = recip_connector_xz + recip_link_y
right_recip_link_center = (right_recip_link_pos1 + right_recip_link_pos2)/2

recip_link_angle = math.radians(utils.angle_between((recip_joint_xz[0], recip_joint_xz[2]),
                                                    (recip_connector_xz[0], recip_connector_xz[2])))
recip_link_euler = [0, 0, recip_link_angle]
recip_link_quaternion = chrono.Q_from_Euler123(chrono.ChVectorD(*recip_link_euler))
recip_link_length = np.linalg.norm(recip_connector_xz - recip_joint_xz)
recip_link_size = np.array([recip_link_length/2, fusion_recip_thickness/2, fusion_recip_thickness/2])

# Connector link setup
connector_link_y = np.array([0, fusion_connector_link_y[1] - fusion_connector_thickness/2 - body_size[1], 0])

left_connector_link_pos1 = recip_connector_xz - connector_link_y
left_connector_link_pos2 = thigh_connector_xz - connector_link_y
left_connector_link_center = (left_connector_link_pos1 + left_connector_link_pos2)/2

right_connector_link_pos1 = recip_connector_xz + connector_link_y
right_connector_link_pos2 = thigh_connector_xz + connector_link_y
right_connector_link_center = (right_connector_link_pos1 + right_connector_link_pos2)/2

connector_link_angle = math.radians(utils.angle_between((recip_connector_xz[0], recip_connector_xz[2]),
                                                        (thigh_connector_xz[0], thigh_connector_xz[2])))
connector_link_euler = [0, 0, connector_link_angle]
connector_link_quaternion = chrono.Q_from_Euler123(chrono.ChVectorD(*connector_link_euler))
connector_link_length = np.linalg.norm(thigh_connector_xz - recip_connector_xz)
connector_link_size = np.array([connector_link_length/2, fusion_connector_thickness/2, fusion_connector_thickness/2])

# Thigh link setup
thigh_link_y = np.array([0, fusion_thigh_link_y[1] - fusion_thigh_thickness/2 - body_size[1], 0])

left_thigh_link_pos1 = thigh_joint_xz - thigh_link_y
left_thigh_link_pos2 = thigh_shin_xz - thigh_link_y
left_thigh_link_center = (left_thigh_link_pos1 + left_thigh_link_pos2)/2

right_thigh_link_pos1 = thigh_joint_xz + thigh_link_y
right_thigh_link_pos2 = thigh_shin_xz + thigh_link_y
right_thigh_link_center = (right_thigh_link_pos1 + right_thigh_link_pos2)/2

thigh_link_angle = math.radians(utils.angle_between((thigh_joint_xz[0], thigh_joint_xz[2]),
                                                    (thigh_shin_xz[0], thigh_shin_xz[2])))
thigh_link_euler = [0, 0, thigh_link_angle]
thigh_link_quaternion = chrono.Q_from_Euler123(chrono.ChVectorD(*thigh_link_euler))
thigh_link_length = np.linalg.norm(thigh_joint_xz - thigh_shin_xz)
thigh_link_size = np.array([thigh_link_length/2, fusion_thigh_thickness/2, fusion_thigh_thickness/2])

thigh_dist_to_connector = np.linalg.norm(thigh_connector_xz - thigh_joint_xz)
thigh_dist_to_4bar = np.linalg.norm(thigh_4bar_xz - thigh_joint_xz)

# Shin link setup
shin_link_y = np.array([0, fusion_shin_link_y[1] - fusion_shin_thickness/2 - body_size[1], 0])

left_shin_link_pos1 = thigh_shin_xz - shin_link_y
left_shin_link_pos2 = shin_ankle_xz - shin_link_y
left_shin_link_center = (left_shin_link_pos1 + left_shin_link_pos2)/2

right_shin_link_pos1 = thigh_shin_xz + shin_link_y
right_shin_link_pos2 = shin_ankle_xz + shin_link_y
right_shin_link_center = (right_shin_link_pos1 + right_shin_link_pos2)/2

shin_link_angle = math.radians(utils.angle_between((thigh_shin_xz[0], thigh_shin_xz[2]),
                                                   (shin_ankle_xz[0], shin_ankle_xz[2])))
shin_link_euler = [0, 0, shin_link_angle]
shin_link_quaternion = chrono.Q_from_Euler123(chrono.ChVectorD(*shin_link_euler))
shin_link_length = np.linalg.norm(thigh_shin_xz - shin_ankle_xz)
shin_link_size = np.array([shin_link_length/2, fusion_shin_thickness/2, fusion_shin_thickness/2])

# Four bar link setup
four_bar_link_y = np.array([0, fusion_4bar_link_y[1] - fusion_4bar_thickness/2 - body_size[1], 0])

left_4bar_link_pos1 = thigh_4bar_xz - four_bar_link_y
left_4bar_link_pos2 = ankle_4bar_xz - four_bar_link_y
left_4bar_link_center = (left_4bar_link_pos1 + left_4bar_link_pos2)/2

right_4bar_link_pos1 = thigh_4bar_xz + four_bar_link_y
right_4bar_link_pos2 = ankle_4bar_xz + four_bar_link_y
right_4bar_link_center = (right_4bar_link_pos1 + right_4bar_link_pos2)/2

four_bar_link_angle = math.radians(utils.angle_between((thigh_4bar_xz[0], thigh_4bar_xz[2]),
                                                       (ankle_4bar_xz[0], ankle_4bar_xz[2])))
four_bar_link_euler = [0, 0, shin_link_angle]
four_bar_link_quaternion = chrono.Q_from_Euler123(chrono.ChVectorD(*four_bar_link_euler))
four_bar_link_length = np.linalg.norm(thigh_4bar_xz - ankle_4bar_xz)
four_bar_link_size = np.array([four_bar_link_length/2, fusion_4bar_thickness/2, fusion_4bar_thickness/2])

# Ankle link setup
ankle_link_y = np.array([0, fusion_ankle_y[1] - fusion_ankle_thickness/2 - body_size[1], 0])

left_ankle_link_pos1 = shin_ankle_xz - ankle_link_y
left_ankle_link_pos2 = ankle_4bar_xz - ankle_link_y
left_ankle_link_center1 = (left_ankle_link_pos1 + left_ankle_link_pos2)/2

left_ankle_link_pos3 = shin_ankle_xz - ankle_link_y
left_ankle_link_pos4 = ankle_toe_xz - ankle_link_y
left_ankle_link_center2 = (left_ankle_link_pos3 + left_ankle_link_pos4)/2

right_ankle_link_pos1 = shin_ankle_xz + ankle_link_y
right_ankle_link_pos2 = ankle_4bar_xz + shin_link_y
right_ankle_link_center1 = (right_ankle_link_pos1 + right_ankle_link_pos2)/2

right_ankle_link_pos3 = shin_ankle_xz + ankle_link_y
right_ankle_link_pos4 = ankle_toe_xz + shin_link_y
right_ankle_link_center2 = (right_ankle_link_pos3 + right_ankle_link_pos4)/2

ankle_link_angle1 = math.radians(utils.angle_between((shin_ankle_xz[0], shin_ankle_xz[2]),
                                                    (ankle_4bar_xz[0], ankle_4bar_xz[2])))
ankle_link_euler1 = [0, 0, ankle_link_angle1]
ankle_link_quaternion1 = chrono.Q_from_Euler123(chrono.ChVectorD(*ankle_link_euler1))
ankle_link_angle2 = ankle_link_angle1 - math.radians(37.8)
ankle_link_euler2 = [0, 0, ankle_link_angle2]
ankle_link_quaternion2 = chrono.Q_from_Euler123(chrono.ChVectorD(*ankle_link_euler2))
ankle_link_length1 = np.linalg.norm(shin_ankle_xz - ankle_4bar_xz)
ankle_link_size1 = np.array([ankle_link_length1/2, fusion_ankle_thickness/2, fusion_ankle_thickness/2])
ankle_link_length2 = np.linalg.norm(shin_ankle_xz - ankle_toe_xz)
ankle_link_size2 = np.array([ankle_link_length2/2, fusion_ankle_thickness/2, fusion_ankle_thickness/2])


