import cv2
import math
import numpy as np
import utils
from matplotlib import pyplot as plt
import random


class JointChain:

    def __init__(self, origin):
        self.links = []
        self.angles = []
        self.joints = [np.array(origin, np.double)]
        self.image = 255 * np.ones((1440, 2560, 3), np.uint8)

    def add_link(self, link_length, default_angle=0):
        self.angles += [math.radians(default_angle)]
        self.links += [link_length]
        self.joints += [utils.raytrace_line(self.joints[-1], math.radians(default_angle), link_length)]
        return self

    def recalc(self):
        for i, link in enumerate(self.links):
            self.joints[i + 1] = utils.raytrace_line(self.joints[i], self.angles[i], link)

    def set_angle(self, i, angle):
        self.angles[i] = math.radians(angle)

    def draw(self, image=np.zeros((720, 1280, 3), np.uint8)):
        for i, _ in enumerate(self.links):
            cv2.line(image, tuple(self.joints[i].astype(np.int)),
                     tuple(self.joints[i + 1].astype(np.int)), color=(255, 0, 0))
        for i in self.joints:
            cv2.circle(image, tuple(i.astype(int)), 4, color=(0, 255, 0))

    def circle(self, x, y):
        cv2.circle(self.image, (x, y), 4, color=(0, 255, 0))

    def show(self):
        cv2.imshow("hi", self.image)
        cv2.waitKey(0)

    def reverse(self):
        self.joints = self.joints[::-1]
        self.links = self.links[::-1]

    def fabrik_once(self, goal, check_dist=False):
        if check_dist and sum(self.links) < utils.dist(goal, self.joints[0]):
            print(utils.dist(goal, self.joints[0]))
            raise ValueError('goal is too far away')
        # new_chain = JointChain((int(goal[0]), int(goal[1])))
        self.reverse()
        self.joints[0] = np.array(goal, np.double)
        for i, point in enumerate(self.joints[1:]):
            self.set_angle(i, utils.angle_between(self.joints[i], point))
            self.recalc()

    def ccd_once(self, goal):
        # for _ in range(iters):
        for i, _ in enumerate(self.angles[::-1]):
            if i == 0:
                self.angles[-i - 1] = math.radians(utils.angle_between(self.joints[-i - 2], goal))
            else:
                self.angles[-i - 1] += math.radians(utils.angle_between(self.joints[-i - 2], goal) - utils.angle_between(
                    self.joints[-i - 2], self.joints[-1]))
            self.recalc()
        err = self.joints[-1] - np.array(goal, np.double)
        return err

    def analytic_ik(self, goal):
        assert len(self.links) == 2, \
            "An inverse kinematics problem can only be solved analytically for a 2-link system"
        self.angles = analytic_2link(self.links[0], self.links[1], self.joints[0], goal)
        self.recalc()
        print(self.angles)


def fabrik(goal, chain, iters):
    if iters == 0:
        return chain
    origin = tuple(chain.joints[0])
    for i in range(iters):
        chain.fabrik_once(goal)
        chain.fabrik_once(origin)
        # chain.draw()
    err = (chain.joints[-1][0] - goal[0], chain.joints[-1][1] - goal[1])
    return err


def analytic_2link(l1, l2, p1, p2, flip=False):
    dy = p2[0] - p1[0]
    dx = p2[1] - p1[1]
    r2 = dx**2 + dy**2
    _theta2 = math.pi - math.acos((-r2 + l1**2 + l2**2) / (2 * l1 * l2))
    if flip:
        _theta2 = -_theta2
    # _x = dx * (l1 + l2*math.cos(_theta2)) + dy * l2 * math.sin(_theta2)
    # _y = dy * (l1 + l2*math.cos(_theta2)) - dx * l2 * math.sin(_theta2)
    d = a = l1 + l2 * math.cos(_theta2)
    b = l2 * math.sin(_theta2)
    s1 = (d * dx - b * dy)
    c1 = (b * dx + a * dy)

    theta1 = math.atan2(s1, c1)
    return theta1, _theta2 + theta1


# def run_legs(p):
#     recip_angle, chain_angle = p
#
#     # Setting up the first four bar linkage and drawing the part that's actually acting as a kinematic chain
#     goal = utils.raytrace_line(center, math.radians(recip_angle), 45.57)
#     # cProfile.run('fabrik(tuple(goal.astype(int)), chain2, 30)')
#     fabrik(tuple(goal.astype(int)), recip_loop, 10)
#     # recip_loop.circle(int(center[0]), int(center[1]))
#     # recip_loop.draw(image=display)
#
#     # drawing the recip joint to close the kinematic loop
#     # cv2.circle(display, tuple(center.astype(int)), 4, color=(0, 255, 0))
#     # cv2.line(display, tuple(center.astype(int)), tuple(goal.astype(int)), color=(255, 0, 0))
#
#     upper_thigh = utils.raytrace_line(thigh_mount, recip_loop.angles[0], 127)
#     # cv2.circle(display, tuple(upper_thigh.astype(int)), 4, color=(0, 255, 0))
#
#     lower_thigh = utils.raytrace_line(thigh_mount, recip_loop.angles[0], 165.1)
#     # cv2.circle(display, tuple(lower_thigh.astype(int)), 4, color=(0, 255, 0))
#     # cv2.line(display, tuple(thigh_mount.astype(int)), tuple(lower_thigh.astype(int)), color=(255, 0, 0))
#
#     # draw shin
#     shin_loop.joints[0] = upper_thigh
#     front_lower_shin = utils.raytrace_line(lower_thigh, chain_angle, 177.8)
#     # shin_loop.angles[0] = recip_loop.angles[0]
#     shin_loop.angles[0] = utils.bound_between(chain_angle, recip_loop.angles[0] + math.radians(40), recip_loop.angles[0] + math.radians(140))
#     shin_loop.angles[1] = recip_loop.angles[0]
#     shin_loop.recalc()
#     # shin_loop.draw(image=display)
#     # cv2.line(display, tuple(front_lower_shin.astype(int)), tuple(lower_thigh.astype(int)), color=(255, 0, 0))
#     toe_point = utils.raytrace_line(front_lower_shin, shin_loop.angles[1] - math.radians(142.7 - 180), 165.1)
#     # cv2.circle(display, tuple(toe_point.astype(int)), 4, color=(0, 255, 0))
#     # cv2.line(display, tuple(front_lower_shin.astype(int)), tuple(toe_point.astype(int)), color=(255, 0, 0))
#
#     # cv2.imshow('display', display)
#     # cv2.waitKey(0)
#     # print(chain2.angles[0])
#     return toe_point - goal
#
#
# TODO: clean up all the stuff here so the the naming and stuff actually makes sense
# if __name__ == "__main__":
#     thigh_mount = np.array([300+63.5, 720-476.25-100], np.double)
#     recip_loop = JointChain(thigh_mount).add_link(90, 0).add_link(85.152, 0)
#     lower_thigh = utils.raytrace_line(thigh_mount, recip_loop.angles[0], 165.1)
#     chain_bar_angle = math.radians(0)
#     shin_loop = JointChain(lower_thigh).add_link(177.80, -180).add_link(38.1, -180)
#     center = np.array([300+38.1, 720-400.05-100], np.double)
#     recip_angle = 0
#     thigh_angles = np.zeros(720, np.double)
#     upper_thigh_points = np.zeros((720, 2), np.double)
#     lower_ankle_points = np.zeros((720, 2), np.double)
#     foot_points = np.zeros((73, 73, 2), np.double)
#     goal = (300, 300)
#     # while True:
#     #     display = np.zeros((720, 1280, 3), np.uint8)
#     #     fsolve(run_legs, np.array((60, 90), np.double))
#     # for i in range(720):
#     #     run_legs(i, 0)
#     #     thigh_angles[i] = recip_loop.angles[0]
#     # plt.plot(thigh_angles)
#     # plt.show()
#     # fig = plt.figure()
#     # ax = fig.add_subplot(111, projection='3d')
#     for j in range(73):
#         # print(j)
#         print(chain_bar_angle)
#         for i in range(73):
#             display = np.zeros((720, 1280, 3), np.uint8)
#             foot_points[i, j] = run_legs((recip_angle, chain_bar_angle))
#             recip_angle += 5
#         # run_legs(recip_angle, chain_bar_angle)
#         chain_bar_angle += math.radians(5)
#         plt.plot(foot_points[:, j, 0], foot_points[:, j, 1])
#     # ax.plot(foot_points[:, :, 0], foot_points[:, :, 1], range(73))
#     plt.show()
if __name__ == '__main__':
    while True:
        chain = JointChain((300, 300))
        display = np.zeros((720, 1280, 3), np.uint8)
        chain.add_link(random.random() * 100. + 50).add_link(175)
        chain.analytic_ik((250., 300.))
        # fabrik((300., 250.), chain, 30)
        print(chain.joints)
        print(chain.links)
        chain.draw(display)
        cv2.imshow('display', display)
        cv2.waitKey(0)

