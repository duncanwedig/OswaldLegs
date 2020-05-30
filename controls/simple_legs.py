from inversekinematics.iksolvers import JointChain, fabrik, analytic_2link
import cv2
import math
import numpy as np
import utils
import cProfile
import random
from enum import Enum, auto

THIGH_MOUNT = np.array((300 + 63.5, 720 - 476.25 - 100), np.double)
RECIP_MOUNT = np.array([300 + 38.1, 720 - 400.05 - 100], np.double)
RECIP_LENGTH = 45.57
CONNECTOR_LENGTH = 85.152
THIGH_LENGTH = 165.1
SHIN_LENGTH = 177.8
BACK_ANKLE_LENGTH = 38.1
FRONT_ANKLE_LENGTH = 165.1
THIGH_DIST_TO_CONNECTOR = 90
THIGH_DIST_TO_FOUR_BAR = 127


# TODO: make there be constants for everything
class Leg:

    def __init__(self):
        self.THIGH_SHIN_CHAIN = JointChain(THIGH_MOUNT).add_link(THIGH_LENGTH, 0).add_link(SHIN_LENGTH, 0)
        self.RECIP_CHAIN = JointChain(RECIP_MOUNT).add_link(RECIP_LENGTH, 0).add_link(CONNECTOR_LENGTH, 0)

        self.ankle_angle = 0
        self.back_ankle = utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[1],
                                              -self.THIGH_SHIN_CHAIN.angles[0], -BACK_ANKLE_LENGTH)
        self.back_thigh_connection = utils.raytrace_line(THIGH_MOUNT,
                                                         -self.THIGH_SHIN_CHAIN.angles[0], THIGH_DIST_TO_FOUR_BAR)
        self.upper_thigh_connection = utils.raytrace_line(THIGH_MOUNT,
                                                          self.THIGH_SHIN_CHAIN.angles[0], THIGH_DIST_TO_CONNECTOR)

    def legs_ccd(self, goal, iters=30):
        for i in range(iters):
            self.THIGH_SHIN_CHAIN.angles[1] += math.radians(
                utils.angle_between(self.THIGH_SHIN_CHAIN.joints[1], goal) - utils.angle_between(
                    self.THIGH_SHIN_CHAIN.joints[1], utils.raytrace_line(
                        self.THIGH_SHIN_CHAIN.joints[2], 
                        self._thigh_angle - math.radians(142.7-180), FRONT_ANKLE_LENGTH)))
            self.THIGH_SHIN_CHAIN.angles[1] = utils.bound_between(self.THIGH_SHIN_CHAIN.angles[1],
                                                                  self._thigh_angle + math.radians(40),
                                                                  self._thigh_angle + math.radians(140))
            self.THIGH_SHIN_CHAIN.recalc()
            self._thigh_angle += math.radians(
                utils.angle_between(self.THIGH_SHIN_CHAIN.joints[0], goal) - utils.angle_between(
                    self.THIGH_SHIN_CHAIN.joints[0], utils.raytrace_line(
                        self.THIGH_SHIN_CHAIN.joints[2],
                        self._thigh_angle - math.radians(142.7-180), FRONT_ANKLE_LENGTH)))
            self._thigh_angle = utils.bound_between(self._thigh_angle, 0.15, 1.45)
            self.THIGH_SHIN_CHAIN.recalc()
        self.recalc_from_foot()

    def recalc_from_foot(self):
        self.back_ankle = utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[2], self._thigh_angle, -BACK_ANKLE_LENGTH)
        self.back_thigh_connection = utils.raytrace_line(self._thigh_mount,
                                                         self._thigh_angle, THIGH_DIST_TO_FOUR_BAR)
        self.upper_thigh_connection = utils.raytrace_line(self._thigh_mount, self._thigh_angle, THIGH_DIST_TO_CONNECTOR)
        # fabrik(self.upper_thigh_connection, self.RECIP_CHAIN, 30)
        self.RECIP_CHAIN.analytic_ik(self.upper_thigh_connection)

    def recalc(self, recip_angle=None, chain_angle=None):
        if recip_angle is None:
            recip_angle = self._recip_angle
        if chain_angle is None:
            chain_angle = self._shin_angle - self._connector_angle - math.pi
        self.RECIP_CHAIN.angles[0] = recip_angle
        self.RECIP_CHAIN.joints[1] = utils.raytrace_line(self._recip_mount, recip_angle, RECIP_LENGTH)
        self.RECIP_CHAIN.angles[1], self._thigh_angle = analytic_2link(
            CONNECTOR_LENGTH, THIGH_DIST_TO_CONNECTOR, self.RECIP_CHAIN.joints[1], self._thigh_mount)
        self._thigh_angle = -math.pi + self._thigh_angle
        if self._thigh_angle > 1.57 or self._thigh_angle < 0:
            self.RECIP_CHAIN.angles[1], self._thigh_angle = analytic_2link(
                CONNECTOR_LENGTH, THIGH_DIST_TO_CONNECTOR, self.RECIP_CHAIN.joints[1], self._thigh_mount, flip=True)
            self._thigh_angle = -math.pi + self._thigh_angle
        self._shin_angle = chain_angle + self._connector_angle + math.pi
        self.RECIP_CHAIN.recalc()
        self.THIGH_SHIN_CHAIN.recalc()
        self.back_ankle = utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[2], self._thigh_angle, -BACK_ANKLE_LENGTH)
        self.back_thigh_connection = utils.raytrace_line(self._thigh_mount,
                                                         self._thigh_angle, THIGH_DIST_TO_FOUR_BAR)
        self.upper_thigh_connection = self.RECIP_CHAIN.joints[2]

    def relocate_around_foot(self, angle_around_point=math.pi/2):
        reverse_ankle_angle = math.pi - self._ankle_angle
        angle_delta = angle_around_point - reverse_ankle_angle
        self.THIGH_SHIN_CHAIN.joints[2] = utils.raytrace_line(self._foot_point, -reverse_ankle_angle - angle_delta,
                                                              FRONT_ANKLE_LENGTH)
        reverse_shin_angle = math.pi - self._shin_angle + angle_delta
        self.THIGH_SHIN_CHAIN.joints[1] = utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[2], -reverse_shin_angle,
                                                              SHIN_LENGTH)
        reverse_thigh_angle = math.pi - self._thigh_angle + angle_delta
        self._thigh_mount = utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[1], -reverse_thigh_angle,
                                                THIGH_LENGTH)
        self._thigh_angle -= angle_delta

        self.RECIP_CHAIN.joints[2] = utils.raytrace_line(self._thigh_mount, self._thigh_angle, THIGH_DIST_TO_CONNECTOR)
        reverse_connector_angle = math.pi - self._connector_angle + angle_delta
        self.RECIP_CHAIN.joints[1] = utils.raytrace_line(self.RECIP_CHAIN.joints[2], -reverse_connector_angle,
                                                         CONNECTOR_LENGTH)
        reverse_recip_angle = math.pi - self._recip_angle + angle_delta
        self._recip_mount = utils.raytrace_line(self.RECIP_CHAIN.joints[1], -reverse_recip_angle, RECIP_LENGTH)
        self._connector_angle -= angle_delta
        self._recip_angle -= angle_delta
        self._shin_angle -= math.pi + angle_delta
        self.THIGH_SHIN_CHAIN.recalc()

    def revert_to_base_mount(self, recip_angle=None, chain_angle=None):
        if recip_angle is None:
            recip_angle = self._recip_angle
        if chain_angle is None:
            chain_angle = self._shin_angle - self._connector_angle - math.pi
        self._thigh_mount = THIGH_MOUNT
        self._recip_mount = RECIP_MOUNT
        self.recalc(recip_angle, chain_angle)

    def move_foot_to(self, point):
        delta = point - self._foot_point
        self._thigh_mount += delta
        self._recip_mount += delta
        self.recalc()

    def move_thigh_mount_to(self, point):
        delta = point - self._thigh_mount
        self._thigh_mount += delta
        self._recip_mount += delta
        self.recalc()

    @property
    def _thigh_mount(self):
        return self.THIGH_SHIN_CHAIN.joints[0]

    @_thigh_mount.setter
    def _thigh_mount(self, value):
        self.THIGH_SHIN_CHAIN.joints[0] = value

    @_thigh_mount.getter
    def thigh_mount(self):
        return self._thigh_mount

    @property
    def _recip_mount(self):
        return self.RECIP_CHAIN.joints[0]

    @_recip_mount.setter
    def _recip_mount(self, value):
        self.RECIP_CHAIN.joints[0] = value

    @property
    def _recip_angle(self):
        return self.RECIP_CHAIN.angles[0]

    @_recip_angle.setter
    def _recip_angle(self, value):
        self.RECIP_CHAIN.angles[0] = value

    @_recip_angle.getter
    def recip_angle(self):
        return self._recip_angle

    @property
    def _connector_angle(self):
        return self.RECIP_CHAIN.angles[1]

    @_connector_angle.setter
    def _connector_angle(self, value):
        self.RECIP_CHAIN.angles[1] = value

    @property
    def _thigh_angle(self):
        return self.THIGH_SHIN_CHAIN.angles[0]

    @_thigh_angle.setter
    def _thigh_angle(self, value):
        self.THIGH_SHIN_CHAIN.angles[0] = value

    @property
    def _shin_angle(self):
        return self.THIGH_SHIN_CHAIN.angles[1]

    @_shin_angle.setter
    def _shin_angle(self, value):
        self.THIGH_SHIN_CHAIN.angles[1] = value

    @property
    def _ankle_angle(self):
        return self._thigh_angle - math.radians(142.7-180)

    @property
    def _foot_point(self):
        return utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[2], self._ankle_angle, FRONT_ANKLE_LENGTH)

    @_foot_point.getter
    def foot_point(self):
        return self._foot_point

    def draw(self, image):
        self.THIGH_SHIN_CHAIN.draw(image)
        self.RECIP_CHAIN.draw(image)

        cv2.line(image,
                 tuple(self.THIGH_SHIN_CHAIN.joints[-1].astype(int)),
                 tuple(self._foot_point.astype(int)),
                 color=(255, 0, 0))
        cv2.line(image, tuple(self.THIGH_SHIN_CHAIN.joints[-1].astype(int)), tuple(self.back_ankle.astype(int)),
                 color=(255, 0, 0))
        cv2.line(image, tuple(self.back_ankle.astype(int)), tuple(self.back_thigh_connection.astype(int)),
                 color=(255, 0, 0))

        cv2.circle(image, tuple(self.back_ankle.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self._thigh_mount.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self.back_thigh_connection.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self.upper_thigh_connection.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self._foot_point.astype(int)), 4, color=(0, 0, 255))


class Legs:

    class LegState(Enum):
        LEFT_STANCE = 1
        LEFT_SWING = 2

    LEFT_STANCE = LegState.LEFT_STANCE
    LEFT_SWING = LegState.LEFT_SWING

    def __init__(self, ground_height=600):
        self.left = Leg()
        self.right = Leg()
        self.left.recalc(0, -0.75)
        self.right.recalc(0, -0.75)
        self.state = self.LEFT_STANCE
        self.current_pivot = self.left.foot_point
        self.ground_height = ground_height

    def step_both_recip(self, left_delta, right_delta):
        if self.state == self.LEFT_STANCE:
            foot_point = self.left.foot_point
            self.left.recalc(self.left.recip_angle + left_delta)
            self.left.move_foot_to(foot_point)
            self.right.recalc(self.right.recip_angle + right_delta)
            self.right.move_thigh_mount_to(self.left.thigh_mount)

            if self.right.foot_point[1] > self.ground_height:
                self.state = self.LEFT_SWING
        else:
            foot_point = self.right.foot_point
            self.right.recalc(self.right.recip_angle + right_delta)
            self.right.move_foot_to(foot_point)
            self.left.recalc(self.left.recip_angle + left_delta)
            self.left.move_thigh_mount_to(self.right.thigh_mount)

            if self.left.foot_point[1] > self.ground_height:
                self.state = self.LEFT_STANCE

    def draw(self, image):
        self.right.draw(image)
        self.left.draw(image)

        cv2.line(image, (0, self.ground_height), (10000, self.ground_height), (255, 0, 0))


def reach_for_random_target():
    display = np.zeros((720, 1280, 3), np.uint8)
    legs = Leg()
    goal = (400, 600)
    while True:
        display = np.zeros((720, 1280, 3), np.uint8)
        goal = (random.randint(300, 600), random.randint(400, 600))
        cProfile.runctx('legs.legs_ccd(goal, 30)', globals(), locals())
        legs.draw(display)
        cv2.circle(display, goal, 4, color=(0, 0, 255))
        cv2.imshow("", display)
        cv2.waitKey(0)


def pivot_leg_around_point():
    recip_angle = 0
    left_leg = Leg()
    left_leg.recalc(recip_angle, -0.75)
    goal = (400, 400)
    goal = np.array(goal, np.double)
    left_leg.move_foot_to(goal)
    while True:
        display = np.zeros((720, 1280, 3), np.uint8)
        foot_point = left_leg.foot_point
        left_leg.revert_to_base_mount()
        left_leg.recalc(recip_angle, -0.75)
        left_leg.move_foot_to(foot_point)
        left_leg.draw(display)
        cv2.imshow("", display)
        cv2.waitKey(1)
        recip_angle += 0.005
        if recip_angle >= math.pi * 2:
            recip_angle -= 2 * math.pi


def leg_pair_walking():
    legs = Legs()
    display = np.zeros((720, 1280, 3), np.uint8)
    legs.step_both_recip(0, math.pi)
    while True:
        legs.step_both_recip(0.005, 0.005)
        display = np.zeros((720, 1280, 3), np.uint8)
        legs.draw(display)
        cv2.imshow("", display)
        cv2.waitKey(1)


if __name__ == "__main__":
    leg_pair_walking()
    # cProfile.runctx('pivot_leg_around_point()', globals(), locals())
    # pivot_leg_around_point()
    # reach_for_random_target()
