from inversekinematics.iksolvers import JointChain, fabrik
import cv2
import math
import numpy as np
import utils
import cProfile
import random


# TODO: make there be constants for everything
class Legs:

    THIGH_MOUNT = np.array((300 + 63.5, 720 - 476.25 - 100), np.double)
    RECIP_MOUNT = np.array([300+38.1, 720-400.05-100], np.double)

    THIGH_SHIN_CHAIN = JointChain(THIGH_MOUNT).add_link(165.1, 0).add_link(177.8, 0) # .add_link(165.1, 0)
    RECIP_CHAIN = JointChain(RECIP_MOUNT).add_link(45.57, 0).add_link(85.152, 0)

    ankle_angle = 0
    back_ankle = utils.raytrace_line(THIGH_SHIN_CHAIN.joints[1],
                                     -THIGH_SHIN_CHAIN.angles[0], -38.1)
    back_thigh_connection = utils.raytrace_line(THIGH_MOUNT,
                                                -THIGH_SHIN_CHAIN.angles[0], 127)
    upper_thigh_connection = utils.raytrace_line(THIGH_MOUNT,
                                                 THIGH_SHIN_CHAIN.angles[0], 90)

    def legs_ccd(self, goal, iters):
        for i in range(iters):
            self.THIGH_SHIN_CHAIN.angles[1] += math.radians(
                utils.angle_between(self.THIGH_SHIN_CHAIN.joints[1], goal) - utils.angle_between(
                    self.THIGH_SHIN_CHAIN.joints[1], utils.raytrace_line(
                        self.THIGH_SHIN_CHAIN.joints[2], 
                        self.THIGH_SHIN_CHAIN.angles[0] - math.radians(142.7-180), 165.1)))
            self.THIGH_SHIN_CHAIN.angles[1] = utils.bound_between(self.THIGH_SHIN_CHAIN.angles[1],
                                                                  self.THIGH_SHIN_CHAIN.angles[0] + math.radians(40),
                                                                  self.THIGH_SHIN_CHAIN.angles[0] + math.radians(140))
            self.THIGH_SHIN_CHAIN.recalc()
            self.THIGH_SHIN_CHAIN.angles[0] += math.radians(
                utils.angle_between(self.THIGH_SHIN_CHAIN.joints[0], goal) - utils.angle_between(
                    self.THIGH_SHIN_CHAIN.joints[0], utils.raytrace_line(
                        self.THIGH_SHIN_CHAIN.joints[2],
                        self.THIGH_SHIN_CHAIN.angles[0] - math.radians(142.7-180), 165.1)))
            self.THIGH_SHIN_CHAIN.angles[0] = utils.bound_between(self.THIGH_SHIN_CHAIN.angles[0], 0.15, 1.45)
            self.THIGH_SHIN_CHAIN.recalc()

    def recalc_from_foot(self):
        self.ankle_angle = self.THIGH_SHIN_CHAIN.angles[0] - math.radians(142.7-180)
        self.back_ankle = utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[2],
                                         self.THIGH_SHIN_CHAIN.angles[0], -38.1)
        self.back_thigh_connection = utils.raytrace_line(self.THIGH_MOUNT,
                                                         self.THIGH_SHIN_CHAIN.angles[0], 127)
        self.upper_thigh_connection = utils.raytrace_line(self.THIGH_MOUNT,
                                                         self.THIGH_SHIN_CHAIN.angles[0], 90)
        fabrik(self.upper_thigh_connection, self.RECIP_CHAIN, 30)

    def draw(self, image):
        self.recalc_from_foot()
        self.THIGH_SHIN_CHAIN.draw(image)
        self.RECIP_CHAIN.draw(image)

        cv2.line(image,
                 tuple(self.THIGH_SHIN_CHAIN.joints[-1].astype(int)),
                 tuple(utils.raytrace_line(self.THIGH_SHIN_CHAIN.joints[-1], self.ankle_angle, 165.1).astype(int)),
                 color=(255, 0, 0))
        cv2.line(image, tuple(self.THIGH_SHIN_CHAIN.joints[-1].astype(int)), tuple(self.back_ankle.astype(int)),
                 color=(255, 0, 0))
        cv2.line(image, tuple(self.back_ankle.astype(int)), tuple(self.back_thigh_connection.astype(int)), color=(255, 0, 0))

        cv2.circle(image, tuple(self.back_ankle.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self.THIGH_MOUNT.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self.back_thigh_connection.astype(int)), 4, color=(0, 255, 0))
        cv2.circle(image, tuple(self.upper_thigh_connection.astype(int)), 4, color=(0, 255, 0))


def main():
    display = np.zeros((720, 1280, 3), np.uint8)
    legs = Legs()
    goal = (400, 600)
    while True:
        display = np.zeros((720, 1280, 3), np.uint8)
        goal = (random.randint(300, 600), random.randint(400, 600))
        cProfile.runctx('legs.legs_ccd(goal, 30)', globals(), locals())
        legs.draw(display)
        cv2.circle(display, goal, 4, color=(0, 0, 255))
        cv2.imshow("", display)
        cv2.waitKey(0)


if __name__ == "__main__":
    main()
