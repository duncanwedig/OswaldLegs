import math
import numpy as np


def angle_between(p1, p2):
    return math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))


def raytrace_line(p1, angle, distance):
    return p1 + distance * np.array([math.cos(angle), math.sin(angle)], np.double)


def dist(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)


def bound_between(value, lower_lim, upper_lim):
    return min(upper_lim, max(lower_lim, value))


def deriv(f: callable, x, epsilon=0.0001):
    return (f(x+epsilon) - f(x-epsilon))/(2*epsilon)
