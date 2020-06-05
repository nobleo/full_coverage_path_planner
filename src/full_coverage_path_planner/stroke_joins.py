import numpy as np
from math import sqrt

def half_circle(start, end, previous=None):
    """Generate a half circle between start and end, returns interpolated vector i and half-circle j coordinates"""

    r = (end - start) / 2
    for i in np.linspace(start, end)[1:-1]:
        # circle formula: j = sqrt(r^2 - (i-a)^2) + b
        # a is offset in i and in our case identical to the radius
        # b is offset in j and not relevant in our case
        j = sqrt(r ** 2 - (i - start - r) ** 2)
        yield (i, j)


def mwm(start, end, previous, v_depth=1.0, v_bottom_off_center=0.0):
    """Generate a path that connects two strokes via a V, so that the two strokes combined look like an M,
    with a small v in the middle
    :param start: one top of the v
    :param end: the other top of the v
    :param previous: To indicate direction of the V (to not make a ^), previous is the point that comes before the start of the V (so the bottom-left point of an M or top-left in a W)
    :return: yields all the points of the v (so the bottom of the v)

    >>> mwm(np.array((1, 2)), np.array((2, 3)), np.array((3, 0)))
    array([ 2.,  2.])
    """

    # This needs to know in which direction to put the V. This may or may not be alternating even:
    # The pattern could be M M M but just as well M M M
    #                       W W (v point up)       V V with the inner-v point
    start, end, previous = np.array(start), np.array(end), np.array(previous)

    r = (end - start) / 2.
    middle = start + r

    delta_ver = previous - start
    delta_hor = start - end

    distance_hor = np.linalg.norm(delta_hor)
    distance_ver = np.linalg.norm(delta_ver)

    normalized_delta_hor = (delta_hor / distance_hor) if np.any(delta_hor) else 0.0
    normalized_delta_ver = (delta_ver / distance_ver) if np.any(delta_ver) else 0.0

    # try:
    v = middle + (normalized_delta_ver * distance_hor * v_depth)
    v2 = v + (normalized_delta_hor * v_bottom_off_center * r)

    yield tuple(v2)