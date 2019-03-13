#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import copy
import numpy as np


"""
@author: sonia

This function samples a segment in the plane according to
a Halton sequence by sampling over [0, 1] and then using
the parametrization of the segment to map to the
corresponding point in the segment.

The function can be extended to any parametrized curve.
"""


def haltonPointInSegment(n, base, q1, q2):
    # parameters:
    # n = the nth point in the Halton sequence
    # base = the prime number basis of the Halton sequence
    # q1 = first end point of segment, entered as a list
    # q2 = second end point of segment, entered as a list

    # convert to np arrays
    n = int(n)
    base = float(base)

    # nth halton sequence point on [0, 1]
    pB = 0.
    f = 1/base
    itmp = n
    x = 0.
    while(itmp > 0):
        q = math.floor(itmp/base)
        r = itmp % base
        x = x + f * r
        itmp = q
        f = f/base
        pB = x

    # Obtain point in segment
    q = (pB*q1[0] + (1 - pB)*q2[0], pB*q1[1] + (1 - pB)*q2[1])
    return q


"""
Patrick Therrien, May 5,2015
This code takes a point and a polygon and checks whether or not the point is
inside the polygon by created normal vectors to each segment and checking the
dot product with a vector from the initial point of the segment to the given
point q
"""


def dist(p1, p2):
    return np.sqrt(np.square(p1[0] - p2[0]) + np.square(p1[1] - p2[1]))


def isPointInConvexPolygon(q, P):
    """
    :param q: a point
    :param P: a list of points to define a polygon
    :return: 0 if point is outside of polygon, 1 if point is on or inside
    the polygon
    """
    polyPList = copy.deepcopy(P)  # this ensures the changes stay local

    polyPList.append(polyPList[0])
    # Initialize relevant variables
    pV = [0, 0]
    fail = 0
    qV = [0, 0]
    pPHV = [0, 0]
    passVar = 0

    for i in range(len(polyPList) - 1):  # If the point is a vertex, autopass
        if q == polyPList[i]:
            passVar = 1
    if passVar == 0:
        for j in range(len(polyPList) - 1):
            p1 = polyPList[j]
            p2 = polyPList[j + 1]
            # create vector along segment
            pPHV[0] = (p2[0] - p1[0]) / dist(p1, p2)
            pPHV[1] = (p2[1] - p1[1]) / dist(p1, p2)
            # rotate vector 90deg so it is normal to segment
            pV[0] = pPHV[1] * -1
            pV[1] = pPHV[0]
            # create vector from q point to first segment point
            qV[0] = (q[0] - p1[0]) / dist(q, p1)
            qV[1] = (q[1] - p1[1]) / dist(q, p1)
            # take dot product and if it is negative then q is outside
            # respective plane
            dotP = np.dot(pV, qV)
            if dotP < 0:
                fail = 1

    del polyPList  # make sure variable is not reused

    if fail:
        return 0
    else:
        return 1


def inCollision(nearest_node, new_point, obstacleList):
    # try different values on n of collisions to improve your collision checker
    collision_points_on_segment = 10 # change the check point to 10
    prime_number = 2
    for i in range(1, collision_points_on_segment):
        point = haltonPointInSegment(i, prime_number,
                                     (nearest_node.x, nearest_node.y),
                                     (new_point[0], new_point[1]))
        for obstacle in obstacleList:
            val = isPointInConvexPolygon(point, obstacle)
            if val == 1:
                return True  # collision
    return False


