#!/usr/bin/env python

from __future__ import print_function

import rospy
from collections import namedtuple
from cyborg_nav.srv import DistanceToGoal, DistanceToGoalResponse
from geometry_msgs.msg import Pose
from rosarnl.srv import MakePlan
from scipy.spatial import distance

NAME = 'distance_to_goal_server'

Point = namedtuple('Point', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])


class DistanceToGoalHandler():
    def __init__(self):
        rospy.init_node(NAME)

        srv_name = '/cyborg/nav/get_distance_to_goal'
        rospy.Service(srv_name, DistanceToGoal, self.__distance_cb)

        srv_name = '/rosarnl_node/make_plan'
        rospy.wait_for_service(srv_name)
        try:
            self._plan_svc = rospy.ServiceProxy(srv_name, MakePlan)
        except rospy.ServiceException as e:
            rospy.logerr("Service call to %s failed: %s" % (srv_name, e))

        rospy.spin()

    # Return a geometry_msgs.Point as a named tuple
    def __point(self, p):
        return Point(p.x, p.y, p.z)

    # Return a geometry_msgs.Quaternion as a named tuple
    def __quaternion(self, q):
        return Quaternion(q.x, q.y, q.z, q.w)

    # Get the distance between two Poses
    def __dist(self, a, b):
        p1 = a.position
        p2 = b.position
        return distance.euclidean(self.__point(p1), self.__point(p2))

    # Get the length of a path
    def __get_length(self, path):
        # Here, we create a pairwise list of Poses, compute the distances
        # between them, and sum all the distances
        return sum(map(lambda p: self.__dist(p[0], p[1]), zip(path, path[1:])))

    def __distance_cb(self, data):
        # Create a Pose message to validate our input data
        position = self.__point(data.goal.position)
        orientation = self.__quaternion(data.goal.orientation)
        goal = Pose(position=position, orientation=orientation)

        # Get a path to the given goal
        path = self._plan_svc(goal).path

        # Calculate distance if a path was found, or return inf
        distance = self.__get_length(path) if path else float('inf')

        return DistanceToGoalResponse(distance=distance)


if __name__ == "__main__":
    DistanceToGoalHandler()
