#!/usr/bin/env python

from __future__ import print_function

import rospy
from collections import namedtuple
from cyborg_nav.srv import AvailableGoals, DistanceToGoal
from cyborg_navigation.srv import ClosestGoal, ClosestGoalResponse

NAME = 'closest_goal_server'


class Locations():
    Pose = namedtuple('Pose', ['position', 'orientation'])
    Point = namedtuple('Point', ['x', 'y', 'z'])
    Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])

    def __init__(self):
        # Available goals service proxy
        srv_name = '/cyborg/nav/get_available_goals'
        rospy.wait_for_service(srv_name)
        try:
            self._goals_svc = rospy.ServiceProxy(srv_name, AvailableGoals)
        except rospy.ServiceException as e:
            print("Service call to %s failed: %s" % (srv_name, e))

        self.cache = []

    def __iter__(self):
        self.index = 0
        return self

    # Get all named locations
    def next(self):
        if not self.cache:
            for goal in self._goals_svc().goals:
                position = self.__point(goal.position)
                orientation = self.Quaternion(0.0, 0.0, 0.0, 1.0)
                pose = self.Pose(position, orientation)
                self.cache.append((goal.name, pose))

        try:
            result = self.cache[self.index]
        except IndexError:
            raise StopIteration

        self.index += 1
        return result

    # Return a geometry_msgs.Point2D as a named tuple
    def __point(self, p):
        return self.Point(p.x, p.y, 0.0)


class ClosestGoalHandler():
    def __init__(self):
        rospy.init_node(NAME)

        # Distance to goal service proxy
        srv_name = '/cyborg/nav/get_distance_to_goal'
        rospy.wait_for_service(srv_name)
        try:
            self._dist_svc = rospy.ServiceProxy(srv_name, DistanceToGoal)
        except rospy.ServiceException as e:
            print("Service call to %s failed: %s" % (srv_name, e))

        # Closest goal service
        srv_name = '/cyborg/nav/get_closest_goal'
        rospy.Service(srv_name, ClosestGoal, self.goal_cb)

        self.locations = Locations()

        rospy.spin()

    # Get the closest point of interest
    def goal_cb(self, data):
        # Get the distances to each location.
        # If no path is found, the path will be empty
        dist = dict((k, self._dist_svc(v).distance) for k, v in self.locations)

        # Get entry with the shortest distance
        name, dist = min(dist.iteritems(), key=lambda p: p[1])

        return ClosestGoalResponse(name=name, distance=dist)


if __name__ == "__main__":
    ClosestGoalHandler()
