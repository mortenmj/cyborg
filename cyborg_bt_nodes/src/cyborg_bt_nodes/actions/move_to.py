#!/usr/bin/env python
import actionlib
import b3
import rospy

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cyborg_types import Point, Quaternion, Pose


class MoveTo(b3.Action):
    def __init__(self, location, radius):
        self.description = "Move to the given location"
        super(MoveTo, self).__init__()

        self._req = None
        self._cli = actionlib.SimpleActionClient(
                '/move_base',
                MoveBaseAction)

        try:
            self._cli.wait_for_server(rospy.Duration(5))
        except:
            rospy.loginfo('Timed out waiting for move_base')

        self._status = None
        self._goal_status = None
        self._target_location = location
        self._target_radius = radius

    def __str__(self):
        return 'MoveTo <%s>' % self._target_location

    def _start(self, tick):
        if not self._req:
            coords = tick.blackboard.get('locations')[self._target_location]
            position = Point(*coords)
            orientation = Quaternion(0, 0, 0, 1)
            pose = Pose(position, orientation)

            self._req = MoveBaseGoal()
            self._req.target_pose.header.frame_id = 'map'
            self._req.target_pose.header.stamp = rospy.Time.now()
            self._req.target_pose.pose = pose

        self._cli.send_goal(
                self._req,
                done_cb=self._done_cb,
                active_cb=self._active_cb,
                feedback_cb=self._feedback_cb)

    def _stop(self):
        self._cli.cancel_goal()

    # Called before the tick callback, if not closed
    def open(self, tick):
        # Set start time, so we can time out if necessary
        start_time = rospy.Time.now()
        tick.blackboard.set('start_time', start_time, tick.tree.id, self.id)

        # Cancel all existing goals
        self._cli.cancel_all_goals()

        # Send the goal
        self._start(tick)

        self._action_finished = False

        rospy.loginfo('Running action: %s' % self)

    # Contains the execution code for this node
    def tick(self, tick):
        # start_time = tick.blackboard.get('start_time', tick.tree.id, self.id)
        # elapsed_time = rospy.Time.now() - start_time

        if not self._action_finished:
            self._status = b3.RUNNING
        else:
            if self._goal_status == GoalStatus.SUCCEEDED:
                self._status = b3.SUCCESS
            elif self._goal_status == GoalStatus.PREEMPTED:
                self._status = b3.RUNNING
                self._action_finished = False
                self._start(tick)
            else:
                self._status = b3.FAILURE

        return self._status

    def _active_cb(self):
        pass

    def _feedback_cb(self, feedback):
        cur_pos = Pose.from_pose_stamped(feedback.base_position)
        target_pos = self._req.target_pose.pose

        # Check if we've arrived within the accepted radius of our target
        if cur_pos.distance_to(target_pos) < self._target_radius:
            self._stop()
            self._goal_status = GoalStatus.SUCCEEDED
            self._action_finished = True

    def _done_cb(self, result_state, result):
        # We might have already set _goal_status to SUCCEEDED
        # when preempting in _feedback_cb.
        if self._goal_status != GoalStatus.SUCCEEDED:
            self._goal_status = result_state

        self._action_finished = True


if __name__ == "__main__":
    MoveTo(None, None)
