#!/usr/bin/python
"""
Test cases
"""
import sys
import time
import threading
import unittest
import rospy
import rostest
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint

PKG = 'twist_controller'
NAME = 'test_twist_controller'


class TestTwistController(unittest.TestCase):
    """
    A basic testcase for the twist controller.
    It assumes that the twist controller will send in a constant rate
    the control information.
    When dbw_enabed is set to False, no control information shall be send
    anymore as of user interaction.
    """

    def __init__(self, *args):
        super(TestTwistController, self).__init__(*args)
        self.success = False
        self.failed_once = False
        self.dbw_enabled = True
        self.dbw_enabled_callback = self.dbw_enabled
        self.lock = threading.Lock()

    def callback(self, data):
        with self.lock:
            value = self.dbw_enabled_callback
            if value:
                self.success = False if self.failed_once else True
            elif not self.dbw_enabled: # Last check if we already switched back.
                self.failed_once = True
                self.success = False

        rospy.loginfo("%sxpected callback. dbw_enabled_callback %r, dbw_enabled %r, Failed before %r",
                      "E" if value else "Une", value, self.dbw_enabled, self.failed_once)


    def test_notify(self):
        """
        Test that information is published to any of the three control commands
        when dbw node is enabled.
        """
        rospy.init_node(NAME, anonymous=True)

        currentVelocityPub = rospy.Publisher('/current_velocity', TwistStamped)
        twistCmdPub = rospy.Publisher('/twist_cmd', TwistStamped)
        currentPosePub = rospy.Publisher('/current_pose', PoseStamped)
        waypointsPub = rospy.Publisher('/final_waypoints', Lane)
        enablerPub = rospy.Publisher('/vehicle/dbw_enabled', Bool)

        rospy.Subscriber('/vehicle/steering_cmd', SteeringCmd, self.callback)
        rospy.Subscriber('/vehicle/throttle_cmd', ThrottleCmd, self.callback)
        rospy.Subscriber('/vehicle/brake_cmd', BrakeCmd, self.callback)
        count = 1

        while not rospy.is_shutdown() and count < 10:
            timeout_t = time.time() + 1.0  # 1 second
            self.dbw_enabled = False if self.dbw_enabled else True
            """
            Immediately react if dbw is enabled
            """
            while time.time() < timeout_t:
                currentVelocityPub.publish(self._get_twiststamped(10.0))
                twistCmdPub.publish(self._get_twiststamped(12.0))
                currentPosePub.publish(self._get_posestamped(5.0))
                waypointsPub.publish(self._get_lane())

                enablerPub.publish(Bool(self.dbw_enabled))
                rospy.logwarn("dbw_enabled: %r", self.dbw_enabled)
                time.sleep(0.01)
                """
                Grace period if switching to dbw disabled
                """
                with self.lock:
                    self.dbw_enabled_callback = self.dbw_enabled
            count += 1
        self.assert_(self.success, str(self.success))

    def _get_lane(self):
        lane = Lane()
        lane.header.frame_id = "/base_waypoints"
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = [self._get_waypoint(-1), self._get_waypoint(0),
                          self._get_waypoint(1), self._get_waypoint(2)]
        return lane

    def _get_waypoint(self, value=0.0):
        waypoint = Waypoint()
        waypoint.pose = self._get_posestamped(value)
        waypoint.twist = self._get_twiststamped(0.0)
        return waypoint

    @classmethod
    def _get_posestamped(cls, value=0.0):
        goal = PoseStamped()
        goal.header.frame_id = "/current_pose"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.z = value
        goal.pose.position.x = value
        goal.pose.position.y = value
        goal.pose.orientation.w = 1.0
        return goal

    @classmethod
    def _get_twiststamped(cls, value=0.0):
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "/current_pose"
        twist.twist.linear.x = value
        twist.twist.linear.y = value
        twist.twist.linear.z = value
        twist.twist.angular.x = value
        twist.twist.angular.y = value
        twist.twist.angular.z = value
        return twist

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTwistController, sys.argv)
