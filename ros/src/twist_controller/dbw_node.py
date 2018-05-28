#!/usr/bin/env python

"""
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

"""

import math
import rospy
import threading
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
from twist_controller import Controller
from helper import compute_cte

# Dont publish if last published values don't differ above corresponding EPSILON
STEERING_EPSILON = 0.1
THROTTLE_EPSILON = 0.05
BRAKE_EPSILON = 0.05

class DBWNode(object):
    """
    DBWNode is in charge of publishing all control values for the car.

    Using information about current state of the car ( velocity and position )
    plus the desired trajectory ( velocity and path to follow ) it publishes
    the control values for the car: throttle, brake, steer.
    """
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        max_throttle_pct = rospy.get_param('~max_throttle_percentage', 1.)
        rospy.logwarn("dbw_node: max throttle pct: %f", max_throttle_pct)
        max_braking_pct = rospy.get_param('~max_braking_percentage', -1.)
        rospy.logwarn("dbw_node: max braking pct: %f", max_braking_pct)

        self.lock = threading.Lock()
        self.dbw_enabled = False
        self.last_throttle = 2*THROTTLE_EPSILON
        self.last_brake = 2*BRAKE_EPSILON
        self.last_steer = 2*STEERING_EPSILON
        self.activated = False
        self.current_velocity = None
        self.proposed_velocities = None
        self.current_pose = None
        self.waypoints = None

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)
        self.controller = Controller(vehicle_mass,
                                     fuel_capacity,
                                     brake_deadband,
                                     decel_limit,
                                     accel_limit,
                                     wheel_radius,
                                     wheel_base,
                                     steer_ratio,
                                     max_lat_accel,
                                     max_steer_angle,
                                     max_throttle_pct,
                                     max_braking_pct)

        rospy.Subscriber('/current_velocity',
                         TwistStamped,
                         self.current_velocity_cb,
                         queue_size=1)

        rospy.Subscriber('/twist_cmd',
                         TwistStamped,
                         self.twist_cmd_cb,
                         queue_size=1)

        rospy.Subscriber('/vehicle/dbw_enabled',
                         Bool,
                         self.dbw_enabled_cb,
                         queue_size=1)

        rospy.Subscriber('/current_pose',
                         PoseStamped,
                         self.current_pose_cb,
                         queue_size=1)

        rospy.Subscriber('/final_waypoints',
                         Lane,
                         self.waypoints_cb,
                         queue_size=1)

        self.loop()

    def loop(self):
        """Loop that computes throttle, brake and steer to publish."""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if all([self.proposed_velocities, self.current_velocity, self.current_pose, self.waypoints]):
                with self.lock:
                    is_activated = self.activated
                    # Calculates CTE of given pose using waypoints
                    cte = compute_cte(self.waypoints, self.current_pose)
                throttle, brake, steer = self.controller.control(is_activated,
                                                                 cte,
                                                                 self.proposed_velocities.twist.linear.x,
                                                                 self.proposed_velocities.twist.angular.z,
                                                                 self.current_velocity.twist.linear.x)
                if is_activated:
                    rospy.logdebug("%f, %f, %f", throttle, brake, steer)
                    self.publish(throttle, brake, steer)
            rate.sleep()


    def publish(self, throttle, brake, steer):
        """Publish throttle, brake and steer."""
        if abs(throttle - self.last_throttle) < THROTTLE_EPSILON:
            tcmd = ThrottleCmd()
            tcmd.enable = True
            tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
            tcmd.pedal_cmd = throttle
            self.throttle_pub.publish(tcmd)
        else:
            rospy.logdebug("not publish throttle: %f", abs(throttle - self.last_throttle))

        if abs(steer - self.last_steer) < STEERING_EPSILON:
            scmd = SteeringCmd()
            scmd.enable = True
            scmd.steering_wheel_angle_cmd = steer
            self.steer_pub.publish(scmd)
        else:
            rospy.logdebug("not publish steer: %f", abs(steer - self.last_steer))

        if abs(brake - self.last_brake) < BRAKE_EPSILON:
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)
        else:
            rospy.logdebug("not publish brake: %f", abs(brake - self.last_brake))

        self.last_throttle = throttle
        self.last_brake = brake
        self.last_steer = steer


    def current_velocity_cb(self, msg):
        self.current_velocity = msg


    def twist_cmd_cb(self, msg):
        self.proposed_velocities = msg


    def dbw_enabled_cb(self, msg):
        if (self.activated != msg.data):
            rospy.logwarn("%s has been %s",
                          rospy.get_name(),
                          "activated" if msg.data else "deactivated")

        self.activated = msg.data


    def current_pose_cb(self, msg):
        with self.lock:
            self.current_pose = msg.pose


    def waypoints_cb(self, msg):
        with self.lock:
            self.waypoints = msg.waypoints


if __name__ == '__main__':
    DBWNode()
