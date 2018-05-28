#!/usr/bin/env python
"""
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
"""
import math
import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane
from helper import is_waypoint_behind_pose


LOOKAHEAD_WPS = 200 #Number of waypoints we will publish
MAX_SPEED = 10
lambda_fct = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

class WaypointUpdater(object):
    """WaypointUpdater computes the Lane the car should follow."""

    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

        # Done: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.current_frame_id = None
        self.base_waypoints = None
        self.len_base_waypoints = 0
        self.seq = 0
        self.current_waypoint_ahead = None
        self.closest_obstacle = None
        self.current_velocity = 0
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_waypoints_ahead()
            rate.sleep()


    def pose_cb(self, msg):
        """Update the state of the vehicle and which frame is the current one."""
        self.current_pose = msg.pose
        self.current_frame_id = msg.header.frame_id
        if self.base_waypoints is None:
            return


    def waypoints_cb(self, waypoints):
        """Sets the callbacks in this object."""
        self.base_waypoints = waypoints.waypoints
        self.len_base_waypoints = len(self.base_waypoints)


    def traffic_cb(self, msg):
        """Callback to get the position of the next traffic light."""
        self.closest_obstacle = msg.data


    def velocity_cb(self, msg):
        """Callback to get the current speed of the vehicle."""
        self.current_velocity = msg.twist.linear.x


    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message. We will implement it later
        pass


    def get_waypoint_velocity(self, waypoint):
        """Unwrap the waypoint to return the value of the linear speed."""
        return waypoint.twist.twist.linear.x


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        """Unwraps the waypoint object to set the value for the linear speed."""
        waypoints[waypoint].twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        """Calculates the euclidean distance between two waypoints given."""
        dist = 0
        for i in xrange(wp1, wp2+1):
            dist += lambda_fct(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


    def get_waypoint_indices(self, start_index, length):
        """Computes a cyclic list of waypoint indices.

        Args:
        start_index (int): Initial index of the list
        length (int): Desired length of resulting list, defaults to length of base points list

        Returns:
        cyclic list of waypoint indices
        """
        # making sure that length does not overpass base waypoints length
        length = min(self.len_base_waypoints, length)
        end_index = start_index + length
        q, r = divmod(end_index, self.len_base_waypoints)
        # q can either be 0 or 1
        if q == 0:
            return range(start_index, r)
        return range(start_index, self.len_base_waypoints) + range(0, r)


    def closest_waypoint_index(self):
        """ Computes the index of closest waypoint w.r.t current position."""

        rospy.logdebug("computing closest_waypoint_index for pos %d, %d",
                       self.current_pose.position.x,
                       self.current_pose.position.y)

        if self.current_waypoint_ahead is None:
            possible_waypoint_indices = self.get_waypoint_indices(0,
                                                                   self.len_base_waypoints)
            closest_distance = float('inf')
        else:
            possible_waypoint_indices = self.get_waypoint_indices(self.current_waypoint_ahead, LOOKAHEAD_WPS)
            closest_distance = lambda_fct(self.base_waypoints[self.current_waypoint_ahead].pose.pose.position,
                                  self.current_pose.position)

        prev_index = possible_waypoint_indices.pop(0)
        closer_point_found = True

        while closer_point_found and len(possible_waypoint_indices) > 0:
            index = possible_waypoint_indices.pop(0)
            distance = lambda_fct(self.base_waypoints[index].pose.pose.position,
                          self.current_pose.position)

            if distance > closest_distance:
                closer_point_found = False
            else:
                closest_distance = distance
                prev_index = index

        while is_waypoint_behind_pose(self.current_pose, self.base_waypoints[prev_index]):
            prev_index += 1
            prev_index %= self.len_base_waypoints

        self.current_waypoint_ahead = prev_index

        return prev_index


    def publish_waypoints_ahead(self):
        """ Publishes a Lane of LOOKAHEAD_WPS waypoints /final_waypoint topic."""
        if self.base_waypoints is None or self.current_pose is None:
            return
        start_index = self.closest_waypoint_index()
        self.current_waypoint_ahead = start_index
        waypoint_indices = self.get_waypoint_indices(start_index, LOOKAHEAD_WPS)

        lane = Lane()
        lane.header.frame_id = self.current_frame_id
        lane.header.stamp = rospy.Time.now()
        lane.header.seq = self.seq
        lane.waypoints = [self.base_waypoints[i] for i in waypoint_indices]
        if self.closest_obstacle is None or  self.closest_obstacle == -1 or self.closest_obstacle > waypoint_indices[-1]:
            # There is no traffic light near us, go full speed
            self.current_velocity = self.current_velocity+0.5 if self.current_velocity < MAX_SPEED else self.current_velocity
            speeds = MAX_SPEED
        # There is a traffic light in front
        elif self.closest_obstacle != -1:
            # there is a traffic light
            distance = self.distance(lane.waypoints, 0, self.closest_obstacle - start_index)
            if self.current_velocity > 2:
                speeds = min(10, .15*(distance - 5))
            else:
                speeds = 0
        else:
            speeds = np.linspace(self.current_velocity, MAX_SPEED, 1 + (LOOKAHEAD_WPS // 8))
            full_speed = np.ones(7 * LOOKAHEAD_WPS // 8) * MAX_SPEED
            speeds = np.concatenate((speeds, full_speed))
        rospy.logdebug("wp_updater: published speed: {}".format(speeds))

        lookahead = LOOKAHEAD_WPS if len(lane.waypoints) > LOOKAHEAD_WPS else len(lane.waypoints)
        for i in xrange(lookahead):
            self.set_waypoint_velocity(lane.waypoints, i, speeds)
        self.final_waypoints_pub.publish(lane)
        self.seq += 1


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
