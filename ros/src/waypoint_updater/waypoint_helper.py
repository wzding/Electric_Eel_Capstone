"""
Helper functions for the waypoint updater.
"""

import math
import tf

def is_waypoint_behind_pose(pose, waypoint):
    """Check that waypoint is ahead of given pose w.r.t. to pose direction."""
    _, _, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                          pose.orientation.y,
                                                          pose.orientation.z,
                                                          pose.orientation.w])

    shift_x = waypoint.pose.pose.position.x - pose.position.x
    shift_y = waypoint.pose.pose.position.y - pose.position.y

    shifted_rotated_x = shift_x * math.cos(0 - yaw) - shift_y * math.sin(0 - yaw)

    return False if shifted_rotated_x > 0 else True
