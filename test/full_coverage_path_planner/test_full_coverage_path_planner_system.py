#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32
from tracking_pid.msg import traj_point, PidDebug
import math
import numpy as np
import rospy
import sys
import tf
import unittest

PKG = 'tracking_pid'


class TestTrackingPID(unittest.TestCase):

    def setUp(self):
        rospy.init_node("rostest_tracking_pid_node")
        self.trajectory_finished_sub = rospy.Subscriber("trajectory_finished", Bool,
                                                        self.trajectory_finished_callback, queue_size=1)
        self.listener = tf.TransformListener()
        self.trajectory_finished = False

    def trajectory_finished_callback(self, trajectory_finished_msg):
        rospy.loginfo("Trajectory finished message received on topic")
        self.trajectory_finished = trajectory_finished_msg.data

    def quaternion_to_yaw(self, quaternion_in):
        quaternion = (
            quaternion_in.x,
            quaternion_in.y,
            quaternion_in.z,
            quaternion_in.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return (euler[2] + math.pi) % (2 * math.pi) - math.pi  # wrap

    def test_tracking_pid(self):
        """ Several checks are done:
        - Test that interpolator point and robot start moving
        - Test that error at all times is bounded
        - Test that after some time final goal is reached
        A path that does not start along the y-axis is expected
        """
        p1_msg = rospy.wait_for_message("trajectory", traj_point, timeout=5)
        p1_yaw = self.quaternion_to_yaw(p1_msg.pose.pose.orientation)
        self.listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        (trans1, rot1) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        rospy.sleep(2.0)
        p2_msg = rospy.wait_for_message("trajectory", traj_point, timeout=5)
        p2_yaw = self.quaternion_to_yaw(p2_msg.pose.pose.orientation)

        if (p1_msg.pose.pose.position.x == p2_msg.pose.pose.position.x and
                p1_msg.pose.pose.position.y == p2_msg.pose.pose.position.y and
                p1_yaw == p2_yaw):
            self.assertTrue(False, "Trajectory point has not moved")

        rospy.loginfo("Wait max 140 seconds for reaching goal")
        test_start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - test_start_time < 140.0:
            self.debug_msg = rospy.wait_for_message("debug", PidDebug, timeout=5)
            error_vec = (
                self.debug_msg.error.linear.x,
                self.debug_msg.error.linear.y,
                self.debug_msg.error.linear.z)
            error = np.linalg.norm(error_vec)
            self.assertLess(error, 0.5, "Linear error greater than 0.5 m")
            yaw_error = self.debug_msg.error.angular.z
            self.assertLess(yaw_error, 0.5, "Linear error greater than 0.5 rad")
            self.coverage_msg = rospy.wait_for_message("coverage_progress", Float32, timeout=5)
            if self.trajectory_finished is True:
                break

        self.assertTrue(self.trajectory_finished, "Trajectory not finished in 140 seconds")
        self.assertGreaterEqual(self.coverage_msg.data, 0.95, "Coverage less than 95%")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'rostest_tracking_pid_node', TestTrackingPID)
