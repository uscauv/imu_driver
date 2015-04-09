#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import nav_msgs.msg

import seabee_imu_integrator
from seabee_imu_driver.srv import *


class Imu(object):

    def launch(self):
        self.seq = 0
        rospy.init_node("Imu")
        rospy.Service("reset_odom", SeabeeResetOdom, self.reset_handle)
        rospy.Subscriber("nav_filtered_signals/filter_stack", sensor_msgs.msg.Imu, self.callback)
        self.pub = rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=50)
        self.integrator = seabee_imu_integrator.TrapezoidalIntegrator()
        self.integrator.instantiate_values()
        rospy.spin()

    def callback(self, data):
        self.integrator.update_values(data)
        odom_msg = self.create_odom_msg(data)
        self.pub.publish(odom_msg)

    def create_odom_msg(self, data):
        odom_msg = nav_msgs.msg.Odometry()
        self.fill_odom_msg_header(odom_msg, data)
        self.fill_odom_msg_pose(odom_msg)
        self.fill_odom_msg_twist(odom_msg)
        odom_msg.child_frame_id = "seabee_base"
        return odom_msg

    def fill_odom_msg_header(self, msg, data):
        msg.header.seq = self.seq
        self.seq += 1
        msg.header.stamp = data.header.stamp
        msg.header.frame_id = "odom"

    def fill_odom_msg_pose(self, msg):
        msg.pose.pose.position.x = self.integrator.position_x
        msg.pose.pose.position.y = self.integrator.position_y
        msg.pose.pose.position.z = self.integrator.position_z
        msg.pose.pose.orientation.x = self.integrator.orientation_x
        msg.pose.pose.orientation.y = self.integrator.orientation_y
        msg.pose.pose.orientation.z = self.integrator.orientation_z
        msg.pose.pose.orientation.w = self.integrator.orientation_w
        #covariance ignored (left at default [0])

    def fill_odom_msg_twist(self, msg):
        msg.twist.twist.linear.x = self.integrator.velocity_x
        msg.twist.twist.linear.y = self.integrator.velocity_y
        msg.twist.twist.linear.z = self.integrator.velocity_z
        msg.twist.twist.angular.x = self.integrator.velocity_w_x
        msg.twist.twist.angular.y = self.integrator.velocity_w_y
        msg.twist.twist.angular.z = self.integrator.velocity_w_z
        #covariance ignored (left at default [0])

    def reset_handle(self, msg):
        self.integrator.instantiate_values()

if __name__ == "__main__":
    imu = Imu()
    imu.launch()
