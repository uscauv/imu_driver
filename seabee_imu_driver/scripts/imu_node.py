#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import nav_msgs.msg

import seabee_imu_integrator


class Imu(object):

    def launch(self):
        self.seq = 0
        rospy.init_node("Imu")
        self.integrator = seabee_imu_integrator.BasicIntegrator()
        self.integrator.instantiate_values()
        self.pub = rospy.Publisher("odom", nav_msgs.msg.Odometry, queue_size=50)
        rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, self.callback)  # may need to be changed depending on the data published
        rospy.spin()

    def callback(self, data):
        self.integrator.update_values(data)
        odom_msg = self.create_odom_msg(data)

        #rospy.loginfo(odom_msg)
        #rospy.loginfo(self.integrator.delta_t)
        self.pub.publish(odom_msg)

    def create_odom_msg(self, data):
        odom_msg = nav_msgs.msg.Odometry()
        # header
        odom_msg.header.seq = self.seq
        self.seq += 1
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"

        odom_msg.child_frame_id = "seabee_base"
        # position
        odom_msg.pose.pose.position.x = self.integrator.position_x
        odom_msg.pose.pose.position.y = self.integrator.position_y
        odom_msg.pose.pose.position.z = self.integrator.position_z
        odom_msg.pose.pose.orientation.x = self.integrator.orientation_x
        odom_msg.pose.pose.orientation.y = self.integrator.orientation_y
        odom_msg.pose.pose.orientation.z = self.integrator.orientation_z
        odom_msg.pose.pose.orientation.w = self.integrator.orientation_w
        #odom_msg.pose.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # not sure about this
        # twist
        odom_msg.twist.twist.linear.x = self.integrator.velocity_x
        odom_msg.twist.twist.linear.y = self.integrator.velocity_y
        odom_msg.twist.twist.linear.z = self.integrator.velocity_z
        odom_msg.twist.twist.angular.x = self.integrator.velocity_w_x
        odom_msg.twist.twist.angular.y = self.integrator.velocity_w_y
        odom_msg.twist.twist.angular.z = self.integrator.velocity_w_z
        #odom_msg.twist.covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # not sure about this

        return odom_msg


if __name__ == "__main__":
    imu = Imu()
    imu.launch()
