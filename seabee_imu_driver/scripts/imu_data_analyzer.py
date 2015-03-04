#!/usr/bin/env python

##  DEFUNCT AND NOT CURRENTLY USED FOR ANYTHING

import rospy
import sensor_msgs.msg
import nav_msgs.msg
import math
import tf.transformations


class Analyzer(object):

    def __init__(self):
        self.num_calls = 0
        self.orien_total = 0
        self.ang_total = 0
        self.acc_total = 0
        self.pub = rospy.Publisher("/odom_analysis", sensor_msgs.msg.Imu, queue_size=10)

    def callback(self, data):
        print(data.pose.pose.position.x)
        """angles = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w], axes="sxyz")
        msg = sensor_msgs.msg.Imu()
        msg.header.frame_id = '/base_imu'
        msg.header.stamp = rospy.Time.now()
        #msg.orientation.x = 9.8*math.sin(2*math.pi*math.fabs(angles[0])/3.14)
        #msg.orientation.y = 9.8*math.sin(2*math.pi*math.fabs(angles[1])/1.56)
        #msg.orientation.z = 9.8*math.sin(2*math.pi*math.fabs(angles[2])/3.13)
        msg.orientation.x = angles[0]
        msg.orientation.y = angles[1]
        msg.orientation.z = angles[2]
        msg.linear_acceleration.x = data.linear_acceleration.x
        msg.linear_acceleration.y = data.linear_acceleration.y
        msg.linear_acceleration.z = data.linear_acceleration.z
        self.pub.publish(msg)
        #print(str(angles[0]/3.16*math.pi) + " " + str(angles[1]/1.56*math.pi/2) + " "  + str(math.sin(angles[0]/3.16*math.pi)) + " " + str(math.sin(angles[1]/1.56*math.pi/2)) + " " + str(9.8*math.sin(angles[0]/3.16*math.pi)) + " " + str(9.8*math.sin(angles[1]/1.56*math.pi/2)))
        #3print(str(9.8*math.sin(angles[0]/3.16*math.pi)) + " " + str(9.8*math.sin(angles[1]/1.56*math.pi/2)))"""

    def magnitude(self, x, y, z):
        return math.sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2))

    def print_values(self):
        print("Printing values")
        print("Num calls: " + self.num_calls)
        print("Orien_total = " + self.orien_total)
        print("Ang_total = " + self.ang_total)
        print("Acc_total = " + self.acc_total)


if __name__ == "__main__":
    rospy.init_node("data_analyzer")
    analy = Analyzer()
    rospy.Subscriber("nav_filtered_signals", nav_msgs.msg.Odometry, analy.callback)
    rospy.spin()
