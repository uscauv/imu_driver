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
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        # y =2*(y*z+x*w)
        # x = 2*(y*w-x*z)
        # z = (x*x+y*y-z*z-w*w)
        print(2*(y*z-x*w)*9.81)

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
    rospy.Subscriber("imu/data", sensor_msgs.msg.Imu, analy.callback)
    rospy.spin()
