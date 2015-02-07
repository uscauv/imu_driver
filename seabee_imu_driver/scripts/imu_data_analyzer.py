#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import math
import tf.transformations


class Analyzer(object):

    def __init__(self):
        self.num_calls = 0
        self.orien_total = 0
        self.ang_total = 0
        self.acc_total = 0

    def callback(self, data):
        self.num_calls += 1
        self.orien_total += self.magnitude(data.orientation.x, data.orientation.y, data.orientation.z)
        self.ang_total += self.magnitude(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)
        self.acc_total += self.magnitude(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z)
        angles = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w], axes="sxyz")
        #print(angles)
        #9.8*math.sin(2*math.pi*math.fabs(angles[0])/3.14)+9.8*math.sin(2*math.pi*math.fabs(angles[1])/1.56)+9.8*math.sin(2*math.pi*math.fabs(angles[2])/3.13)
        #print(str(angles[0]/3.16*math.pi) + " " + str(angles[1]/1.56*math.pi/2) + " "  + str(math.sin(angles[0]/3.16*math.pi)) + " " + str(math.sin(angles[1]/1.56*math.pi/2)) + " " + str(9.8*math.sin(angles[0]/3.16*math.pi)) + " " + str(9.8*math.sin(angles[1]/1.56*math.pi/2)))
        print(str(9.8*math.sin(angles[0]/3.16*math.pi)) + " " + str(9.8*math.sin(angles[1]/1.56*math.pi/2)))

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
    rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, analy.callback)
    try:
        while not rospy.is_shutdown():
            rospy.sleep(100)
    finally:
        analy.print_values()
