#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import scipy.signal
import imu_data_logger


class ButterworthFilter(object):

    def initialize(self, order=6, cutoff_freq=.01):
        self.b, self.a = scipy.signal.butter(order, cutoff_freq)
        self.logger = imu_data_logger.Logger()
        self.logger.initialize()
        self.seq = 0 #used to tag messages so they can be put in the right order later
        self.pub = rospy.Publisher("nav_filtered_signals", sensor_msgs.msg.Imu, queue_size=2000)
        while not rospy.is_shutdown():
            if(self.logger.length() != 0):
                self.filter(self.logger.get_next_message_group())
            rospy.sleep(1)

    def filter(self, data):
        data = self.prepare_data(data)
        output = self.apply_filter(data)
        self.construct_and_send_messages(output)

    def prepare_data(self, data):
        data = self.remove_gravity_from_data(data)
        return data

    def remove_gravity_from_data(self, data):
        for data_point in data:
            x = 9.81*2*(data.orientation.y*data.orientation.w-data.orientation.x*data.orientation.z)
            y = 9.81*2*(data.orientation.x*data.orientation.w+data.orientation.z*data.orientation.y)
            z = 9.81*(data.orientation.x*data.orientation.x+data.orientation.y*data.orientation.y-data.orientation.z*data.orientation.z-data.orientation.w*data.orientation.w)
            data.linear_acceleration.x = data.linear_acceleration.x+x
            data.linear_acceleration.y = data.linear_acceleration.y+y
            data.linear_acceleration.z = data.linear_acceleration.z+z
            #print("%1.5f %1.5f %1.5f" % (abs(x), abs(y), abs(z)))
        return data

    def apply_filter(self, data):
        x = []
        y = []
        z = []
        time = []
        for data_point in data:
            x.append(data_point.linear_acceleration.x)
            y.append(data_point.linear_acceleration.y)
            z.append(data_point.linear_acceleration.z)
            time.append(data_point.header.stamp)
        x_output = scipy.signal.lfilter(self.b, self.a, x)
        y_output = scipy.signal.lfilter(self.b, self.a, y)
        z_output = scipy.signal.lfilter(self.b, self.a, z)
        return (x_output, y_output, z_output, time)

    def construct_and_send_messages(self, data):
        for data_point in data:
            msg = sensor_msgs.msg.Imu()
            msg.header.seq = self.seq
            msg.header.stamp = data_point[3]
            msg.header.frame_id = "filter"
            msg.linear_acceleration.x = data_point[0]
            msg.linear_acceleration.y = data_point[1]
            msg.linear_acceleration.z = data_point[2]
            self.pub.publish(msg)
            self.seq += 1


if __name__ == '__main__':
    rospy.init_node("seabee_imu_filter")
    filter = ButterworthFilter()
    filter.initialize()
