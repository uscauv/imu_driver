#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import scipy.signal
import sys


class FilterStack(object):

    def initialize(self):
        self.filters = self.initialize_filters()
        self.logger = Logger().initialize()
        while not rospy.is_shutdown():
            if(self.logger.length() != 0):
                self.filter(self.logger.get_next_message_group())
            rospy.sleep(1)

    def initialize_filters(self):
        filter_list = []
        butterworth_filter = ButterworthFilter().initialize()
        moving_average_filter = MovingAverageFilter().initialize()

        filter_list.append(moving_average_filter)
        filter_list.append(butterworth_filter)

        return filter_list

    def filter(self, data):
        prepared_data = self.prepare_data(data)
        output = self.apply_filters(prepared_data)
        self.construct_and_send_messages(output)

    def prepare_data(self, data):
        return self.remove_gravity_from_data(data)

    def remove_gravity_from_data(self, data):
        for index in range(1000, len(data)-1):
            data_point = data[index]
            x1 = data_point.orientation.x
            y1 = data_point.orientation.y
            z1 = data_point.orientation.z
            w1 = data_point.orientation.w
            x = 9.81*2*(y1*w1-x1*z1)
            y = 9.81*-2*(x1*w1+y1*z1)
            z = 9.81*(x1*x1+y1*y1-z1*z1-w1*w1)
            data[index].linear_acceleration.x = data_point.linear_acceleration.x+x
            data[index].linear_acceleration.y = data_point.linear_acceleration.y+y
            data[index].linear_acceleration.z = data_point.linear_acceleration.z+z
        return data

    def apply_filters(self, data):
        for curr_filter in self.filters:
            data  = curr_filter.filter(data)
        return data

    def construct_and_send_messages(self, data):
        for index in range(500, 1499):
            data_point = data[index]
            msg = sensor_msgs.msg.Imu()
            msg.header.seq = self.seq
            msg.header.stamp = data_point[3]
            msg.header.frame_id = "filter"
            msg.linear_acceleration.x = data_point[0]  # if (abs(data_point[0]) > .02) else 0
            msg.linear_acceleration.y = data_point[1]
            msg.linear_acceleration.z = data_point[2]
            print(data_point[0])
            self.pub.publish(msg)
            self.seq += 1
        


class ButterworthFilter(object):

    def initialize(self, order=6, cutoff_freq=.01, manual_data_feed=False):
        self.b, self.a = scipy.signal.butter(order, cutoff_freq)
        self.logger = Logger()
        self.logger.initialize()
        self.seq = 0 #used to tag messages so they can be put in the right order later
        self.pub = rospy.Publisher("nav_filtered_signals/butterworth", sensor_msgs.msg.Imu, queue_size=2000)
        if not manual_data_feed:
            while not rospy.is_shutdown():
                if(self.logger.length() != 0):
                    self.filter(self.logger.get_next_message_group(), manual_data_feed)
                rospy.sleep(1)

    def filter(self, data, manual_data_feed):
        if not manual_data_feed:
            data = self.prepare_data(data)
        output = self.apply_filter(data)
        self.construct_and_send_messages(output)

    def prepare_data(self, data):
        #data = self.average_orientation(data)
        data = self.remove_gravity_from_data(data)
        return data

    def average_orientation(self, data):
        average_width = 100
        for index in range(average_width/2+1000, len(data)+average_width/2+1999):
            data[index-average_width/2].orientation.x = sum([data_point.orientation.x for data_point in data[index-average_width/2:index+average_width/2]])/average_width
            data[index-average_width/2].orientation.y = sum([data_point.orientation.y for data_point in data[index-average_width/2:index+average_width/2]])/average_width
            data[index-average_width/2].orientation.z = sum([data_point.orientation.z for data_point in data[index-average_width/2:index+average_width/2]])/average_width
            data[index-average_width/2].orientation.w = sum([data_point.orientation.w for data_point in data[index-average_width/2:index+average_width/2]])/average_width
        return data

    def remove_gravity_from_data(self, data):
        for index in range(1000, len(data)-1):
            data_point = data[index]
            x1 = data_point.orientation.x
            y1 = data_point.orientation.y
            z1 = data_point.orientation.z
            w1 = data_point.orientation.w
            x = 9.81*2*(y1*w1-x1*z1)
            y = 9.81*-2*(x1*w1+y1*z1)
            z = 9.81*(x1*x1+y1*y1-z1*z1-w1*w1)
            data[index].linear_acceleration.x = data_point.linear_acceleration.x+x
            data[index].linear_acceleration.y = data_point.linear_acceleration.y+y
            data[index].linear_acceleration.z = data_point.linear_acceleration.z+z
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
        return_data = []
        for index in range(0,len(x_output)):
            return_data.append((x_output[index], y_output[index], z_output[index], time[index]))
        return return_data

    def construct_and_send_messages(self, data):
        for index in range(500, 1499):
            data_point = data[index]
            msg = sensor_msgs.msg.Imu()
            msg.header.seq = self.seq
            msg.header.stamp = data_point[3]
            msg.header.frame_id = "filter"
            msg.linear_acceleration.x = data_point[0]  # if (abs(data_point[0]) > .02) else 0
            msg.linear_acceleration.y = data_point[1]
            msg.linear_acceleration.z = data_point[2]
            print(data_point[0])
            self.pub.publish(msg)
            self.seq += 1


class MovingAverageFilter(object):

    def initialize(self, average_width=50, feed_to_filter=None):
        self.logger = Logger()
        self.logger.initialize()
        self.seq = 0
        if(feed_to_filter is not None):
            self.data_to_pass_on = []
        self.pub = rospy.Publisher("nav_filtered_signals/moving_average", sensor_msgs.msg.Imu, queue_size=2000)
        while not rospy.is_shutdown():
            if(self.logger.length() != 0):
                self.filter(self.logger.get_next_message_group(), feed_to_filter)
            rospy.sleep(1)

    def filter(self, data, feed_to_filter):
        data = self.prepare_data(data)
        output = self.apply_filter(data)
        self.construct_and_send_messages(output)
        if(feed_to_filter is not None):
            self.data_to_pass_on.append(output)
            if(self.seq > 3):
                feed_to_filter.filter(self.data_to_pass_on)
                self.data_to_pass_on = self.data_to_pass_on[500:2000]

    def prepare_data(self, data):
        data = self.remove_gravity_from_data(data)
        return data

    def remove_gravity_from_data(self, data):
        for index in range(1000, len(data)-1):
            data_point = data[index]
            x1 = data_point.orientation.x
            y1 = data_point.orientation.y
            z1 = data_point.orientation.z
            w1 = data_point.orientation.w
            x = 9.81*2*(y1*w1-x1*z1)
            y = 9.81*-2*(x1*w1+y1*z1)
            z = 9.81*(x1*x1+y1*y1-z1*z1-w1*w1)
            data[index].linear_acceleration.x = data_point.linear_acceleration.x+x
            data[index].linear_acceleration.y = data_point.linear_acceleration.y+y
            data[index].linear_acceleration.z = data_point.linear_acceleration.z+z
        return data

    def apply_filter(self, data):
        x = []
        y = []
        z = []
        time = []
        for index in range(500, 1499):
            data_point = data[index]
            x_avg = sum(point.linear_acceleration.x for point in data[index-50:index+50])/100
            y_avg = sum(point.linear_acceleration.y for point in data[index-50:index+50])/100
            z_avg = sum(point.linear_acceleration.z for point in data[index-50:index+50])/100
            x.append(x_avg)
            y.append(y_avg)
            z.append(z_avg)
            time.append(data[index].header.stamp)
        return_data = []
        for index in range(0, len(x)):
            return_data.append((x[index], y[index], z[index], time[index]))
        return return_data

    def construct_and_send_messages(self, data):
        for data_point in data:
            msg = sensor_msgs.msg.Imu()
            msg.header.seq = self.seq
            msg.header.stamp = data_point[3]
            msg.header.frame_id = "filter"
            msg.linear_acceleration.x = data_point[0]  # if (abs(data_point[0]) > .02) else 0
            msg.linear_acceleration.y = data_point[1]
            msg.linear_acceleration.z = data_point[2]
            self.pub.publish(msg)
            self.seq += 1


class Logger(object):

    def initialize(self, initial_message_pack_size=2000, message_pack_update_size=1000):
        self.data_list = []
        self.current_message_pack = []
        self.iteration = 0
        self.initial_message_pack_size = initial_message_pack_size
        self.message_pack_update_size = message_pack_update_size
        self.first_iteration = True
        rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, self.log_message)

    def log_message(self, data):
        self.current_message_pack.append(data)
        self.iteration += 1
        if((self.iteration == self.message_pack_update_size and not self.first_iteration) or (self.first_iteration and self.iteration == self.initial_message_pack_size)):
            self.iteration = 0
            self.first_iteration = False
            self.data_list.append(self.current_message_pack)
            self.current_message_pack = self.current_message_pack[self.message_pack_update_size:self.initial_message_pack_size]

    def get_next_message_group(self):
        next_pack = self.data_list.pop(0)
        return next_pack

    def length(self):
        length = len(self.data_list)
        return length


if __name__ == '__main__':
    if(sys.argv[1] == "moving_average"):
        rospy.init_node("seabee_imu_moving_average_filter")
        butterworth_filter = ButterworthFilter()
        butterworth_filter.initialize(manual_data_feed=True)
        moving_average_filter = MovingAverageFilter()
        moving_average_filter.initialize(feed_to_filter=butterworth_filter)
    if(sys.argv[1] == "butterworth"):
        rospy.init_node("seabee_imu_butterworth_filter")
        butterworth_filter = ButterworthFilter()
        butterworth_filter.initialize()
