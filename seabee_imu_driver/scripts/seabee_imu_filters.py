#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import scipy.signal
import copy


class FilterStack(object):

    def initialize(self):
        filter_width = 50
        message_pack_update_size = 100
        initial_message_pack_size = 200
        self.logger = Logger()
        self.logger.initialize(initial_message_pack_size=initial_message_pack_size, message_pack_update_size=message_pack_update_size)
        self.filters = self.initialize_filters(filter_width=filter_width, message_pack_update_size=message_pack_update_size)
        self.pub = rospy.Publisher("nav_filtered_signals/filter_stack", sensor_msgs.msg.Imu, queue_size=2000)
        self.seq = 0
        while not rospy.is_shutdown():
            if(self.logger.length() != 0):
                self.filter(self.logger.get_next_message_group())
            rospy.sleep(.1)

    def initialize_filters(self, filter_width, message_pack_update_size):
        filter_list = []
        moving_average_filter = MovingAverageFilter()
        moving_average_filter.initialize(average_width=filter_width, message_pack_update_size=message_pack_update_size)
        filter_list.append(moving_average_filter)
        return filter_list

    def filter(self, data):
        prepared_data = self.prepare_data(data)
        output = self.apply_filters(prepared_data)
        self.construct_and_send_messages(output)

    def prepare_data(self, data):
        prepared_data = self.remove_gravity_from_data(data)
        return prepared_data

    def remove_gravity_from_data(self, data):
        for data_point in data:
            x1 = data_point.orientation.x
            y1 = data_point.orientation.y
            z1 = data_point.orientation.z
            w1 = data_point.orientation.w
            x = 9.81*2*(y1*w1-x1*z1)
            y = 9.81*-2*(x1*w1+y1*z1)
            z = 9.81*(x1*x1+y1*y1-z1*z1-w1*w1)
            data_point.linear_acceleration.x = data_point.linear_acceleration.x+x
            data_point.linear_acceleration.y = data_point.linear_acceleration.y+y
            data_point.linear_acceleration.z = data_point.linear_acceleration.z+z
        return data

    def apply_filters(self, data):
        for curr_filter in self.filters:
            data = curr_filter.apply_filter(data)
        return data

    def construct_and_send_messages(self, data):
        for data_point in data[50:150]:
            data_point.header.frame_id = "felter"
            data_point.header.seq = self.seq
            self.pub.publish(data_point)
            self.seq += 1


class BaseFilter(object):

    def initialize(self):
        pass

    def filter(self, data):
        pass


class ButterworthFilter(BaseFilter):

    def initialize(self, order=4, cutoff_freq=.01, message_pack_size=2000, message_pack_update_size=1000):
        self.b, self.a = scipy.signal.butter(order, cutoff_freq)
        self.message_pack_size = message_pack_size
        self.message_pack_update_size = message_pack_update_size
        self.seq = 0

    def filter(self, data):
        data = self.prepare_data(data)
        output = self.apply_filter(data)

    def prepare_data(self, data):
        return data

    def apply_filter(self, data):
        x = []
        y = []
        z = []
        for data_point in data:
            x.append(data_point.linear_acceleration.x)
            y.append(data_point.linear_acceleration.y)
            z.append(data_point.linear_acceleration.z)
        x_output = scipy.signal.lfilter(self.b, self.a, x)
        y_output = scipy.signal.lfilter(self.b, self.a, y)
        z_output = scipy.signal.lfilter(self.b, self.a, z)
        for index in range(0, len(data)):
            data[index].linear_acceleration.x = x_output[index]
            data[index].linear_acceleration.y = y_output[index]
            data[index].linear_acceleration.z = z_output[index]
        return data

    def construct_and_send_messages(self, data):
        for index in range(500, 1500):
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


class MovingAverageFilter(BaseFilter):

    def initialize(self, average_width=50, message_pack_update_size=100):
        self.seq = 0
        self.average_width = average_width
        self.message_pack_update_size = message_pack_update_size

    def filter(self, data):
        data = self.prepare_data(data)
        output = self.apply_filter(data)

    def prepare_data(self, data):
        return data

    def apply_filter(self, data):
        num_data_points = len(data)
        midpoint_of_data = num_data_points/2
        filter_width = self.average_width  # the number of points we average at a time
        num_points_to_filter = self.message_pack_update_size
        x = []
        y = []
        z = []
        for index in range(midpoint_of_data-num_points_to_filter/2, midpoint_of_data+num_points_to_filter/2):
            x_avg = sum(point.linear_acceleration.x for point in data[index-self.average_width/2:index+self.average_width/2])/(self.average_width)
            y_avg = sum(point.linear_acceleration.y for point in data[index-self.average_width/2:index+self.average_width/2])/(self.average_width)
            z_avg = sum(point.linear_acceleration.z for point in data[index-self.average_width/2:index+self.average_width/2])/(self.average_width)
            x.append(x_avg)
            y.append(y_avg)
            z.append(z_avg)
        for index in range(midpoint_of_data-num_points_to_filter/2, midpoint_of_data+num_points_to_filter/2):
            data[index].linear_acceleration.x = x[index-(midpoint_of_data-num_points_to_filter/2)] #if (x[index-(len(data)/2-self.average_width)] > .05 or x[index-(len(data)/2-self.average_width)] < -.05) else 0
            data[index].linear_acceleration.y = y[index-(midpoint_of_data-num_points_to_filter/2)] #if (abs(y[index-(len(data)/2-self.average_width)]) > .02) else 0
            data[index].linear_acceleration.z = z[index-(midpoint_of_data-num_points_to_filter/2)] #if (abs(z[index-(len(data)/2-self.average_width)]) > .02) else 0
        return data

    def construct_and_send_messages(self, data):
        for data_point in data[500:1499]:
            print(len(data))
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

    def initialize(self, initial_message_pack_size=200, message_pack_update_size=100):
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
            self.data_list.append(copy.deepcopy(self.current_message_pack))
            self.current_message_pack = self.current_message_pack[self.message_pack_update_size:self.initial_message_pack_size]

    def get_next_message_group(self):
        next_pack = self.data_list.pop(0)
        return next_pack

    def length(self):
        length = len(self.data_list)
        return length


if __name__ == '__main__':
        rospy.init_node("seabee_imu_filter")
        filter_stack = FilterStack()
        filter_stack.initialize()
