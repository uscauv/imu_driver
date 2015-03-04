#!/usr/bin/env python

import rospy
import sensor_msgs.msg


class Logger(object):

    def initialize(self):
        self.data_list = []
        self.current_message_pack = []
        self.iteration = 0
        rospy.Subscriber("/imu/data", sensor_msgs.msg.Imu, self.log_message)

    def log_message(self, data):
        self.current_message_pack.append(data)
        self.iteration += 1
        if(self.iteration == 2000):
            self.iteration = 0
            self.data_list.append(self.current_message_pack)
            self.current_message_pack = []

    def get_next_message_group(self):
        next_pack = self.data_list.pop(0)
        return next_pack

    def length(self):
        length = len(self.data_list)
        return length
