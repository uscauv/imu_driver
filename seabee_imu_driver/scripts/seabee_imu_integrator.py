#!/usr/bin/env python

import rospy


class Integrator(object):

    def __init__(self):
        super.__init__()

    def instantiate_values(self):
        pass

    def update_values(self):
        pass

    def get_values(self):
        pass


class BasicIntegrator(Integrator):

    def __init__(self):
        pass

    def instantiate_values(self):
        self.t_secs_last = None
        self.delta_t = 10
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.position_w_x = 0
        self.position_w_y = 0
        self.position_w_z = 0
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_z = 0
        self.velocity_w_x = 0  # angular velocity
        self.velocity_w_y = 0
        self.velocity_w_z = 0
        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0

    def update_values(self, data):
        #direct values
        self.t_secs = data.header.stamp.secs
        self.t_nsecs = data.header.stamp.nsecs
        self.acceleration_x = data.linear_acceleration.x  # note that these include the accleration of gravity
        self.acceleration_y = data.linear_acceleration.y
        self.acceleration_z = data.linear_acceleration.z-9.8
        self.velocity_w_x = data.angular_velocity.x
        self.velocity_w_y = data.angular_velocity.y
        self.velocity_w_z = data.angular_velocity.z
        self.orientation_x = data.orientation.x
        self.orientation_y = data.orientation.y
        self.orientation_z = data.orientation.z
        self.orientation_w = data.orientation.w
        #interpreted values
        if(self.t_secs_last is None):
            print("in update values")
            self.t_secs_last = self.t_secs  # then this is our first data package
            self.t_nsecs_last = self.t_nsecs
            return
        else:
            self.delta_t = (self.t_secs-self.t_secs_last) + (1.0*self.t_nsecs-self.t_nsecs_last)/pow(10, 9)
            self.t_secs_last = self.t_secs
            self.t_nsecs_last = self.t_nsecs
            #print(self.delta_t)
        self.velocity_x += self.acceleration_x*self.delta_t
        self.velocity_y += self.acceleration_y*self.delta_t
        self.velocity_z += self.acceleration_z*self.delta_t

        self.position_x += self.acceleration_x*self.delta_t*self.delta_t
        self.position_y += self.acceleration_y*self.delta_t*self.delta_t
        self.position_z += self.acceleration_z*self.delta_t*self.delta_t
        self.position_w_x += self.velocity_w_x*self.delta_t
        self.position_w_y += self.velocity_w_y*self.delta_t
        self.position_w_z += self.velocity_w_z*self.delta_t

    def get_values(self):
        pass

    def integrate(self, data):
        pass
