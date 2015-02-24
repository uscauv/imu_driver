#!/usr/bin/env python

import tf.transformations
import math


class Integrator(object):

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

    def update_values(self):
        pass


class BasicIntegrator(Integrator):

    def update_values(self, data):
        self.update_direct_values(data)
        if(self.t_secs_last is None):
            self.t_secs_last = self.t_secs  # then this is our first data package
            self.t_nsecs_last = self.t_nsecs
            return
        else:
            self.delta_t = (self.t_secs-self.t_secs_last) + (self.t_nsecs-self.t_nsecs_last)/pow(10, 9)
            self.t_secs_last = self.t_secs
            self.t_nsecs_last = self.t_nsecs
        self.update_interpreted_values()

    def update_direct_values(self, data):
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

    def update_interpreted_values(self):
        self.integrate()

    def integrate(self):
        self.velocity_x += self.acceleration_x*self.delta_t
        self.velocity_y += self.acceleration_y*self.delta_t
        self.velocity_z += self.acceleration_z*self.delta_t

        self.position_x += self.velocity_x*self.delta_t
        self.position_y += self.velocity_y*self.delta_t
        self.position_z += self.velocity_z*self.delta_t

        self.position_w_x += self.velocity_w_x*self.delta_t
        self.position_w_y += self.velocity_w_y*self.delta_t
        self.position_w_z += self.velocity_w_z*self.delta_t


class LessBasicIntegrator(Integrator):

    def instantiate_values(self):
        super(LessBasicIntegrator, self).instantiate_values()
        self.x_accel_data = 0
        self.y_accel_data = 0
        self.num_its = 0

    def update_values(self, data):
        self.update_direct_values(data)
        if(self.t_secs_last is None):
            self.t_secs_last = self.t_secs  # then this is our first data package
            self.t_nsecs_last = self.t_nsecs
            return
        else:
            self.delta_t = (self.t_secs-self.t_secs_last) + (1.0*self.t_nsecs-self.t_nsecs_last)/pow(10, 9)
            self.t_secs_last = self.t_secs
            self.t_nsecs_last = self.t_nsecs
        self.update_interpreted_values(data)

    def update_direct_values(self, data):
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

    def update_interpreted_values(self, data):
        self.account_for_gravity(data)
        self.filter_noisy_data()
        self.integrate()

    def account_for_gravity(self, data):
        angles = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w], axes="sxyz")
        self.acceleration_x -= math.fabs(9.8*math.sin(angles[1]/1.56*math.pi/2))
        self.acceleration_y -= math.fabs(9.8*math.sin(angles[0]/3.16*math.pi))
        self.x_accel_data += self.acceleration_x
        self.y_accel_data += self.acceleration_y
        self.num_its += 1
        #print "{0:10f}  {1:10f}".format(self.acceleration_x, self.acceleration_y)
        #print(str(self.x_accel_data) + " " + str(self.y_accel_data) + " " + str(self.num_its))

    def filter_noisy_data(self):
        if(math.fabs(self.acceleration_x) < .0015):
            self.acceleration_x = 0
        if(math.fabs(self.acceleration_y) < .0015):
            self.acceleration_y = 0

    def integrate(self):
        self.velocity_x += self.acceleration_x*self.delta_t
        self.velocity_y += self.acceleration_y*self.delta_t
        self.velocity_z += self.acceleration_z*self.delta_t

        self.position_x += self.velocity_x*self.delta_t
        self.position_y += self.velocity_y*self.delta_t
        self.position_z += self.velocity_z*self.delta_t

        self.position_w_x += self.velocity_w_x*self.delta_t
        self.position_w_y += self.velocity_w_y*self.delta_t
        self.position_w_z += self.velocity_w_z*self.delta_t
