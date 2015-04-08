#!/usr/bin/env python

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
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = 0

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
            self.delta_t = (self.t_secs-self.t_secs_last) + (self.t_nsecs-self.t_nsecs_last)*pow(10, -9)
            self.t_secs_last = self.t_secs
            self.t_nsecs_last = self.t_nsecs
        self.update_interpreted_values()

    def update_direct_values(self, data):
        self.t_secs = data.header.stamp.secs
        self.t_nsecs = data.header.stamp.nsecs
        self.acceleration_x = data.linear_acceleration.x 
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

        self.position_x += self.velocity_x*self.delta_t if (abs(self.velocity_x) > .3) else 0
        self.position_y += self.velocity_y*self.delta_t
        self.position_z += self.velocity_z*self.delta_t

        self.position_w_x += self.velocity_w_x*self.delta_t
        self.position_w_y += self.velocity_w_y*self.delta_t
        self.position_w_z += self.velocity_w_z*self.delta_t


class TrapezoidalIntegrator(Integrator):

    def instantiate_values(self):
        super(TrapezoidalIntegrator, self).instantiate_values()
        self.last_msg = None

    def update_values(self, data):
        if(self.last_msg is None):
            self.last_msg = data
            return
        self.integrate(data)
        self.drift_filter_v2(data)
        self.last_msg = data

    def integrate(self, data):
        delta_t = 1.0*data.header.stamp.secs-1.0*self.last_msg.header.stamp.secs+1.0*(data.header.stamp.nsecs-self.last_msg.header.stamp.nsecs)*pow(10, -9)
        last_velocity = (self.velocity_x, self.velocity_y, self.velocity_z)
        self.velocity_x += (data.linear_acceleration.x + self.last_msg.linear_acceleration.x)/2.0*delta_t
        self.velocity_y += (data.linear_acceleration.y + self.last_msg.linear_acceleration.y)/2.0*delta_t
        self.velocity_z += (data.linear_acceleration.z + self.last_msg.linear_acceleration.z)/2.0*delta_t

        self.position_x += (self.velocity_x+last_velocity[0])/2.0*delta_t
        self.position_y += (self.velocity_y+last_velocity[1])/2.0*delta_t
        self.position_z += (self.velocity_z+last_velocity[2])/2.0*delta_t

    def drift_filter(self, data):
        self.velocity_x += -.001*abs(self.velocity_x)*math.sin(self.velocity_x*2*math.pi/.2)
        self.velocity_y += -.001*abs(self.velocity_y)*math.sin(self.velocity_y*2*math.pi/.2)
        self.velocity_z += -.001*abs(self.velocity_z)*math.sin(self.velocity_z*2*math.pi/.2)

    def drift_filter_v2(self, data):
        self.velocity_x += -.001*math.sin(self.velocity_x/(1+1000*pow(self.velocity_x, 4)))
        self.velocity_y += -.001*math.sin(self.velocity_y/(1+1000*pow(self.velocity_y, 4)))
        self.velocity_z += -.001*math.sin(self.velocity_z/(1+1000*pow(self.velocity_z, 4)))


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
        self.filter_noisy_data()
        self.integrate()

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
