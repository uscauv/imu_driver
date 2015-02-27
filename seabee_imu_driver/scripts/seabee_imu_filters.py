#!/usr/bin/env python

import numpy
import rospy
import sensor_msgs.msg
import nav_msgs.msg
import scipy.signal


class KalmanFilter(object):

    def makePosVelAccFilter(self):
        state_trans_matrix = numpy.matrix([[1, 0, 0, 1, 0, 0, 0, 0, 0],
                                           [0, 1, 0, 0, 1, 0, 0, 0, 0],
                                           [0, 0, 1, 0, 0, 1, 0, 0, 0],
                                           [0, 0, 0, 1, 0, 0, 1, 0, 0],
                                           [0, 0, 0, 0, 1, 0, 0, 1, 0],
                                           [0, 0, 0, 0, 0, 1, 0, 0, 1],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0]])
        control_matrix = numpy.matrix([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 1, 0, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 1, 0],
                                       [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        observation_matrix = numpy.matrix([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0, 0, 0, 0]])
        initial_state_estimate = numpy.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0])
        p = numpy.matrix([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 1, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 1]])
        q = numpy.matrix([[.01, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, .01, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, .01, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, .01, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, .01, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, .01, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, .01, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, .01, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, .01]])
        r = numpy.matrix([[.5, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, .5, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, .5, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, .5, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, .5, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, .5, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, .5, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, .5, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, .5]])
        self.initialize(state_trans_matrix,
                        control_matrix,
                        observation_matrix,
                        initial_state_estimate,
                        p,
                        q,
                        r)

    def makeAccFilter(self):
        raise Error("Not yet implemented")

    def initialize(self, _A, _B, _H, _x, _P, _Q, _R):
        self.A = _A                      # State transition matrix.
        self.B = _B                      # Control matrix.
        self.H = _H                      # Observation matrix.
        self.current_state_estimate = _x  # Initial state estimate.
        self.current_prob_estimate = _P  # Initial covariance estimate.
        self.Q = _Q                      # Estimated error in process.
        self.R = _R                      # Estimated error in measurements.

    def GetCurrentState(self):
        return self.current_state_estimate

    def Step(self, control_vector, measurement_vector):
        #---------------------------Prediction step-----------------------------
        predicted_state_estimate = self.A * self.current_state_estimate + self.B * control_vector
        predicted_prob_estimate = (self.A * self.current_prob_estimate) * numpy.transpose(self.A) + self.Q
        #--------------------------Observation step-----------------------------
        innovation = measurement_vector - self.H*predicted_state_estimate
        innovation_covariance = self.H*predicted_prob_estimate*numpy.transpose(self.H) + self.R
        #-----------------------------Update step-------------------------------
        kalman_gain = predicted_prob_estimate * numpy.transpose(self.H) * numpy.linalg.inv(innovation_covariance)
        self.current_state_estimate = predicted_state_estimate + kalman_gain * innovation
        # We need the size of the matrix so we can make an identity matrix.
        size = self.current_prob_estimate.shape[0]
        # eye(n) = nxn identity matrix.
        self.current_prob_estimate = (numpy.eye(size)-kalman_gain*self.H)*predicted_prob_estimate


class ButterworthFilter(object):

    def initialize(self, order=6, nyquist_freq=.04, analog=Truej;):
        self.b, self.a = scipy.signal.butter(order, nyquist_freq)
        print(self.b)
        print(self.a)
        self.x_accel = []
        self.y_accel = []
        self.z_accel = []
        self.time = []
        self.num = 0
        self.seq = 0
        self.pub = rospy.Publisher("nav_filtered_signals", sensor_msgs.msg.Imu, queue_size=50)
        rospy.spin()

    def log_data(self, data):
        self.x_accel.append(data.linear_acceleration.x)
        self.y_accel.append(data.linear_acceleration.y)
        self.z_accel.append(data.linear_acceleration.z)
        self.time.append(data.header.stamp)
        self.num += 1
        if(self.num == 1000):
            self.filter()

    def filter(self):
        print("*******************************************************")
        x_output = scipy.signal.lfilter(self.b, self.a, self.x_accel)
        y_output = scipy.signal.lfilter(self.b, self.a, self.y_accel)
        z_output = scipy.signal.lfilter(self.b, self.a, self.z_accel)
        iteration = 0
        for x in x_output:
            msg = nav_msgs.msg.Odometry()
            msg.header.seq = self.seq
            msg.header.stamp = self.time[iteration]
            msg.header.frame_id = "filter"
            msg.linear_acceleration.x = x
            msg.linear_acceleration.y = y_output[iteration]
            msg.linear_acceleration.z = z_output[iteration]
            self.pub.publish(msg)
            iteration += 1
        self.num = 0
        self.x_accel = []
        self.y_accel = []
        self.z_accel = []
        self.time = []


if __name__ == '__main__':
    rospy.init_node("seabee_imu_filter")
    filter = ButterworthFilter()
    rospy.Subscriber( "/imu/data", sensor_msgs.msg.Imu, filter.log_data)
    filter.initialize()
