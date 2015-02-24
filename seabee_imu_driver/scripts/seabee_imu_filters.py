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

    def initialize(self, order=1, nyquist_freq=460800):
        self.b, self.a = scipy.signal.butter(order, nyquist_freq)
        self.data = []
        self.num = 0
        self.seq = 0
        self.pub = rospy.Publisher("nav_filtered_signals", nav_msgs.msg.Odometry, queue_size=50)
        rospy.spin()

    def log_data(self, data):
        print("data logged")
        self.data.append(data.linear_acceleration.x)
        self.num += 1
        if(self.num == 5):
            self.filter(data)

    def filter(self, input_signal):
        print("filtered")
        output_signal = scipy.signal.lfilter(self.b, self.a, input_signal, axis=float)
        for x in output_signal:
            msg = nav_msgs.msg.Odometry()
            msg.header.seq = self.seq
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "filter"
            msg.pose.pose.position.x = x
            self.pub.publish(msg)
        self.num = 0
        self.data = []


if __name__ == '__main__':
    rospy.init_node("seabee_imu_filter")
    filter = ButterworthFilter()
    rospy.Subscriber( "/imu/data", sensor_msgs.msg.Imu, filter.log_data)
    filter.initialize()
