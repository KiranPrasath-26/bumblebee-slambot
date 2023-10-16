#!/usr/env/bin python3
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class KalmanFilter(object):

    def __init__(self):
        self.odom_sub_ = rospy.Subscriber("bumblebee_controller/odom_noisy", Odometry, self.odomCallback)
        self.imu_sub_ = rospy.Subscriber("/imu", Imu, self.imuCallback)

        self.odom_pub_ = rospy.Publisher("bumblebee_controller/odom_kalman", Odometry, queue_size=10)

        self.mean_ = 0.0
        self.variance_ = 1000.0
        self.imu_angular_z_ = 0.0
        self.is_first_odometry = True
        self.last_angular_z_ = 0.0
        self.motion_ = 0.0
        self.kalman_odom_ = Odometry()
        self.motion_variance_ = 4.0
        self.measurement_variance_ = 0.5

    def imuCallback(self, imu):
        self.imu_angular_z_ = imu.angular_velocity.z

    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odometry:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odometry = False
            return
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        self.measurementUpdate()
        self.statePrediction()
        

        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)

        self.last_angular_z_ = odom.twist.twist.angular.z

    def measurementUpdate(self):
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) / (self.measurement_variance_ + self.variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_)/(self.measurement_variance_ + self.variance_)

    def statePrediction(self):
        self.mean_ = (self.mean_ + self.motion_)
        self.variance_ = self.variance_ + self.motion_variance_