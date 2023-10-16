#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
import tf_conversions
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
 
class SimpleController(object):

    def __init__(self, wheel_radius, wheel_separation):
        rospy.loginfo("using wheel radius: %d", wheel_radius)
        rospy.loginfo("using wheel separation: %d", wheel_separation)

        self.wheel_radius_ = wheel_radius
        self.wheel_separation_ = wheel_separation
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = rospy.Time.now()
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.odom_pub_ = rospy.Publisher("bumperbot_controller/odom", Odometry, queue_size = 10)
        self.right_cmd_pub_ = rospy.Publisher("/wheel_right_controller/command", Float64, queue_size=10)
        self.left_cmd_pub_ = rospy.Publisher("/wheel_left_controller/command", Float64, queue_size=10)

        self.vel_sub_ = rospy.Subscriber("/bumblebee_controller/cmd_vel", Twist, self.velCallback)
        self.joint_sub_ = rospy.Subscriber("joint_states", JointState, self.jointCallback)

        self.speed_conversion_ = np.array([[wheel_radius/2, wheel_radius/2], [wheel_radius/wheel_separation, -wheel_radius/wheel_separation]])

        rospy.loginfo("The conversion matrix is: %s", self.speed_conversion_)

    def velCallback(self, msg):
        robot_speed = np.array([[msg.linear.x],
                               [msg.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)

        right_speed = Float64(wheel_speed[0,0])
        left_speed = Float64(wheel_speed[1,0])

        self.left_cmd_pub_.publish(left_speed)
        self.right_cmd_pub_.publish(right_speed)

    def jointCallback(self, msg):
        dp_left = msg.position[0] - self.left_wheel_prev_pos_ ## dp - delta phi
        dp_right = msg.position[1] - self.right_wheel_prev_pos_
        
        dt = (msg.header.stamp - self.prev_time_).to_sec()

        fi_left = dp_left / dt
        fi_right = dp_right / dt

        linear = (self.wheel_radius_ * fi_left + self.wheel_radius_ * fi_right) / 2
        angular  = (self.wheel_radius_ * fi_left - self.wheel_radius_ * fi_right) / self.wheel_separation_

        self.left_wheel_prev_pos_ = msg.position[0]
        self.right_wheel_prev_pos_ = msg.position[1]
        self.prev_time_ = msg.header.stamp

        ds = (self.wheel_radius_ * dp_left + self.wheel_radius_ *  dp_right) / 2
        dtheta = (self.wheel_radius_ * dp_left - self.wheel_radius_ * dp_right) / self.wheel_separation_

        self.theta_ += dtheta
        self.x_ += ds * math.cos(self.theta_)
        self.y_ += ds * math.sin(self.theta_)

        q = tf_conversions.transformations.quaternion_from_euler(0,0, self.theta_)
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = rospy.Time.now()

        self.br_.sendTransform(self.transform_stamped_)

