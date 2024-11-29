# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry as Odom
import time
import numpy as np
import math
from utmrbc_msgs.msg import Odometry as RbcOdometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class Odom_Publisher(Node):

    def __init__(self):
        super().__init__('Odom_publisher')
        timer_period = 0.007
        self.Odom_msg = Odom()
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
        qos_profile = QoSProfile(

            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.publisher_ = self.create_publisher(Odom, '/Odom', qos_profile)
        self.create_subscription(RbcOdometry, '/ekf_input', self.subscriber_callback, qos_profile)

        self.error_over_time = 0.0
        self.prev_x_pos      = 0.0
        self.prev_y_pos      = 0.0
        self.imu_yaw         = 0.0
        self.prev_imu_yaw    = 0.0
        self.imu_yaw_rate    = 0.0
        self.time            = time.time()
        self.delta_time      = 0.0
        self.prev_time       = 0.0


    def subscriber_callback(self, msg):
 
        self.Odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.Odom_msg.header.frame_id = "Odom"
        self.Odom_msg.child_frame_id = "base_link"

        self.time = time.time()

        self.imu_yaw = msg.imu_yaw
        self.Odom_msg.pose.pose.position.x = msg.odom_pose_x
        self.Odom_msg.pose.pose.position.y = msg.odom_pose_y
        self.Odom_msg.pose.pose.position.z = 0.0

        self.Odom_msg.pose.covariance = [0.01 + self.error_over_time, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.01 + self.error_over_time, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.01,]
        
        self.delta_time = self.time - self.prev_time

        self.delta_x_pos = self.Odom_msg.pose.pose.position.x- self.prev_x_pos
        self.Odom_msg.twist.twist.linear.x = self.delta_x_pos/self.delta_time 
        self.delta_y_pos = self.Odom_msg.pose.pose.position.y - self.prev_y_pos
        self.Odom_msg.twist.twist.linear.y = self.delta_y_pos/self.delta_time 
        self.imu_yaw_rate = self.imu_yaw - self.prev_imu_yaw
        self.Odom_msg.twist.twist.linear.z = self.imu_yaw_rate/self.delta_time 

        self.Odom_msg.twist.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 0.001,]

        self.Odom_msg.pose.pose.orientation.x = 0.0
        self.Odom_msg.pose.pose.orientation.y = 0.0
        self.Odom_msg.pose.pose.orientation.z = math.sin(math.radians(self.imu_yaw)/2)
        self.Odom_msg.pose.pose.orientation.w = math.cos(math.radians(self.imu_yaw)/2)

        self.Odom_msg.twist.twist.angular.x = 0.0
        self.Odom_msg.twist.twist.angular.y = 0.0
        self.Odom_msg.twist.twist.angular.z = self.imu_yaw_rate

        self.prev_x_pos = self.Odom_msg.pose.pose.position.x
        self.prev_y_pos = self.Odom_msg.pose.pose.position.x
        self.prev_imu_yaw = self.imu_yaw
        self.prev_time = self.time

        self.get_logger().info('odom_x:"%f" odom_y"%f" imu_yaw"%f"' % (self.Odom_msg.pose.pose.position.x  , self.Odom_msg.pose.pose.position.y , self.imu_yaw))


    def timer_callback(self):
        if(self.Odom_msg.pose.pose.position.x == 0.0 and self.Odom_msg.pose.pose.position.y== 0.0):
            self.error_over_time = 0.0
        else:
            self.error_over_time = self.error_over_time + 0.0001
        self.publisher_.publish(self.Odom_msg)

def main(args=None):

    rclpy.init(args=args)
    Odom_Pub = Odom_Publisher()
    rclpy.spin(Odom_Pub)
    Odom_Pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
