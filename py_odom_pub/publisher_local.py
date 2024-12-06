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
from utmrbc_msgs.msg import Local as local
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

class Local_Publisher(Node):

    def __init__(self):
        super().__init__('Local_publisher')
        timer_period = 0.007
        self.Odom_msg = local()
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
        qos_profile = QoSProfile(

            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.publisher_ = self.create_publisher(local, '/final_pose', qos_profile)
        self.create_subscription(Odom, '/odometry/filtered', self.subscriber_callback, qos_profile)

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
 
        self.Odom_msg.local_x = msg.pose.pose.position.x
        self.Odom_msg.local_y = msg.pose.pose.position.y
        self.time = time.time()
        # self.get_logger().info('local_x:"%f" local_y"%f"' % (self.Odom_msg.local_x  , self.Odom_msg.local_y))


    def timer_callback(self):

        self.publisher_.publish(self.Odom_msg)

def main(args=None):

    rclpy.init(args=args)
    Local_Pub = Local_Publisher()
    rclpy.spin(Local_Pub)
    Local_Pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
