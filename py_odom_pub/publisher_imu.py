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
from sensor_msgs.msg import Imu
from utmrbc_msgs.msg import Odometry as RbcOdometry
import numpy as np
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class Imu_Publisher(Node):

    def __init__(self):
        super().__init__('Imu_publisher')
        self.Imu_msg = Imu()
        timer_period = 0.007
        self.timer = self.create_timer(timer_period, self.timer_callback)
        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.publisher = self.create_publisher(Imu, '/Imu', qos_profile)
        self.create_subscription(RbcOdometry, '/ekf_input', self.subscriber_callback, qos_profile)

    def subscriber_callback(self, msg):
        # msg = RbcOdometry()
        self.Imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.Imu_msg.header.frame_id = "base_link"

        self.Imu_msg.angular_velocity.x = msg.imu_angular_vx
        self.Imu_msg.angular_velocity.y = msg.imu_angular_vy
        self.Imu_msg.angular_velocity.z = msg.imu_angular_vz

        self.Imu_msg.angular_velocity_covariance = np.array([0.001, 0.0, 0.0,
                                                             0.0, 0.001, 0.0,
                                                             0.0, 0.0, 0.001], dtype=np.float64)
        
        self.Imu_msg.linear_acceleration.x =  msg.imu_linear_ax
        self.Imu_msg.linear_acceleration.y =  msg.imu_linear_ay
        self.Imu_msg.linear_acceleration.z =  9.81

        self.Imu_msg.linear_acceleration_covariance = np.array([0.001, 0.0, 0.0,
                                                                0.0, 0.001, 0.0,
                                                                0.0, 0.0, 0.001], dtype=np.float64)

        self.Imu_msg.orientation.x =  msg.imu_orientation_x
        self.Imu_msg.orientation.y =  msg.imu_orientation_y
        self.Imu_msg.orientation.z =  msg.imu_orientation_z
        self.Imu_msg.orientation.w =  msg.imu_orientation_w

        self.Imu_msg._orientation_covariance = np.array([0.001, 0.0, 0.0,
                                                         0.0, 0.001, 0.0,
                                                         0.0, 0.0, 0.001], dtype=np.float64)

        self.get_logger().info('acc_x:"%f" acc_y:"%f" yaw:"%f"' % (self.Imu_msg.linear_acceleration.x , self.Imu_msg.linear_acceleration.y, msg.imu_yaw)) 
 
    def timer_callback(self):
        self.publisher.publish(self.Imu_msg)

def main(args=None):

    rclpy.init(args=args)
    Imu_Pub = Imu_Publisher()
    rclpy.spin(Imu_Pub)
    Imu_Pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
