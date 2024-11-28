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
from nav_msgs.msg import Odometry
import time
import numpy as np
from utmrbc_msgs.msg import Odometry as RbcOdometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from pymodbus.client import ModbusTcpClient


class Laser_Publisher(Node):

    def __init__(self):
        super().__init__('Laser_publisher')
        timer_period = 0.02
        self.Odom_msg = Odometry()
        qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.publisher_ = self.create_publisher(Odometry, '/Odom_laser', qos_profile)
        self.client = ModbusTcpClient('192.168.1.250', port=502, timeout=1)
        self.value = None
        self.laser_x = 0.0
        self.laser_y = 0.0
        self.prev_laser_x = 0.0
        self.prev_laser_y = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.delta_pos_x = 0.0
        self.delta_pos_y = 0.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = time.time()
        self.prev_time = 0.0
        self.delta_time = 0.0



    def timer_callback(self):

        if self.client.connect():
            try: 
                # ur welcome :)
                self.value = self.client.read_holding_registers(200, 9)
                self.laser_x = self.value.registers[0]/1000
                self.laser_y = self.value.registers[8]/1000
                self.get_logger().info('laser_x:"%f" laser_y"%f"' % (self.pos_x , self.pos_y))
                # self.get_logger().info(f"X: {self.laser_x}, Y: {self.laser_y}")
                # else:
                #     logging.error("Failed")
            except Exception as e:
                # time.sleep(1)
                self.get_logger().info(f"Exception during Modbus read: {e}")

        self.time = time.time()
        self.delta_time = self.time - self.prev_time

        self.delta_pos_x = self.laser_x - self.prev_laser_x
        self.delta_pos_y = self.laser_y - self.prev_laser_y

        self.pos_x = self.pos_x + self.delta_pos_x
        self.pos_y = self.pos_y + self.delta_pos_y

        self.prev_laser_x = self.laser_x
        self.prev_laser_y = self.laser_y

        self.Odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.Odom_msg.header.frame_id = "Odom"
        self.Odom_msg.child_frame_id = "base_link"

        self.Odom_msg.pose.pose.position.x = self.pos_x
        self.Odom_msg.pose.pose.position.y = self.pos_y
        self.Odom_msg.pose.pose.position.z = 0.0

        self.Odom_msg.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
                                         0.0, 0.0, 0.0, 0.0, 0.0, -1.0,]

        self.Odom_msg.twist.twist.linear.x = self.delta_pos_x/self.delta_time
        self.Odom_msg.twist.twist.linear.y = self.delta_pos_y/self.delta_time
        self.Odom_msg.twist.twist.linear.z = 0.0

        self.Odom_msg.twist.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, -1.0,]

        self.Odom_msg.pose.pose.orientation.x = 0.0
        self.Odom_msg.pose.pose.orientation.y = 0.0
        self.Odom_msg.pose.pose.orientation.z = 0.0
        self.Odom_msg.pose.pose.orientation.w = 1.0

        self.Odom_msg.twist.twist.angular.x = 0.0
        self.Odom_msg.twist.twist.angular.y = 0.0
        self.Odom_msg.twist.twist.angular.z = 0.0

        self.prev_time = self.time

        self.publisher_.publish(self.Odom_msg)

def main(args=None):

    rclpy.init(args=args)
    Laser_Pub = Laser_Publisher()
    rclpy.spin(Laser_Pub)
    Laser_Pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
