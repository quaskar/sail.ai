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
import socket
from std_msgs.msg import String


class NmeaUpdPublisher(Node):

    def __init__(self):
        super().__init__('nmea_udp_publisher')
        self.publisher_ = self.create_publisher(String, 'nmea_sentence', 10)
        self.NmeaSocket = []
        self.open()       

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def open(self):
        self.NmeaSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.NmeaSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.NmeaSocket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.NmeaSocket.bind(('', 10110))

    def timer_callback(self):
        message, address = self.NmeaSocket.recvfrom(8192)
        msg = String()

        message = message.decode("ascii")
        message = '$' + message.split('$')[-1]

        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % message)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = NmeaUpdPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
