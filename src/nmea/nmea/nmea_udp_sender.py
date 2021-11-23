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
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from std_msgs.msg import String, Int32
import socket


class Nmea_Udp_Sender(Node):

    def __init__(self):
        super().__init__('nmea_udp_sender')
        self.subscription = self.create_subscription(
            String,
            'nmea_sentence',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # parameters
        self.declare_parameter('ip', '127.0.0.1',  ParameterDescriptor(description='UDP ip address to send the nmea sentence to'))
        self.declare_parameter('port', 10110,  ParameterDescriptor(description='UDP port to send the nmea sentence to'))

        self.socket = []
        self.open()


    def open(self):
        self.socket = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
        # Allow UDP broadcast
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


    def close(self):
        self.socket.close()


    def listener_callback(self, msg):
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = int(self.get_parameter('port').get_parameter_value().integer_value)
        

        try:
            mess = msg.data
            
            # check for empty message
            if not mess:
                self.get_logger().warn('Empty message received %s' % mess)

            mess = mess.strip()
            mess = mess + u"\r\n"
            self.socket.sendto(mess.encode("utf-8"),(ip, port))
        
        except Exception as msg:
            self.get_logger().fatal('Fatal Error: %s' % vars(msg))
            self.close()
            return



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Nmea_Udp_Sender()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
