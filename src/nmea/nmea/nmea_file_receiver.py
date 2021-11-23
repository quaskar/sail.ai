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
from std_msgs.msg import String


class NmeaFilePublisher(Node):

    def __init__(self):
        super().__init__('nmea_file_publisher')
        self.publisher_ = self.create_publisher(String, 'nmea_sentence', 10)
        

        # parameters
        self.declare_parameter('filename', './data/nmea_datalog.txt',  ParameterDescriptor(description='Log file of a nmea trace to replay'))
        self.declare_parameter('repeat', False,  ParameterDescriptor(description='Repeat from beginning if trace file end is reached'))
        self.declare_parameter('timelaps', 0.1,  ParameterDescriptor(description='Time laps between two nmea sentences to publish'))

        self.f = []
        self.open()

        timer_period = self.get_parameter('timelaps').get_parameter_value().double_value  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def open(self):
        filename = self.get_parameter('filename').get_parameter_value().string_value
        self.f = open(filename, 'r')

    def close(self):
        self.f.close()

    def timer_callback(self):
        msg = String()
        repeat = self.get_parameter('repeat').get_parameter_value().bool_value

        line = self.f.readline()

        # check for end of line
        if (line == ""):
            if repeat:
                self.f.seek(0)
                line = self.f.readline()
            else:
                self.close()
                #ToDo: stop timer properly
                return

        

        line = line.strip()
        if line == "":
            return 

        self.get_logger().debug('Publishing: "%s"' % line)

        # prepare and publish nmea sentence
        msg.data = line + u"\r\n"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = NmeaFilePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
