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
import pynmea2
from std_msgs.msg import String, Float64, UInt32, Header
from sensor_msgs.msg import NavSatFix, NavSatStatus


class Nmea_Decoder(Node):

    def __init__(self):
        super().__init__('nmea_decoder')
        self.subscription = self.create_subscription(
            String,
            'nmea_sentence',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # define publisher values
        self.publisher_depth = self.create_publisher(Float64, 'depth', 10)
        self.publisher_water_temperature = self.create_publisher(Float64, 'water_temperature', 10)

        self.publisher_apparent_wind_angle = self.create_publisher(Float64, 'apparent_wind_angle', 10)
        self.publisher_apparent_wind_speed = self.create_publisher(Float64, 'apparent_wind_speed', 10)
        self.publisher_apparent_wind_angle_relative = self.create_publisher(Float64, 'apparent_wind_angle_relative', 10)
        self.publisher_apparent_wind_speed_relative = self.create_publisher(Float64, 'apparent_wind_speed_relative', 10)

        self.publisher_true_wind_angle = self.create_publisher(Float64, 'true_wind_angle', 10)
        self.publisher_true_wind_speed = self.create_publisher(Float64, 'true_wind_speed', 10)
        self.publisher_true_wind_angle_relative = self.create_publisher(Float64, 'true_wind_angle_relative', 10)
        self.publisher_true_wind_speed_relative = self.create_publisher(Float64, 'true_wind_speed_relative', 10)

        self.publisher_magnetic_heading = self.create_publisher(Float64, 'magnetic_heading', 10)
        self.publisher_true_heading = self.create_publisher(Float64, 'true_heading', 10)

        self.publisher_speed_through_water = self.create_publisher(Float64, 'speed_through_water', 10)
        self.publisher_speed_over_ground = self.create_publisher(Float64, 'speed_over_ground', 10)
        self.publisher_velocity_made_good = self.create_publisher(Float64, 'velocity_made_good', 10)

        self.publisher_logge = self.create_publisher(Float64, 'logge', 10)
        self.publisher_logge_after_reset = self.create_publisher(Float64, 'logge_after_reset', 10)
        self.publisher_position = self.create_publisher(NavSatFix, 'position', 10)
        self.publisher_position_gll = self.create_publisher(NavSatFix, 'position_gll', 10)

        # parameters
        self.declare_parameter('depth_transducer_offset',
                               0.0,
                               ParameterDescriptor(description='Offset of depth transducer to water surface in meter'))
        self.declare_parameter('talker_filter',
                               '*',
                               ParameterDescriptor(description='Filter for talker device publishing a nmea '
                                                               'sentence ("*" means filter deactivated)'))

    def listener_callback(self, msg):
        """
        listener to process received nmea sentence string.

        :param msg: sentence data in pynmea format
        """
        # Decode NMEA sentence
        try:
            # decode nmea message string
            nmea_sentence = pynmea2.parse(msg.data)

            # apply talker filter
            talker_filter = self.get_parameter('talker_filter').get_parameter_value().string_value
            if nmea_sentence.talker == talker_filter or talker_filter == '*':

                # parse by sending it to specialized message decoder function
                if nmea_sentence.sentence_type == 'DBT':    # DBT - Depth below transducer
                    self.publish_dbt(nmea_sentence)
                elif nmea_sentence.sentence_type == 'DPT':  # DPT - Depth of water
                    self.get_logger().debug('Depth: %f' % float(nmea_sentence.data[0]))
                    f = Float64()
                    f.data = float(nmea_sentence.data[0])
                    self.publisher_depth.publish(f)
                elif nmea_sentence.sentence_type == 'GGA':  # GGA - Global Positioning System Fix Data, Time, Position and fix related data fora GPS receiver.
                    self.publish_gga(nmea_sentence)
                elif nmea_sentence.sentence_type == 'GLL':  # GLL - Geographic Position - Latitude/Longitude
                    self.publish_gll(nmea_sentence)
                elif nmea_sentence.sentence_type == 'HDM':  # HDM - Heading - Magnetic
                    self.publish_hdm(nmea_sentence)
                elif nmea_sentence.sentence_type == 'MTW':  # MTW - Water Temperature
                    self.publish_mtw(nmea_sentence)
                elif nmea_sentence.sentence_type == 'MWD':  # MWD - Wind Direction and Speed
                    self.publish_mwd(nmea_sentence)
                elif nmea_sentence.sentence_type == 'VHW':  # VHW - Water speed and heading
                    self.publish_vhw(nmea_sentence)
                elif nmea_sentence.sentence_type == 'VLW':  # VLW - Distance Traveled through Water
                    self.publish_vlw(nmea_sentence)
                elif nmea_sentence.sentence_type == 'VPW':  # VPW - Speed - Measured Parallel to Wind
                    self.publish_vpw(nmea_sentence)
                elif nmea_sentence.sentence_type == 'VWR':  # VWR - Apparent wind relative bearing and velocity
                    self.publish_vwr(nmea_sentence)
                elif nmea_sentence.sentence_type == 'VWT':  # VWT - True wind relative bearing and velocity
                    self.publish_vwt(nmea_sentence)
                else:
                    # unknown message type
                    self.get_logger().info('Type: %s' % vars(nmea_sentence))
            else:
                self.get_logger().debug('Talker ID (%s) unknown' % nmea_sentence.talker)

        except pynmea2.nmea.ChecksumError as error:
            self.get_logger().warn("Failure Checksum %s" % msg.data)
        except pynmea2.nmea.ParseError as error:
            self.get_logger().debug("Failure Parsing %s" % msg.data)

    def publish_dbt(self, sentence):
        """
        Publishes depth value based on DBT sentence (depth below transducer). 
        Configure offset of the transducer (depth_transducer_offset) in added to the received depth value

        :param sentence: sentence data in pynmea format
        """ 
        depth = Float64()
        offset = float(self.get_parameter('depth_transducer_offset').get_parameter_value().double_value)
    
        # check for depth in meter
        if sentence.data[2] and sentence.data[3]=='M':
            depth.data = offset + float(sentence.data[2])

        # check for depth in feet            
        elif sentence.data[0] and sentence.data[1]=='f':
            depth.data = offset + float(sentence.data[0]) * 0.3048

        # check for depth in Fathoms
        elif sentence.data[4] and sentence.data[5]=='F':
            depth.data = offset + float(sentence.data[4]) * 1.8288

        else:
            self.get_logger().warn('Cannot decode DBT sentence: %s' % vars(sentence))
            return

        self.get_logger().debug('Depth: %f' % depth.data)
        self.publisher_depth.publish(depth)

    def publish_mtw(self, sentence):
        """
        Publishes water temperature value in degree celcius

        :param sentence: sentence data in pynmea format
        """ 
        temp = Float64()
    
        # check for temperature value
        if sentence.data[0] and sentence.data[1]=='C':
            temp.data = float(sentence.data[0])

        else:
            self.get_logger().warn('Cannot decode MTW sentence: %s' % vars(sentence))
            return

        self.get_logger().debug('Water Temperature: %f' % temp.data)
        self.publisher_water_temperature.publish(temp)

    def publish_hdm(self, sentence):
        """
        Publishes magnetic heading in degree

        :param sentence: sentence data in pynmea format
        """ 
        heading = Float64()
    
        # check for heading value
        if sentence.data[0] and sentence.data[1]=='M':
            heading.data = float(sentence.data[0])

        else:
            self.get_logger().warn('Cannot decode HDM sentence: %s' % vars(sentence))
            return

        self.get_logger().debug('Magnetic Heading: %f' % heading.data)
        self.publisher_magnetic_heading.publish(heading)
            
    def publish_vpw(self, sentence):
        """
        Publishes velocity made good based on VPW dentence

        :param sentence: sentence data in pynmea format
        """ 
        velocity = Float64()
    
        # check for speed value
        if sentence.data[0] and sentence.data[1]=='N':
            velocity.data = float(sentence.data[0])
        else:
            self.get_logger().warn('Cannot decode VPW sentence: %s' % vars(sentence))
            return

        self.get_logger().debug('Velocity made good: %f' % velocity.data)
        self.publisher_velocity_made_good.publish(velocity)

    def publish_mwd(self, sentence):
        """
        Publishes true wind angle and speed based on MWD sentence (true Wind Speed and Angle).
        
        :param sentence: sentence data in pynmea format
        """ 
        speed = Float64()
        angle = Float64()
        
        if sentence.data[2] and (sentence.data[3]=='M'):
            angle.data = -float(sentence.data[2])
        else:
            self.get_logger().warn('Cannot decode MWD sentence on angle: %s' % vars(sentence))
            return

        # decode speed in knots
        if sentence.data[4] and sentence.data[5]=='N':
            speed.data = float(sentence.data[4])

        # decode speed in meter per second
        elif sentence.data[5] and sentence.data[6]=='M':
            speed.data = float(sentence.data[5]) * 1.94384

        else:
            self.get_logger().warn('Cannot decode MWD sentence on speed: %s' % vars(sentence))
            return

        # publish true wind angle and speed
        self.get_logger().debug('True Wind Direction / Speed: %f / %f' % (angle.data, speed.data))
        self.publisher_true_wind_angle.publish(angle)
        self.publisher_true_wind_speed.publish(speed)

    def publish_vhw(self, sentence):
        """
        Publishes heading (true and magnetic) and speed through watter based on VHW sentence (Water speed and heading ).
        
        :param sentence: sentence data in pynmea format
        """ 

        speed = Float64()
        true_heading = Float64()
        magnetic_heading = Float64()

        # decode true heading if available
        if sentence.data[0] and sentence.data[1]=='T':
            true_heading.data = float(sentence.data[0])
            self.publisher_true_heading.publish(true_heading)

        # decode magnetic heading
        if sentence.data[2] and (sentence.data[3]=='M'):
            magnetic_heading.data = float(sentence.data[2])
            self.publisher_magnetic_heading.publish(magnetic_heading)
        else:
            self.get_logger().warn('Cannot decode VHW sentence on magnetic heading: %s' % vars(sentence))

        # decode speed through water in knots
        if sentence.data[4] and sentence.data[5]=='N':
            speed.data = float(sentence.data[4])

        # decode speed through water in kilometer per second
        elif sentence.data[6] and sentence.data[7] == 'K':
            speed.data = float(sentence.data[6]) * 0.539957

        else:
            self.get_logger().warn('Cannot decode VHW sentence on speed through water: %s' % vars(sentence))
            return

        # publish speed through water
        self.get_logger().debug('True Heading / Magnetic Heading / Speed: %f / %f / %f' %
                                (true_heading.data, magnetic_heading.data, speed.data))
        self.publisher_speed_through_water.publish(speed)

    def publish_vlw(self, sentence):
        """
        Publishes logge and logge after reset based on VLW sentence

        :param sentence: sentence data in pynmea format
        """
        logge = Float64()
        logge_reset = Float64()

        # decode for logge in miles
        if sentence.data[0] and (sentence.data[1] == 'N'):
            logge.data = -float(sentence.data[0])
        else:
            self.get_logger().warn('Cannot decode VLW sentence on logge: %s' % vars(sentence))
            return

        # decode for logge after reset in miles
        if sentence.data[2] and sentence.data[3] == 'N':
            logge_reset.data = float(sentence.data[2])
        else:
            self.get_logger().warn('Cannot decode VLW sentence on logge after reset: %s' % vars(sentence))
            return

        # publish logge and logge after reset
        self.get_logger().debug('Logge / Logge after reset: %f / %f' % (logge.data, logge_reset.data))
        self.publisher_logge.publish(logge)
        self.publisher_logge_after_reset.publish(logge_reset)

    def publish_vwr(self, sentence):
        """
        Publishes relative wind angle and speed based on VWR sentence (Relative Wind Speed and Angle).

        :param sentence: sentence data in pynmea format
        """
        speed = Float64()
        angle = Float64()

        # decode apparent relative wind direction
        if sentence.data[0] and (sentence.data[1] == 'R' or sentence.data[1] == 'L'):
            if sentence.data[1] == 'R':
                angle.data = float(sentence.data[0])
            else:
                angle.data = -float(sentence.data[0])
        else:
            self.get_logger().warn('Cannot decode VWR sentence on direction: %s' % vars(sentence))
            return

        # decode velocity in knots
        if sentence.data[2] and sentence.data[3] == 'N':
            speed.data = float(sentence.data[2])

        # decode velocity in meter per second
        elif sentence.data[4] and sentence.data[5] == 'M':
            speed.data = float(sentence.data[4]) * 1.94384

        # decode velocity in kilomeer per second
        elif sentence.data[6] and sentence.data[7] == 'K':
            speed.data = float(sentence.data[6]) * 0.539957

        else:
            self.get_logger().warn('Cannot decode VWR sentence on speed: %s' % vars(sentence))
            return

        # publish apparent relative wind direction and velocity
        self.get_logger().debug('Relative Wind Direction / Speed: %f / %f' % (angle.data, speed.data))
        self.publisher_apparent_wind_angle_relative.publish(angle)
        self.publisher_apparent_wind_speed_relative.publish(speed)

    def publish_vwt(self, sentence):
        """"
        Publishes true wind angle and speed based on VWT sentence (True wind relative bearing and velocity).

        :param sentence: sentence data in pynmea format
        """
        speed = Float64()
        angle = Float64()

        # decode true wind direction
        if sentence.data[0] and (sentence.data[1]=='R' or sentence.data[1]=='L'):
            if sentence.data[1]=='R':
                angle.data = float(sentence.data[0])
            else:
                angle.data = -float(sentence.data[0])
        else:
            self.get_logger().warn('Cannot decode VWT sentence on direction: %s' % vars(sentence))
            return

        # decode velocity in knots
        if sentence.data[2] and sentence.data[3]=='N':
            speed.data = float(sentence.data[2])

        # decode velocity in meter per second
        elif sentence.data[4] and sentence.data[5]=='M':
            speed.data = float(sentence.data[4]) * 1.94384

        # decode velocity in kilometer per second
        elif sentence.data[6] and sentence.data[7]=='K':
            speed.data = float(sentence.data[6]) * 0.539957

        else:
            self.get_logger().warn('Cannot decode VWR sentence on velocity: %s' % vars(sentence))
            return

        # publish true wind direction and velocity
        self.get_logger().debug('True Wind Direction / Speed: %f / %f' % (angle.data, speed.data))
        self.publisher_true_wind_angle.publish(angle)
        self.publisher_true_wind_speed.publish(speed)

    def publish_gga(self, sentence):
        """
        Publishes fix gps position based on GGA sentence 
        
        :param sentence: sentence data in pynmea format
        """ 
        gps = NavSatFix()
        gps.header = Header()
        gps.header.frame_id = "gps"
        gps.status.service = NavSatStatus.SERVICE_GPS

        # decode time stamp
#        if sentence.data[0]:
#            gps.header.stamp.secs = 0 #int(sentence.data[0])
#            gps.header.stamp.nsecs = 0
#        else:
#            self.get_logger().warn('Cannot decode GGA sentence on time stamp: %s' % vars(sentence))
#            return

        # decode latitude
        if sentence.data[1] and (sentence.data[2]=='N' or sentence.data[2]=='S'):
            gps.latitude = float(sentence.data[1])/100
            if sentence.data[2]=='S':
                gps.latitude = -gps.latitude
        else:
            self.get_logger().warn('Cannot decode GGA sentence on latitude: %s' % vars(sentence))
            return

        # decode longitude
        if sentence.data[3] and (sentence.data[4]=='E' or sentence.data[4]=='W'):
            gps.longitude = float(sentence.data[3])/100
            if sentence.data[4]=='W':
                gps.longitude = -gps.longitude
        else:
            self.get_logger().warn('Cannot decode GGA sentence on longitude: %s' % vars(sentence))
            return

        # decode altitude
        if sentence.data[8] and sentence.data[9]=='M':
            gps.altitude = float(sentence.data[8])
        else:
            self.get_logger().warn('Cannot decode GGA sentence on altitude: %s' % vars(sentence))
            return
       
        # decode GPS quality indicator
        if sentence.data[5]:
            if sentence.data[5]==1:
                gps.status.status = NavSatStatus.STATUS_FIX
            else:
                gps.status.status = NavSatStatus.STATUS_NO_FIX
        else:
            self.get_logger().warn('Cannot decode GGA sentence on quality indicator: %s' % vars(sentence))
            return

        self.publisher_position.publish(gps)

    def publish_gll(self, sentence):
        """
        Publishes fix gps position based on GLL sentence 
        
        :param sentence: sentence data in pynmea format
        """ 
        gps = NavSatFix()
        gps.header = Header()
        gps.header.frame_id = "gps"
        gps.status.service = NavSatStatus.SERVICE_GPS

        # decode time stamp
#        if sentence.data[0]:
#            gps.header.stamp.secs = 0 #int(sentence.data[0])
#            gps.header.stamp.nsecs = 0
#        else:
#            self.get_logger().warn('Cannot decode GGA sentence on time stamp: %s' % vars(sentence))
#            return

        # decode latitude
        if sentence.data[0] and (sentence.data[1]=='N' or sentence.data[1]=='S'):
            gps.latitude = float(sentence.data[0])/100
            if sentence.data[1]=='S':
                gps.latitude = -gps.latitude
        else:
            self.get_logger().warn('Cannot decode GLL sentence on latitude: %s' % vars(sentence))
            return

        # decode longitude
        if sentence.data[2] and (sentence.data[3]=='E' or sentence.data[3]=='W'):
            gps.longitude = float(sentence.data[2])/100
            if sentence.data[3]=='W':
                gps.longitude = -gps.longitude
        else:
            self.get_logger().warn('Cannot decode GLL sentence on longitude: %s' % vars(sentence))
            return

        # decode altitude
        gps.altitude = 0.0
               
        # decode GPS quality indicator
        gps.status.status = NavSatStatus.STATUS_FIX
        
        self.publisher_position_gll.publish(gps)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = Nmea_Decoder()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
