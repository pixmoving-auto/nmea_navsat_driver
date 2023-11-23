# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf_transformations import quaternion_from_euler
from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
from libnmea_navsat_driver import parser

from sensor_msgs.msg import Imu 
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from libnmea_navsat_driver import coor_conv
from ublox_msgs.msg import NavPVT
from autoware_sensing_msgs.msg import GnssInsOrientationStamped


def eulerFromQuaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
    Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

class Ros2NMEADriver(Node):
    def __init__(self):
        super().__init__('nmea_navsat_driver')
        
        # CHC -------------
        self.imu_pub = self.create_publisher(Imu, 'chc/imu', 10)
        self.pub_pitch = self.create_publisher(Float32, 'chc/pitch', 2)
        self.pub_heading = self.create_publisher(Float32, 'chc/heading', 2)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'chc/pose', 10)
        self.ublox_navpvt_pub = self.create_publisher(NavPVT, "chc/navpvt", 10)
        self.pub_orientation = self.create_publisher(GnssInsOrientationStamped, 'chc/autoware_orientation', 2)
        # CHC -------------
        
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = self.declare_parameter('epe_quality0', 1000000).value
        self.default_epe_quality1 = self.declare_parameter('epe_quality1', 4.0).value
        self.default_epe_quality2 = self.declare_parameter('epe_quality2', 0.1).value
        self.default_epe_quality4 = self.declare_parameter('epe_quality4', 0.02).value
        self.default_epe_quality5 = self.declare_parameter('epe_quality5', 4.0).value
        self.default_epe_quality9 = self.declare_parameter('epe_quality9', 3.0).value

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    # Returns True if we successfully did something with the passed in
    # nmea_string
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        if not check_nmea_checksum(nmea_string):
            self.get_logger().warn("Received a sentence with an invalid checksum. " +
                                   "Sentence was: %s" % nmea_string)
            return False

        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = self.get_clock().now().to_msg()

        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        current_time_ref = TimeReference()
        current_time_ref.header.stamp = current_time
        current_time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            current_time_ref.source = self.time_ref_source
        else:
            current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]
            if current_fix.status.status > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not math.isnan(data['utc_time']):
                    current_time_ref.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)
        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']
        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)
        elif 'CHC' in parsed_sentence:
            data = parsed_sentence['CHC']
            float_msg = Float32()
            imu_msg = Imu()
            pose_msg = PoseWithCovarianceStamped()
            try:
                # if self.pub_heading.get_subscription_count() > 0:
                float_msg.data = data["heading"]
                self.pub_heading.publish(float_msg)
                    
                if self.pub_pitch.get_subscription_count() > 0:
                    float_msg.data = data["pitch"]
                    self.pub_pitch.publish(float_msg)
                
                if self.imu_pub.get_subscription_count() > 0:
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = frame_id
                    # orientation
                    heading = math.radians(90.0-data['heading'])
                    pitch = math.radians(data['pitch'])
                    roll = math.radians(data['roll'])
                    [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw
                    # linear_acceleration
                    imu_msg.linear_acceleration.x = data["linear_acceleration_y"] * 9.80665
                    imu_msg.linear_acceleration.y = -data["linear_acceleration_x"]* 9.80665
                    imu_msg.linear_acceleration.z = data["linear_acceleration_z"]* 9.80665
                    # angular_velocity
                    # angular velocity the coordinate of imu is y-front x-right z-up, 
                    # so it has to be converted to right-handed coordinate
                    imu_msg.angular_velocity.x = math.radians(data["angular_velocity_y"])
                    imu_msg.angular_velocity.y =  math.radians(-data["angular_velocity_x"])
                    imu_msg.angular_velocity.z =  math.radians(data["angular_velocity_z"])
                    imu_msg.angular_velocity_covariance[0] = 0.001
                    imu_msg.angular_velocity_covariance[4] = 0.001
                    imu_msg.angular_velocity_covariance[8] = 0.001
                    
                    self.imu_pub.publish(imu_msg)
                    
                if self.pose_pub.get_subscription_count() > 0:
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = frame_id
                    # pose msg
                    # pose.position
                    x, y, z = coor_conv.lla2ecef_simple(data['latitude'], data['longitude'], data['altitude'])
                    pose_msg.pose.pose.position.x = x
                    pose_msg.pose.pose.position.y = y
                    pose_msg.pose.pose.position.z = z
                    # pose.orientation
                    heading = math.radians(90.0-data['heading'])
                    pitch = math.radians(data['pitch'])
                    roll = math.radians(data['roll'])
                    [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                    pose_msg.pose.pose.orientation.x = qx
                    pose_msg.pose.pose.orientation.y = qy
                    pose_msg.pose.pose.orientation.z = qz
                    pose_msg.pose.pose.orientation.w = qw
                    
                    self.pose_pub.publish(pose_msg)
                
                if self.ublox_navpvt_pub.get_subscription_count() > 0:
                    satellite_status = int(data['fix_valid']/10)
                    system_status =  int(data['fix_valid'])%10
                    navpvt_msg = NavPVT()
                    if satellite_status == 3: # 纯惯导模式
                        navpvt_msg.fix_type = NavPVT.FIX_TYPE_DEAD_RECKONING_ONLY
                    elif satellite_status == 2: # 组合导航模式
                        navpvt_msg.fix_type = NavPVT.FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED
                    else :
                        if satellite_status == 0 :
                            navpvt_msg.fix_type = NavPVT.FIX_TYPE_NO_FIX
                        elif satellite_status == 6 :
                            navpvt_msg.fix_type = NavPVT.FIX_TYPE_3D
                        else :
                            navpvt_msg.fix_type = NavPVT.FIX_TYPE_2D
                        
                    navpvt_msg.heading = int(math.degrees(data['heading'])*100000)
                    navpvt_msg.lon = int(data['latitude']* 1e7)
                    navpvt_msg.lat = int(data['longitude']* 1e7)
                    navpvt_msg.height = int(data['altitude'])*1000  # mm
                    navpvt_msg.vel_n = int(data['linear_velocity_north']) * 1000   # mm/s
                    navpvt_msg.vel_e = int(data['linear_velocity_east']) * 1000 
                    navpvt_msg.vel_d = int(data['linear_velocity_z']) * 1000 
                    navpvt_msg.g_speed = int(data['linear_velocity_vehihle']) * 1000 
                    
                    self.ublox_navpvt_pub.publish(navpvt_msg)
                
                if self.pub_orientation.get_subscription_count() > 0:
                    orientation_msg = GnssInsOrientationStamped()
                    orientation_msg.header.stamp = self.get_clock().now().to_msg()
                    orientation_msg.header.frame_id = frame_id
                    
                    # orientation msg of autoware
                    # orientation
                    heading = math.radians(90.0-data['heading'])
                    pitch = math.radians(data['pitch'])
                    roll = math.radians(data['roll'])
                    [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                    orientation_msg.orientation.orientation.x = qx
                    orientation_msg.orientation.orientation.y = qy
                    orientation_msg.orientation.orientation.z = qz
                    orientation_msg.orientation.orientation.w = qw
                    
                    orientation_msg.orientation.rmse_rotation_x = 0.001745329 # 0.1 degree
                    orientation_msg.orientation.rmse_rotation_y = 0.001745329 # 0.1 degree 
                    orientation_msg.orientation.rmse_rotation_z = 0.001745329 # 0.1 degree
                    
                    self.pub_orientation.publish(orientation_msg)
            
            except UnicodeDecodeError as err:
                self.get_logger().warn("UnicodeDecodeError: {0}".format(err))
        else:
            return False
        return True

    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gps').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id
