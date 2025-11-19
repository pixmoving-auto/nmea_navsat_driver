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
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference, Temperature
from geometry_msgs.msg import TwistStamped, QuaternionStamped
from tf_transformations import quaternion_from_euler
from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
from libnmea_navsat_driver import parser

from sensor_msgs.msg import Imu 
from std_msgs.msg import UInt8
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from libnmea_navsat_driver import coor_conv
from ublox_msgs.msg import NavPVT
from autoware_sensing_msgs.msg import GnssInsOrientationStamped


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

        # 优控 ADCU A06 IMU 温度-------------
        self.temperature_pub = self.create_publisher(Temperature, '/adcu/imu/temp', 1)
        
        # CHC -------------
        self.imu_pub = self.create_publisher(Imu, 'chc/imu', 10)
        self.pub_pitch = self.create_publisher(Float32, 'chc/pitch', 2)
        self.pub_heading = self.create_publisher(Float32, 'chc/heading', 2)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'chc/pose', 10)
        self.ublox_navpvt_pub = self.create_publisher(NavPVT, "chc/navpvt", 10)
        self.pub_orientation = self.create_publisher(GnssInsOrientationStamped, '/autoware_orientation', 2)
        self.pub_antenna0 = self.create_publisher(UInt8, 'chc/main_antenna_satellite_count', 2)  # 主天线 1 卫星数
        self.pub_antenna1 = self.create_publisher(UInt8, 'chc/auxiliary_antenna_satellite_count', 2)  # # 副天线 2 卫星数
        # CHC -------------
        
        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'vel', 10)
        self.heading_pub = self.create_publisher(QuaternionStamped, 'heading', 10)
        self.time_ref_pub = self.create_publisher(TimeReference, 'time_reference', 10)

        self.time_ref_source = self.declare_parameter('time_ref_source', 'gps').value
        self.use_RMC = self.declare_parameter('useRMC', False).value
        self.valid_fix = False
        # Frame to use for orientation visualization (anchor on vehicle body by default)
        self.orientation_frame_id = self.declare_parameter('orientation_frame_id', 'base_link').value

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

        # Pose covariance configuration when CHC pose is used and PVT covariance is unavailable
        # Standard deviations (meters for position, degrees for orientation)
        self.pose_cov_position_std_xy = self.declare_parameter('pose_cov_position_std_xy', 0.05).value
        self.pose_cov_position_std_z = self.declare_parameter('pose_cov_position_std_z', 0.10).value
        self.pose_cov_orientation_std_deg = self.declare_parameter('pose_cov_orientation_std_deg', 0.15).value
        # Optionally derive TMPVT pose covariance from HDOP if available
        self.use_tmpvt_hdop_for_covariance = self.declare_parameter('use_tmpvt_hdop_for_covariance', True).value
        self.hdop_xy_scale = self.declare_parameter('hdop_xy_scale', 0.10).value   # meters per HDOP
        self.hdop_z_scale = self.declare_parameter('hdop_z_scale', 0.20).value     # meters per HDOP
        self.min_position_std_xy = self.declare_parameter('min_position_std_xy', 0.02).value
        self.min_position_std_z = self.declare_parameter('min_position_std_z', 0.05).value
        # Quality-based covariance mapping (TMPVT quality -> stds). Enabled by default.
        self.use_quality_based_covariance = self.declare_parameter('use_quality_based_covariance', True).value
        # SPS (quality 1~3)
        self.std_xy_sps = self.declare_parameter('std_xy_sps', 0.08).value
        self.std_z_sps = self.declare_parameter('std_z_sps', 0.15).value
        # DGPS (quality 2 can be treated as SPS above, keep explicit knobs)
        self.std_xy_dgps = self.declare_parameter('std_xy_dgps', 0.06).value
        self.std_z_dgps = self.declare_parameter('std_z_dgps', 0.12).value
        # RTK (quality 4/5)
        self.std_xy_rtk = self.declare_parameter('std_xy_rtk', 0.01).value
        self.std_z_rtk = self.declare_parameter('std_z_rtk', 0.02).value
        # NO FIX (quality 0 or others)
        self.std_xy_no_fix = self.declare_parameter('std_xy_no_fix', 1.0).value
        self.std_z_no_fix = self.declare_parameter('std_z_no_fix', 2.0).value
        # Global scale and caps
        self.pose_cov_global_scale = self.declare_parameter('pose_cov_global_scale', 0.01).value
        self.max_position_std_xy = self.declare_parameter('max_position_std_xy', 0.3).value
        self.max_position_std_z = self.declare_parameter('max_position_std_z', 0.6).value
        # Toggle: publish TMPVT pose in ECEF or not (default off)
        self.enable_tmpvt_pose_ecef = self.declare_parameter('enable_tmpvt_pose_ecef', False).value
        # Toggle: publish TMDRPVA pose in ECEF or not (default on for DR PVA users)
        self.enable_drpva_pose_ecef = self.declare_parameter('enable_drpva_pose_ecef', False).value
        # Toggle: publish CHC pose (default off to avoid topic alternation)
        self.enable_chc_pose = self.declare_parameter('enable_chc_pose', False).value
        # Toggle: publish TMPVT-derived NavSatFix (default off to avoid duplicate fix)
        self.publish_tmpvt_fix = self.declare_parameter('publish_tmpvt_fix', False).value
        # Auto-fallback: start with TMPVT fix until DRPVA becomes available
        self.use_tmpvt_fix_until_drpva = self.declare_parameter('use_tmpvt_fix_until_drpva', True).value
        self._using_tmpvt_fallback = self.use_tmpvt_fix_until_drpva
        self.last_valid_fix_time = None
        # Prefer using GGA HDOP to scale TMPVT pose covariance
        self.use_gga_hdop_for_pose_covariance = self.declare_parameter('use_gga_hdop_for_pose_covariance', True).value
        self.gga_hdop_timeout_sec = self.declare_parameter('gga_hdop_timeout_sec', 2.0).value
        self.last_gga_hdop = float("nan")
        self.last_gga_hdop_time_sec = 0.0
        # Also allow using PVT (TMPVT) HDOP when available
        self.use_pvt_hdop_for_pose_covariance = self.declare_parameter('use_pvt_hdop_for_pose_covariance', False).value
        self.pvt_hdop_timeout_sec = self.declare_parameter('pvt_hdop_timeout_sec', 2.0).value
        self.last_pvt_hdop = float("nan")
        self.last_pvt_hdop_time_sec = 0.0
        # Allow using PVT PDOP as well
        self.use_pvt_pdop_for_pose_covariance = self.declare_parameter('use_pvt_pdop_for_pose_covariance', True).value
        self.prefer_pdop_over_hdop = self.declare_parameter('prefer_pdop_over_hdop', True).value
        self.pvt_pdop_timeout_sec = self.declare_parameter('pvt_pdop_timeout_sec', 2.0).value
        self.last_pvt_pdop = float("nan")
        self.last_pvt_pdop_time_sec = 0.0
        # PDOP scales to convert to meters of std
        self.pdop_xy_scale = self.declare_parameter('pdop_xy_scale', 0.06).value
        self.pdop_z_scale = self.declare_parameter('pdop_z_scale', 0.12).value
        # Smoothing for pose covariance to avoid RViz flicker
        self.pose_covariance_ema_alpha = self.declare_parameter('pose_covariance_ema_alpha', 0.3).value  # 0..1, larger=更平滑慢
        self.pose_covariance_max_ratio_step = self.declare_parameter('pose_covariance_max_ratio_step', 2.0).value  # 单帧最多放大/缩小倍数
        self._smoothed_std_xy = float("nan")
        self._smoothed_std_z = float("nan")
        # HDOP smoothing and hold behavior to reduce flicker on missing/jerky inputs
        self.hdop_ema_alpha = self.declare_parameter('hdop_ema_alpha', 0.4).value   # 0..1, larger=更平滑慢
        self._hdop_ema = float("nan")
        self.use_hold_on_missing_hdop = self.declare_parameter('use_hold_on_missing_hdop', True).value
        self.covariance_hold_timeout_sec = self.declare_parameter('covariance_hold_timeout_sec', 1.0).value
        self._last_cov_update_time_sec = 0.0
        # Debug logging
        self.log_hdop_debug = self.declare_parameter('log_hdop_debug', True).value
        # Quality hysteresis and motion-aware smoothing
        self.quality_hysteresis_frames = self.declare_parameter('quality_hysteresis_frames', 5).value
        self._stable_quality = None
        self._quality_pending = None
        self._quality_pending_count = 0
        self.stationary_speed_threshold = self.declare_parameter('stationary_speed_threshold', 0.05).value  # m/s
        self.stationary_pose_covariance_ema_alpha = self.declare_parameter('stationary_pose_covariance_ema_alpha', 0.95).value
        self.stationary_pose_covariance_max_ratio_step = self.declare_parameter('stationary_pose_covariance_max_ratio_step', 1.02).value
        # Hard caps to keep the sphere small under good conditions
        self.rtk_hdop_cap_threshold = self.declare_parameter('rtk_hdop_cap_threshold', 0.8).value
        self.rtk_position_std_xy_cap = self.declare_parameter('rtk_position_std_xy_cap', 0.01).value
        self.rtk_position_std_z_cap = self.declare_parameter('rtk_position_std_z_cap', 0.02).value
        self.stationary_position_std_xy_cap = self.declare_parameter('stationary_position_std_xy_cap', 0.008).value
        self.stationary_position_std_z_cap = self.declare_parameter('stationary_position_std_z_cap', 0.015).value

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

    # ----------------------------------------------------------------------
    # 内部工具方法
    # ----------------------------------------------------------------------
    def _create_navsat_fix(self, frame_id, stamp):
        """构造 NavSatFix 消息的公共方法，统一设置 Header。"""
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = frame_id
        return fix

    def _create_time_reference(self, frame_id, stamp):
        """构造 TimeReference 消息，自动填充 frame_id 与 source。"""
        time_ref = TimeReference()
        time_ref.header.stamp = stamp
        time_ref.header.frame_id = frame_id
        if self.time_ref_source:
            time_ref.source = self.time_ref_source
        else:
            time_ref.source = frame_id
        return time_ref

    def _publish_twist(self, frame_id, stamp, v_x, v_y, v_z=0.0):
        """发布速度消息的便捷接口。"""
        if self.vel_pub.get_subscription_count() == 0:
            return
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.twist.linear.x = v_x
        msg.twist.linear.y = v_y
        msg.twist.linear.z = v_z
        self.vel_pub.publish(msg)

    def _publish_heading_quaternion(self, frame_id, stamp, heading_deg):
        """根据航向角（度）发布 ENU 坐标系下的四元数航向。"""
        if self.heading_pub.get_subscription_count() == 0:
            return
        heading_msg = QuaternionStamped()
        heading_msg.header.stamp = stamp
        heading_msg.header.frame_id = frame_id
        q = quaternion_from_euler(0, 0, math.radians(90.0 - heading_deg))
        heading_msg.quaternion.x = q[0]
        heading_msg.quaternion.y = q[1]
        heading_msg.quaternion.z = q[2]
        heading_msg.quaternion.w = q[3]
        self.heading_pub.publish(heading_msg)

    def _update_tmpvt_dop_cache(self, data):
        """缓存 PQTMPVT 报文中的 HDOP/PDOP 信息，供协方差估计使用。"""
        try:
            pvt_hdop_val = data.get('hdop', float('nan'))
        except Exception:
            pvt_hdop_val = float('nan')
        if not math.isnan(pvt_hdop_val) and pvt_hdop_val > 0.0:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            self.last_pvt_hdop = pvt_hdop_val
            self.last_pvt_hdop_time_sec = now_sec
            if self.log_hdop_debug:
                self.get_logger().info(f"PVT HDOP updated: {pvt_hdop_val:.3f}")

        try:
            pvt_pdop_val = data.get('pdop', float('nan'))
        except Exception:
            pvt_pdop_val = float('nan')
        if not math.isnan(pvt_pdop_val) and pvt_pdop_val > 0.0:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            self.last_pvt_pdop = pvt_pdop_val
            self.last_pvt_pdop_time_sec = now_sec
            if self.log_hdop_debug:
                self.get_logger().info(f"PVT PDOP updated: {pvt_pdop_val:.3f}")

    # Returns True if we successfully did something with the passed in
    # nmea_string
    # ----------------------------------------------------------------------
    # NMEA 报文主入口
    # ----------------------------------------------------------------------
    def add_sentence(self, nmea_string, frame_id, timestamp=None):
        """校验、解析并分发 NMEA 句子，返回是否成功处理。"""
        if not check_nmea_checksum(nmea_string):
            self.get_logger().warn(
                "Received a sentence with an invalid checksum. Sentence was: %s" % nmea_string
            )
            return False

        parsed_sentence = parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            self.get_logger().debug(
                "Failed to parse NMEA sentence. Sentence was: %s" % nmea_string
            )
            return False

        current_time = timestamp if timestamp else self.get_clock().now().to_msg()

        try:
            if not self.use_RMC and 'TMDRPVA' in parsed_sentence:
                return self._handle_tmdrpva(parsed_sentence['TMDRPVA'], frame_id, current_time)
            if not self.use_RMC and 'VTG' in parsed_sentence:
                return self._handle_vtg(parsed_sentence['VTG'], frame_id, current_time)
            if 'RMC' in parsed_sentence:
                return self._handle_rmc(parsed_sentence['RMC'], frame_id, current_time)
            if 'GST' in parsed_sentence:
                return self._handle_gst(parsed_sentence['GST'])
            if 'HDT' in parsed_sentence:
                return self._handle_hdt(parsed_sentence['HDT'], frame_id, current_time)
            if 'GGA' in parsed_sentence:
                return self._handle_gga(parsed_sentence['GGA'])
            if 'CHC' in parsed_sentence:
                return self._handle_chc(parsed_sentence['CHC'], frame_id)
            if 'TMSENMSG' in parsed_sentence:
                return self._handle_tmsenmsg(parsed_sentence['TMSENMSG'], frame_id)
            if 'TMPVT' in parsed_sentence:
                return self._handle_tmpvt(parsed_sentence['TMPVT'], frame_id)
        except Exception as err:
            self.get_logger().warn(f"处理 NMEA 句子时出现异常: {err}")
            return False

        return False

    # ----------------------------------------------------------------------
    # 各类报文处理函数
    # ----------------------------------------------------------------------
    def _handle_tmdrpva(self, data, frame_id, current_time):
        """处理 PQTMDRPVA 报文：组合导航主定位通道。"""
        current_fix = self._create_navsat_fix(frame_id, current_time)
        current_time_ref = self._create_time_reference(frame_id, current_time)
        current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        quality = data['quality']
        if quality == 0:
            current_fix.status.status = NavSatStatus.STATUS_NO_FIX
            self.valid_fix = False
        elif quality == 1:
            current_fix.status.status = NavSatStatus.STATUS_FIX
            self.valid_fix = True
        elif quality == 2:
            current_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
            self.valid_fix = True
        elif quality in [4, 5]:
            current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
            self.valid_fix = True
        else:
            current_fix.status.status = NavSatStatus.STATUS_FIX
            self.valid_fix = True

        current_fix.status.service = NavSatStatus.SERVICE_GPS
        current_fix.latitude = data['latitude']
        current_fix.longitude = data['longitude']
        current_fix.altitude = data['altitude']
        current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(current_fix)

        drpva_fix_valid = current_fix.status.status != NavSatStatus.STATUS_NO_FIX
        if drpva_fix_valid and self.use_tmpvt_fix_until_drpva and self._using_tmpvt_fallback:
            self._using_tmpvt_fallback = False
            self.get_logger().info("PQTMDRPVA fix detected; switching NavSatFix publisher to DRPVA.")

        if not math.isnan(data['utc_time']):
            import time
            import calendar
            utc_time = data['utc_time']
            hours = int(utc_time / 10000)
            minutes = int((utc_time - hours * 10000) / 100)
            seconds = utc_time - hours * 10000 - minutes * 100
            utc_struct = time.gmtime()
            utc_list = list(utc_struct)
            utc_list[3] = hours
            utc_list[4] = minutes
            utc_list[5] = int(seconds)
            unix_time = calendar.timegm(tuple(utc_list)) + (seconds - int(seconds))

            current_time_ref.time_ref = rclpy.time.Time(seconds=unix_time).to_msg()
            self.last_valid_fix_time = current_time_ref
            self.time_ref_pub.publish(current_time_ref)

        if self.pub_orientation.get_subscription_count() > 0:
            orientation_msg = GnssInsOrientationStamped()
            orientation_msg.header.stamp = self.get_clock().now().to_msg()
            orientation_msg.header.frame_id = self.orientation_frame_id
            heading = math.radians(90.0 - data['heading'])
            pitch = math.radians(data['pitch'])
            roll = math.radians(data['roll'])
            [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
            orientation_msg.orientation.orientation.x = qx
            orientation_msg.orientation.orientation.y = qy
            orientation_msg.orientation.orientation.z = qz
            orientation_msg.orientation.orientation.w = qw
            orientation_msg.orientation.rmse_rotation_x = 0.001745329
            orientation_msg.orientation.rmse_rotation_y = 0.001745329
            orientation_msg.orientation.rmse_rotation_z = 0.001745329
            self.pub_orientation.publish(orientation_msg)

        if self.enable_drpva_pose_ecef and self.pose_pub.get_subscription_count() > 0:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = frame_id
            x, y, z = coor_conv.lla2ecef_simple(data['latitude'], data['longitude'], data['altitude'])
            pose_msg.pose.pose.position.x = x
            pose_msg.pose.pose.position.y = y
            pose_msg.pose.pose.position.z = z

            heading = math.radians(90.0 - data['heading'])
            pitch = math.radians(data['pitch'])
            roll = math.radians(data['roll'])
            [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw

            std_xy = None
            std_z = None
            if self.use_quality_based_covariance and ('quality' in data):
                q_raw = data['quality']
                if self._stable_quality is None:
                    self._stable_quality = q_raw
                    self._quality_pending = None
                    self._quality_pending_count = 0
                elif q_raw != self._stable_quality:
                    if self._quality_pending != q_raw:
                        self._quality_pending = q_raw
                        self._quality_pending_count = 1
                    else:
                        self._quality_pending_count += 1
                    if self._quality_pending_count >= max(1, int(self.quality_hysteresis_frames)):
                        self._stable_quality = q_raw
                        self._quality_pending = None
                        self._quality_pending_count = 0
                else:
                    self._quality_pending = None
                    self._quality_pending_count = 0
                q = self._stable_quality
                if q in [4, 5]:
                    std_xy = self.std_xy_rtk
                    std_z = self.std_z_rtk
                elif q in [2]:
                    std_xy = self.std_xy_dgps
                    std_z = self.std_z_dgps
                elif q in [1, 3]:
                    std_xy = self.std_xy_sps
                    std_z = self.std_z_sps
                else:
                    std_xy = self.std_xy_no_fix
                    std_z = self.std_z_no_fix
            if std_xy is None or std_z is None:
                now_sec = self.get_clock().now().nanoseconds / 1e9
                dop_value = float('nan')
                dop_src = 'NONE'
                is_pdop = False
                if self.prefer_pdop_over_hdop and self.use_pvt_pdop_for_pose_covariance and (not math.isnan(self.last_pvt_pdop)) and (now_sec - self.last_pvt_pdop_time_sec <= self.pvt_pdop_timeout_sec):
                    dop_value = self.last_pvt_pdop
                    dop_src = 'PVT_PDOP'
                    is_pdop = True
                else:
                    if self.use_gga_hdop_for_pose_covariance and (not math.isnan(self.last_gga_hdop)) and (now_sec - self.last_gga_hdop_time_sec <= self.gga_hdop_timeout_sec):
                        dop_value = self.last_gga_hdop
                        dop_src = 'GGA_HDOP'
                    elif self.use_pvt_hdop_for_pose_covariance and (not math.isnan(self.last_pvt_hdop)) and (now_sec - self.last_pvt_hdop_time_sec <= self.pvt_hdop_timeout_sec):
                        dop_value = self.last_pvt_hdop
                        dop_src = 'PVT_HDOP'
                    elif self.use_pvt_pdop_for_pose_covariance and (not math.isnan(self.last_pvt_pdop)) and (now_sec - self.last_pvt_pdop_time_sec <= self.pvt_pdop_timeout_sec):
                        dop_value = self.last_pvt_pdop
                        dop_src = 'PVT_PDOP'
                        is_pdop = True
                if not math.isnan(dop_value) and dop_value > 0.0:
                    if math.isnan(self._hdop_ema) or self.hdop_ema_alpha <= 0.0 or self.hdop_ema_alpha >= 1.0:
                        self._hdop_ema = dop_value
                    else:
                        self._hdop_ema = self.hdop_ema_alpha * self._hdop_ema + (1.0 - self.hdop_ema_alpha) * dop_value
                    dop_ema = self._hdop_ema
                    if self.log_hdop_debug:
                        self.get_logger().info(f"DRPVA Effective DOP: src={dop_src} ema={dop_ema:.3f}")
                    if is_pdop:
                        std_xy = max(self.min_position_std_xy, dop_ema * self.pdop_xy_scale)
                        std_z = max(self.min_position_std_z, dop_ema * self.pdop_z_scale)
                    else:
                        std_xy = max(self.min_position_std_xy, dop_ema * self.hdop_xy_scale)
                        std_z = max(self.min_position_std_z, dop_ema * self.hdop_z_scale)
                else:
                    now_sec = self.get_clock().now().nanoseconds / 1e9
                    if self.use_hold_on_missing_hdop and self._last_cov_update_time_sec > 0.0 and (now_sec - self._last_cov_update_time_sec <= self.covariance_hold_timeout_sec) and (not math.isnan(self._smoothed_std_xy)) and (not math.isnan(self._smoothed_std_z)):
                        std_xy = self._smoothed_std_xy
                        std_z = self._smoothed_std_z
                        if self.log_hdop_debug:
                            self.get_logger().info("DRPVA holding last covariance due to missing/expired HDOP")
                    else:
                        std_xy = self.pose_cov_position_std_xy
                        std_z = self.pose_cov_position_std_z
            std_xy = min(std_xy * self.pose_cov_global_scale, self.max_position_std_xy)
            std_z = min(std_z * self.pose_cov_global_scale, self.max_position_std_z)

            try:
                q_for_cap = self._stable_quality if self._stable_quality is not None else data.get('quality', None)
            except Exception:
                q_for_cap = None

            try:
                vel_n = float(data.get('vel_n', 0.0))
                vel_e = float(data.get('vel_e', 0.0))
            except Exception:
                vel_n = 0.0
                vel_e = 0.0
            ground_speed = math.hypot(vel_n, vel_e)

            cap_hdop = float('nan')
            now_sec_cap = self.get_clock().now().nanoseconds / 1e9
            if self.use_gga_hdop_for_pose_covariance and (not math.isnan(self.last_gga_hdop)) and (now_sec_cap - self.last_gga_hdop_time_sec <= self.gga_hdop_timeout_sec):
                cap_hdop = self.last_gga_hdop
            elif self.use_pvt_hdop_for_pose_covariance and (not math.isnan(self.last_pvt_hdop)) and (now_sec_cap - self.last_pvt_hdop_time_sec <= self.pvt_hdop_timeout_sec):
                cap_hdop = self.last_pvt_hdop
            elif self.use_pvt_pdop_for_pose_covariance and (not math.isnan(self.last_pvt_pdop)) and (now_sec_cap - self.last_pvt_pdop_time_sec <= self.pvt_pdop_timeout_sec):
                cap_hdop = self.last_pvt_pdop

            if (q_for_cap in [4, 5]) and (not math.isnan(cap_hdop)) and (cap_hdop <= self.rtk_hdop_cap_threshold):
                std_xy = min(std_xy, self.rtk_position_std_xy_cap)
                std_z = min(std_z, self.rtk_position_std_z_cap)

            if ground_speed < self.stationary_speed_threshold:
                std_xy = min(std_xy, self.stationary_position_std_xy_cap)
                std_z = min(std_z, self.stationary_position_std_z_cap)

            alpha_used = self.stationary_pose_covariance_ema_alpha if ground_speed < self.stationary_speed_threshold else self.pose_covariance_ema_alpha
            ratio_used = self.stationary_pose_covariance_max_ratio_step if ground_speed < self.stationary_speed_threshold else self.pose_covariance_max_ratio_step
            if not math.isnan(self._smoothed_std_xy) and 0.0 < alpha_used < 1.0:
                max_ratio = max(1.0, ratio_used)
                min_allowed = self._smoothed_std_xy / max_ratio
                max_allowed = self._smoothed_std_xy * max_ratio
                limited_xy = min(max(std_xy, min_allowed), max_allowed)
                self._smoothed_std_xy = alpha_used * self._smoothed_std_xy + (1.0 - alpha_used) * limited_xy
            else:
                self._smoothed_std_xy = std_xy
            if not math.isnan(self._smoothed_std_z) and 0.0 < alpha_used < 1.0:
                max_ratio = max(1.0, ratio_used)
                min_allowed = self._smoothed_std_z / max_ratio
                max_allowed = self._smoothed_std_z * max_ratio
                limited_z = min(max(std_z, min_allowed), max_allowed)
                self._smoothed_std_z = alpha_used * self._smoothed_std_z + (1.0 - alpha_used) * limited_z
            else:
                self._smoothed_std_z = std_z

            std_xy = self._smoothed_std_xy
            std_z = self._smoothed_std_z
            pos_xy_var = std_xy * std_xy
            pos_z_var = std_z * std_z
            ori_std_rad = math.radians(self.pose_cov_orientation_std_deg)
            ori_var = ori_std_rad * ori_std_rad
            self._last_cov_update_time_sec = self.get_clock().now().nanoseconds / 1e9
            pose_msg.pose.covariance[0] = pos_xy_var
            pose_msg.pose.covariance[7] = pos_xy_var
            pose_msg.pose.covariance[14] = pos_z_var
            pose_msg.pose.covariance[21] = ori_var
            pose_msg.pose.covariance[28] = ori_var
            pose_msg.pose.covariance[35] = ori_var
            self.pose_pub.publish(pose_msg)

        return True

    def _handle_vtg(self, data, frame_id, current_time):
        """处理 VTG 报文：输出地速矢量。"""
        if not self.valid_fix:
            return True

        speed = data['speed']
        true_course = data['true_course']
        self._publish_twist(
            frame_id,
            current_time,
            speed * math.sin(true_course),
            speed * math.cos(true_course)
        )
        return True

    def _handle_rmc(self, data, frame_id, current_time):
        """处理 RMC 报文：可选的备用定位与速度输出。"""
        if self.use_RMC:
            current_fix = self._create_navsat_fix(frame_id, current_time)
            current_fix.status.status = NavSatStatus.STATUS_FIX if data['fix_valid'] else NavSatStatus.STATUS_NO_FIX
            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude

            current_fix.latitude = latitude
            current_fix.longitude = longitude
            current_fix.altitude = float('nan')
            current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            self.fix_pub.publish(current_fix)

            if not math.isnan(data['utc_time']):
                time_ref_msg = self._create_time_reference(frame_id, current_time)
                time_ref_msg.time_ref = rclpy.time.Time(seconds=data['utc_time']).to_msg()
                self.time_ref_pub.publish(time_ref_msg)

        if data['fix_valid']:
            speed = data['speed']
            true_course = data['true_course']
            self._publish_twist(
                frame_id,
                current_time,
                speed * math.sin(true_course),
                speed * math.cos(true_course)
            )
        return True

    def _handle_gst(self, data):
        """处理 GST 报文：采用接收机自带的定位精度估计。"""
        self.using_receiver_epe = True
        self.lon_std_dev = data['lon_std_dev']
        self.lat_std_dev = data['lat_std_dev']
        self.alt_std_dev = data['alt_std_dev']
        return True

    def _handle_hdt(self, data, frame_id, current_time):
        """处理 HDT 报文：输出高精度航向角。"""
        if data['heading']:
            self._publish_heading_quaternion(frame_id, current_time, data['heading'])
        return True

    def _handle_gga(self, data):
        """处理 GGA 报文：缓存 HDOP，供后续协方差估计加权。"""
        try:
            hdop_value = data.get('hdop', float('nan'))
        except Exception:
            hdop_value = float('nan')
        if not math.isnan(hdop_value) and hdop_value > 0.0:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            self.last_gga_hdop = hdop_value
            self.last_gga_hdop_time_sec = now_sec
            if self.log_hdop_debug:
                self.get_logger().info(f"GGA HDOP updated: {hdop_value:.3f}")
        return True

    def _handle_chc(self, data, frame_id):
        """处理 CHC 报文：输出组合导航自身的姿态、速度与卫星信息。"""
        float_msg = Float32()
        imu_msg = Imu()
        pose_msg = PoseWithCovarianceStamped()
        antenna0_count_msg = UInt8()
        antenna1_count_msg = UInt8()

        try:
            float_msg.data = math.radians(data["heading"])
            self.pub_heading.publish(float_msg)

            if self.pub_pitch.get_subscription_count() > 0:
                pitch_msg = Float32()
                pitch_msg.data = math.radians(data["pitch"])
                self.pub_pitch.publish(pitch_msg)

            if self.imu_pub.get_subscription_count() > 0:
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = frame_id
                heading = math.radians(90.0 - data['heading'])
                pitch = math.radians(data['pitch'])
                roll = math.radians(data['roll'])
                [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                imu_msg.orientation.x = qx
                imu_msg.orientation.y = qy
                imu_msg.orientation.z = qz
                imu_msg.orientation.w = qw
                imu_msg.linear_acceleration.x = data["linear_acceleration_y"] * 9.80665
                imu_msg.linear_acceleration.y = -data["linear_acceleration_x"] * 9.80665
                imu_msg.linear_acceleration.z = data["linear_acceleration_z"] * 9.80665
                imu_msg.angular_velocity.x = math.radians(data["angular_velocity_y"])
                imu_msg.angular_velocity.y = math.radians(-data["angular_velocity_x"])
                imu_msg.angular_velocity.z = math.radians(data["angular_velocity_z"])
                imu_msg.angular_velocity_covariance[0] = 0.01
                imu_msg.angular_velocity_covariance[4] = 0.01
                imu_msg.angular_velocity_covariance[8] = 0.01
                self.imu_pub.publish(imu_msg)

            if self.enable_chc_pose and self.pose_pub.get_subscription_count() > 0:
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = frame_id
                x, y, z = coor_conv.lla2ecef_simple(data['latitude'], data['longitude'], data['altitude'])
                pose_msg.pose.pose.position.x = x
                pose_msg.pose.pose.position.y = y
                pose_msg.pose.pose.position.z = z
                heading = math.radians(90.0 - data['heading'])
                pitch = math.radians(data['pitch'])
                roll = math.radians(data['roll'])
                [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                pose_msg.pose.pose.orientation.x = qx
                pose_msg.pose.pose.orientation.y = qy
                pose_msg.pose.pose.orientation.z = qz
                pose_msg.pose.pose.orientation.w = qw
                self.pose_pub.publish(pose_msg)

            if self.ublox_navpvt_pub.get_subscription_count() > 0:
                satellite_status = int(data['fix_valid'] / 10)
                system_status = int(data['fix_valid']) % 10
                navpvt_msg = NavPVT()
                if satellite_status == 3:
                    navpvt_msg.fix_type = NavPVT.FIX_TYPE_DEAD_RECKONING_ONLY
                elif satellite_status == 2:
                    navpvt_msg.fix_type = NavPVT.FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED
                else:
                    if satellite_status == 0:
                        navpvt_msg.fix_type = NavPVT.FIX_TYPE_NO_FIX
                    elif satellite_status == 6:
                        navpvt_msg.fix_type = NavPVT.FIX_TYPE_3D
                    else:
                        navpvt_msg.fix_type = NavPVT.FIX_TYPE_2D
                navpvt_msg.flags = system_status
                navpvt_msg.flags2 = data['satellite_mode']
                navpvt_msg.num_sv = data['num_sat_main']
                navpvt_msg.lon = int(data['longitude'] * 1e7)
                navpvt_msg.lat = int(data['latitude'] * 1e7)
                navpvt_msg.height = int(data['altitude'] * 1000)
                navpvt_msg.h_msl = int(data['hmsl'] * 1000)
                navpvt_msg.vel_n = int(data['vel_n'] * 1000)
                navpvt_msg.vel_e = int(data['vel_e'] * 1000)
                navpvt_msg.vel_d = int(data['vel_d'] * 1000)
                navpvt_msg.g_speed = int(data['g_speed'] * 1000)
                navpvt_msg.head_mot = int((90 - data['heading']) * 100000)
                navpvt_msg.s_weight = int(data['winning_weight'] * 100)
                self.ublox_navpvt_pub.publish(navpvt_msg)

            if self.pub_orientation.get_subscription_count() > 0:
                orientation_msg = GnssInsOrientationStamped()
                orientation_msg.header.stamp = self.get_clock().now().to_msg()
                orientation_msg.header.frame_id = self.orientation_frame_id
                heading = math.radians(90.0 - data['heading'])
                pitch = math.radians(data['pitch'])
                roll = math.radians(data['roll'])
                [qx, qy, qz, qw] = get_quaternion_from_euler(roll, pitch, heading)
                orientation_msg.orientation.orientation.x = qx
                orientation_msg.orientation.orientation.y = qy
                orientation_msg.orientation.orientation.z = qz
                orientation_msg.orientation.orientation.w = qw
                orientation_msg.orientation.rmse_rotation_x = 0.001745329
                orientation_msg.orientation.rmse_rotation_y = 0.001745329
                orientation_msg.orientation.rmse_rotation_z = 0.001745329
                self.pub_orientation.publish(orientation_msg)

            antenna0_count_msg.data = data['main_antenna_1_satellite_count']
            antenna1_count_msg.data = data["auxiliary_antenna_2_satellite_count"]
            self.pub_antenna0.publish(antenna0_count_msg)
            self.pub_antenna1.publish(antenna1_count_msg)
        except UnicodeDecodeError as err:
            self.get_logger().warn("UnicodeDecodeError: {0}".format(err))

        return True

    def _handle_tmsenmsg(self, data, frame_id):
        """处理 TMSENMSG 报文：优控 ADCU 的 IMU 和温度数据。"""
        try:
            if self.temperature_pub.get_subscription_count() > 0:
                temperature_msg = Temperature()
                temperature_msg.header.stamp = self.get_clock().now().to_msg()
                temperature_msg.header.frame_id = frame_id
                temperature_msg.temperature = data["temp"]
                self.temperature_pub.publish(temperature_msg)
                
            if self.imu_pub.get_subscription_count() > 0:
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = frame_id

                # linear_acceleration
                imu_msg.linear_acceleration.x = data["linear_acceleration_y"] * 9.80665
                imu_msg.linear_acceleration.y = -data["linear_acceleration_x"]* 9.80665
                imu_msg.linear_acceleration.z = -data["linear_acceleration_z"]* 9.80665
                
                # angular_velocity
                imu_msg.angular_velocity.x = math.radians(data["angular_velocity_y"])
                imu_msg.angular_velocity.y =  math.radians(-data["angular_velocity_x"])
                imu_msg.angular_velocity.z =  math.radians(-data["angular_velocity_z"])
                
                # 设置协方差矩阵 - 根据实际测量精度设置
                # 线性加速度协方差 (m/s²)²
                imu_msg.linear_acceleration_covariance[0] = 0.1  # X轴
                imu_msg.linear_acceleration_covariance[4] = 0.1  # Y轴  
                imu_msg.linear_acceleration_covariance[8] = 0.1  # Z轴
                
                # 角速度协方差 (rad/s)²
                imu_msg.angular_velocity_covariance[0] = 0.01  # X轴
                imu_msg.angular_velocity_covariance[4] = 0.01  # Y轴
                imu_msg.angular_velocity_covariance[8] = 0.01  # Z轴
                
                # 姿态协方差 (rad)²
                imu_msg.orientation_covariance[0] = 0.1   # X轴
                imu_msg.orientation_covariance[4] = 0.1   # Y轴
                imu_msg.orientation_covariance[8] = 0.1   # Z轴

                self.imu_pub.publish(imu_msg)

        except UnicodeDecodeError as err:
            self.get_logger().warn("UnicodeDecodeError: {0}".format(err))

        return True

    def _handle_tmpvt(self, data, frame_id):
        """处理 TMPVT 报文：启动阶段可作为 NavSatFix 后备来源。"""
        try:
            now_msg = self.get_clock().now().to_msg()
            should_use_fallback = self.use_tmpvt_fix_until_drpva and self._using_tmpvt_fallback
            allow_tmpvt_fix = self.publish_tmpvt_fix and not self.use_tmpvt_fix_until_drpva
            should_publish_fix = self.fix_pub.get_subscription_count() > 0 and (should_use_fallback or allow_tmpvt_fix)

            if should_publish_fix:
                current_fix = self._create_navsat_fix(frame_id, now_msg)
                quality = data['quality']
                if quality == 0:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX
                    current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    self.valid_fix = False
                elif quality in [1, 2, 3]:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                    current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                    self.valid_fix = True
                elif quality in [4, 5]:
                    current_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
                    current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                    self.valid_fix = True
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX
                    current_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                    self.valid_fix = False

                current_fix.status.service = NavSatStatus.SERVICE_GPS
                current_fix.latitude = data['lat']
                current_fix.longitude = data['lon']
                current_fix.altitude = data['alt']

                hdop = data['hdop']
                if not math.isnan(hdop) and hdop > 0:
                    position_std = hdop * 2.0
                    current_fix.position_covariance[0] = (position_std * math.cos(math.radians(data['lat']))) ** 2
                    current_fix.position_covariance[4] = position_std ** 2
                    current_fix.position_covariance[8] = (position_std * 2) ** 2

                self.fix_pub.publish(current_fix)

            self._publish_twist(frame_id, now_msg, data['vel_e'], data['vel_n'], data['vel_d'])
            self._publish_heading_quaternion(frame_id, now_msg, data['heading'])

            if self.time_ref_pub.get_subscription_count() > 0:
                time_ref_msg = self._create_time_reference(frame_id, now_msg)
                if not math.isnan(data['tow']):
                    time_ref_msg.time_ref = now_msg
                self.time_ref_pub.publish(time_ref_msg)

            if hasattr(self, 'pub_antenna0') and self.pub_antenna0.get_subscription_count() > 0:
                sat_used = data.get('num_sat_used')
                if sat_used is not None:
                    antenna0_msg = UInt8()
                    antenna0_msg.data = sat_used
                    self.pub_antenna0.publish(antenna0_msg)

            self._update_tmpvt_dop_cache(data)
        except Exception as err:
            self.get_logger().warn("Error processing PQTMPVT: {0}".format(err))
            return False
        return True

    """Helper method for getting the frame_id with the correct TF prefix"""
    def get_frame_id(self):
        frame_id = self.declare_parameter('frame_id', 'gnss').value
        prefix = self.declare_parameter('tf_prefix', '').value
        if len(prefix):
            return '%s/%s' % (prefix, frame_id)
        return frame_id
