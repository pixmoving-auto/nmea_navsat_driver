nmea_navsat_driver
===============

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver

IMU source priority
-------------------

The shared ``imu`` topic selects the highest-priority live source:
``gpchc > rawimub > tmsenmsg``. All IMU sources are assumed to run at 100 Hz (10 ms per cycle).
The ``imu_source_missed_cycles`` ROS parameter (default: 5, positive integer)
sets the expiry threshold to N * 10 ms, i.e. 50 ms by default since the last
received frame. At or above this threshold the source is expired. This replaces
the previous ``imu_source_timeout`` parameter. Expiry uses monotonic reception
time, independently of message timestamps and ROS time.
The first available source is allowed at startup. Higher-priority sources
preempt immediately; after expiry, fallback occurs on an incoming message
from the highest-priority remaining live source. No cached IMU is replayed.
Source availability is tracked even without subscribers. Other GPCHC topics
and TMSENMSG temperature publishing are unaffected.

RAWIMUB binary input
-------------------

Serial, TCP and UDP driver entry points accept mixed NMEA / RAWIMUB byte
streams. The text-only ``nmea_sentence`` topic and serial-to-Sentence reader
remain NMEA-only; binary bytes must not be passed through a ROS string.
RAWIMUB follows the supplied CGI protocol workbook dated 20260331:
``AA 44 12``, message ID ``0x010C``, little-endian fields, 40-byte payload,
CRC32 over header and payload (initial zero, reflected polynomial
``0xEDB88320``, no final XOR). A trailing LF is optional (CGI-230 omits it).
Fragmented and concatenated packets are buffered; bad CRC frames are rejected.
Valid other NovAtel long/short messages are skipped, not interpreted as NMEA.

At the required 100 Hz device output rate, acceleration is multiplied by
``100 / 655360`` to obtain m/s^2, and gyro by ``100 / 160849.543863`` to obtain
deg/s, then converted to rad/s. The wire Y field is negated to recover the
marked device Y axis. The same mounting convention as GPCHC is then applied:
device right/front/up -> ROS front/left/up, i.e. ``(y, -x, z)``. Confirm this
mounting against the actual installation before use. No additional gravity
factor is applied to acceleration. Output frequency must actually be 100 Hz;
do not infer the conversion scale from dropped frames or arrival jitter.

RAWIMUB publishes through the existing IMU priority selector. It has no
attitude, so ``orientation_covariance[0] = -1``; angular velocity and
acceleration covariances remain zero (unknown). The workbook supplies no
IMU status fault-bit definition, so the status word is decoded but is not
used to invent a health filter.

With ``is_gps_time: false``, messages use reception time. With it enabled,
RAWIMUB uses payload GPS week/seconds converted to UTC independently of host
timezone, and requires header time status 160 (FINE). The integer parameter
``rawimub_gps_utc_leap_seconds`` defaults to 18 and must be maintained for the
deployment date. Existing GPCHC time conversion is unchanged.
