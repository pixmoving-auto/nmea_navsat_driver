"""CGI RAWIMUB, per CGI protocol workbook 20260331 (RAWIMUB sheet).

Little-endian long NovAtel header, ID 0x010C, 40-byte payload. The
optional LF is not included in the length or CRC. Rates below are fixed
at the configured device output of 100 Hz, not inferred from arrival gaps.
"""

import math
import struct

SYNC = b'\xaa\x44\x12'
MESSAGE_ID = 0x010C
OUTPUT_HZ = 100.0


def crc32(data):
    """NovAtel reflected CRC32, initial value zero and no final XOR."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc


def parse_rawimub(frame):
    """Return device-axis SI measurements; reject corrupt/unsupported frames."""
    if len(frame) < 28 or frame[:3] != SYNC:
        raise ValueError('Invalid RAWIMUB header')
    header_len = frame[3]
    message_id = struct.unpack_from('<H', frame, 4)[0]
    payload_len = struct.unpack_from('<H', frame, 8)[0]
    if header_len < 28 or message_id != MESSAGE_ID or payload_len != 40 or frame[6] & 0x60:
        raise ValueError('Unsupported RAWIMUB header or payload length')
    length = header_len + payload_len + 4
    if len(frame) != length and not (len(frame) == length + 1 and frame[-1] == 10):
        raise ValueError('Invalid RAWIMUB frame length')
    if crc32(frame[:length - 4]) != struct.unpack_from('<I', frame, length - 4)[0]:
        raise ValueError('RAWIMUB CRC mismatch')
    week, second, status, az, ay, ax, gz, gy, gx = struct.unpack_from('<IdI6i', frame, header_len)
    if not math.isfinite(second) or not 0 <= second < 604800:
        raise ValueError('Invalid RAWIMUB GPS seconds')
    accel_scale = OUTPUT_HZ / 655360.0
    gyro_scale = math.radians(OUTPUT_HZ / 160849.543863)
    return {
        'gps_week': week, 'gps_second': second, 'imu_status': status,
        'time_status': frame[13],
        # The wire Y fields have the opposite sign to the marked device Y axis.
        'acceleration': (ax * accel_scale, -ay * accel_scale, az * accel_scale),
        'angular_velocity': (gx * gyro_scale, -gy * gyro_scale, gz * gyro_scale),
    }


def device_to_ros(vector):
    """Same mounting convention as GPCHC: device right/front/up -> front/left/up."""
    x, y, z = vector
    return y, -x, z


def gps_time_ns(week, second, leap_seconds):
    """UTC Unix nanoseconds, independent of the host timezone."""
    return ((315964800 + week * 604800 - leap_seconds) * 1000000000
            + round(second * 1000000000))
