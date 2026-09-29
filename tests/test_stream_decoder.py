"""ROS-independent regression tests for mixed-stream framing."""

import struct
import unittest

from libnmea_navsat_driver.rawimub import MESSAGE_ID, SYNC, crc32, parse_rawimub
from libnmea_navsat_driver.stream_decoder import MixedStreamDecoder


def packet(payload, message_id=MESSAGE_ID, short=False):
    header = bytearray(12 if short else 28)
    header[:3] = b'\xaa\x44\x13' if short else SYNC
    header[3] = len(payload) if short else len(header)
    struct.pack_into('<H', header, 4, message_id)
    if not short:
        struct.pack_into('<H', header, 8, len(payload))
    frame = bytes(header) + payload
    return frame + struct.pack('<I', crc32(frame))


class MixedStreamDecoderTest(unittest.TestCase):
    def setUp(self):
        self.frame = packet(struct.pack('<IdI6i', 2400, 123.5, 0, 1, 2, 3, 4, 5, 6))

    def test_every_split_boundary(self):
        stream = b'$FIRST\r\n' + self.frame + b'\n$LAST\n'
        for split in range(len(stream) + 1):
            with self.subTest(split=split):
                decoder = MixedStreamDecoder()
                self.assertEqual(decoder.feed(stream[:split]) + decoder.feed(stream[split:]),
                                 ['$FIRST', self.frame, '$LAST'])

    def test_bytewise_input_and_parser(self):
        decoder = MixedStreamDecoder()
        result = []
        for value in self.frame:
            result.extend(decoder.feed(bytes([value])))
        self.assertEqual(result, [self.frame])
        self.assertEqual(parse_rawimub(result[0])['gps_week'], 2400)

    def test_other_binary_packets_are_consumed_atomically(self):
        for short in (False, True):
            with self.subTest(short=short):
                other = packet(b'$NOT_A_SENTENCE\n' + self.frame, 42, short)
                self.assertEqual(MixedStreamDecoder().feed(other + b'$OK\n'), ['$OK'])

    def test_bad_crc_resynchronizes(self):
        corrupt = self.frame[:-1] + bytes([self.frame[-1] ^ 1])
        self.assertEqual(MixedStreamDecoder().feed(corrupt + self.frame), [self.frame])

    def test_truncated_text_resynchronizes(self):
        self.assertEqual(MixedStreamDecoder().feed(b'$TRUNCATED' + self.frame + b'$OK\n'),
                         [self.frame, '$OK'])

    def test_reset_discards_partial_frame(self):
        decoder = MixedStreamDecoder()
        self.assertEqual(decoder.feed(self.frame[:20]), [])
        decoder.reset()
        self.assertEqual(decoder.feed(b'$OK\n'), ['$OK'])

    def test_oversized_text_is_discarded(self):
        decoder = MixedStreamDecoder()
        self.assertEqual(decoder.feed(b'$' + b'x' * decoder.MAX_FRAME), [])
        self.assertEqual(decoder.feed(b'$OK\n'), ['$OK'])


if __name__ == '__main__':
    unittest.main()
