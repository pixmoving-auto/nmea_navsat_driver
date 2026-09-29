"""Incrementally frame mixed NMEA / NovAtel streams for RAWIMUB input.

Return NMEA strings and RAWIMUB bytes; consume other valid binary frames.
Protocol constants and CRC validation are shared with the RAWIMUB parser.
"""

import struct

from libnmea_navsat_driver.rawimub import MESSAGE_ID, SYNC, crc32


class MixedStreamDecoder:
    """Incremental NMEA / NovAtel binary framing; never decode binary as text.

    Valid other NovAtel long/short packets are consumed atomically, not
    searched for embedded NMEA. Corrupt packets resynchronize byte by byte.
    An incomplete candidate is bounded by MAX_FRAME bytes.
    """
    MAX_FRAME = 8192

    def __init__(self):
        self.buffer = bytearray()

    def reset(self):
        self.buffer.clear()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            if self.buffer[0] == 0xAA:
                if len(self.buffer) < 3:
                    break
                if self.buffer[:3] not in (SYNC, b'\xaa\x44\x13'):
                    del self.buffer[0]
                    continue
                short = self.buffer[2] == 0x13
                if len(self.buffer) < (12 if short else 28):
                    break
                header_len = 12 if short else self.buffer[3]
                payload_len = self.buffer[3] if short else struct.unpack_from('<H', self.buffer, 8)[0]
                message_id = struct.unpack_from('<H', self.buffer, 4)[0]
                total = header_len + payload_len + 4
                if (header_len < (12 if short else 28) or total > self.MAX_FRAME
                        or (not short and message_id == MESSAGE_ID and payload_len != 40)):
                    del self.buffer[0]
                    continue
                if len(self.buffer) < total:
                    break
                frame = bytes(self.buffer[:total])
                if crc32(frame[:-4]) != struct.unpack_from('<I', frame, total - 4)[0]:
                    del self.buffer[0]
                    continue
                del self.buffer[:total]
                if not short and message_id == MESSAGE_ID:
                    result.append(frame)
            elif self.buffer[0] == ord('$'):
                newline = self.buffer.find(b'\n')
                # Recover a truncated ASCII line before the next sentence/frame.
                starts = [self.buffer.find(marker, 1) for marker in (b'$', SYNC, b'\xaa\x44\x13')]
                starts = [pos for pos in starts if pos >= 0]
                if starts and (newline < 0 or min(starts) < newline):
                    del self.buffer[:min(starts)]
                    continue
                if newline < 0:
                    if len(self.buffer) > self.MAX_FRAME:
                        del self.buffer[0]
                        continue
                    break
                line = bytes(self.buffer[:newline]).rstrip(b'\r')
                del self.buffer[:newline + 1]
                try:
                    result.append(line.decode('ascii'))
                except UnicodeDecodeError:
                    pass
            else:
                del self.buffer[0]
        return result
