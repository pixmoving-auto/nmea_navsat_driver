"""Select one live IMU source; independent of ROS and message timestamps."""

import time


class ImuSourceSelector:
    PRIORITY = ('gpchc', 'rawimub', 'tmsenmsg')

    PERIOD_SECONDS = 0.01  # All IMU sources run at 100 Hz.

    def __init__(self, missed_cycles=5, clock=time.monotonic):
        if type(missed_cycles) is not int or missed_cycles <= 0:
            raise ValueError('imu_source_missed_cycles must be a positive integer')
        self.timeout = missed_cycles * self.PERIOD_SECONDS
        self._clock = clock
        self._last_seen = {}
        self.active_source = None

    def accept(self, source):
        """Record a complete message and allow only the highest-priority live source.

        Call for every valid message, including those from suppressed sources.
        No stale messages are replayed on fallback; only the incoming message
        may be published. At startup the first available source is allowed.
        """
        if source not in self.PRIORITY:
            raise ValueError('Unknown IMU source: %s' % source)
        now = self._clock()
        self._last_seen[source] = now
        self.active_source = next(
            candidate for candidate in self.PRIORITY
            if candidate in self._last_seen
            and now - self._last_seen[candidate] < self.timeout)
        return source == self.active_source
