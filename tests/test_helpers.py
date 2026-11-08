"""Offline tests of Spectral_BLDC.helpers against a fake drive that answers as firmware 109 does.

Run from the repository root:  python -m unittest discover -s tests -v
No CAN adapter is needed. The fake builds its answers with the firmware's byte layouts (constants.h), keeps a sound
queue of 31 events and a sample buffer of 1023 that it plays in real time, and can lose frames, stay silent, or
mix in frames from another node.
"""

import collections
import os
import struct
import sys
import time
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import can  # noqa: E402

import Spectral_BLDC as Spectral  # noqa: E402
from Spectral_BLDC import CAN_utils as util  # noqa: E402
from Spectral_BLDC import helpers  # noqa: E402
from Spectral_BLDC.Spectral_BLDC import CanMessage  # noqa: E402

S = Spectral.SpectralCAN
READ_ONLY = {23, 31, 32, 33, 36, 39, 40, 41, 42, 45, 47, 48, 54, 55, 57, 61, 62, 63, 79, 80, 83}


class FakeDrive:
    """One drive on the fake bus."""

    def __init__(self, node, product=1):
        self.node = node
        self.product = product
        self.silent = False
        self.chatter_node = None          # answer also as this node first, with a wrong value
        self.drop_once = set()            # buffer indexes lost the first time they are sent
        self.params = {1: 0, 2: 2000, 3: 0, 4: 1000, 5: 0, 6: 0, 7: 0, 8: 5.0, 9: 0.02, 10: 0.001, 16: 1500.0,
                       17: 0, 24: 4, 25: 1, 26: 8, 27: 1, 28: 0, 29: 0, 30: 0, 31: 0, 32: 0, 33: -1, 39: 1024,
                       43: 0, 45: 123, 52: 0, 54: 85, 55: 40, 79: (1 << 0) | (1 << 2) | (1 << 4) | (2 << 16),
                       81: 2000, 82: 1}
        self.state_script = {}            # parameter -> list of values the next reads give (the last one stays)
        self.capture = []
        self.position = 12345
        self.last_hold = None
        self.sent_params = []
        # sound
        self.queue = collections.deque()
        self.queue_max = 0
        self.queue_drops = 0
        self.played = []
        self.q_running = False
        self.next_due = 0.0
        self.ring = collections.deque()
        self.ring_drops = 0
        self.received_samples = bytearray()
        self.r_playing = False
        self.r_underrun = False
        self.r_rate = 0
        self.r_due = 0.0
        self.r_last = 0.0

    # the drive's clock: the queue and the samples play in real time
    def update(self, now):
        while self.q_running and now >= self.next_due:
            if not self.queue:
                self.q_running = False
                break
            ev = self.queue.popleft()
            self.played.append(ev)
            self.next_due += ev[3] / 1000.0
        if self.r_playing:
            self.r_due += (now - self.r_last) * self.r_rate
            self.r_last = now
            while self.r_due >= 1 and self.ring:
                self.ring.popleft()
                self.r_due -= 1
            if self.r_due >= 1:
                self.r_playing, self.r_underrun, self.r_due = False, True, 0.0

    def frame(self, cmd, data, node=None):
        return can.Message(arbitration_id=util.combine_2_can_id(self.node if node is None else node, cmd, 0),
                           data=bytes(data), is_extended_id=False, timestamp=time.time())

    def param_frame(self, number, value, node=None):
        if number in S.FLOAT_PARAMETERS:
            raw = struct.pack('>f', value)
        else:
            raw = int(value).to_bytes(4, 'big', signed=True)
        return self.frame(S.RESPOND_PARAMETER_ID, bytes([number]) + raw, node)

    def get(self, number):
        script = self.state_script.get(number)
        if script:
            self.params[number] = script.pop(0) if len(script) > 1 else script[0]
        if number == 80:
            return (min(len(self.queue), 255) | (len(self.ring) << 8) | (int(self.q_running) << 25)
                    | (int(self.r_playing) << 26) | (int(self.r_underrun) << 27))
        return self.params.get(number)

    def set(self, number, value):
        if number in S.FLOAT_PARAMETERS:                  # the gains and limits travel as a float's bits
            value = struct.unpack('>f', int(value).to_bytes(4, 'big', signed=True))[0]
        self.sent_params.append((number, value))
        if number in READ_ONLY or number not in self.params:
            return
        if number == 81 and not 0 <= value <= 6000:
            return                                      # refused
        if number == 1 and not 0 <= value <= 255:
            return
        if number == 27:
            value = max(1, min(65535, value))           # clamped
        if number == 28:
            if value == 1:                              # 'now': done after one more read of the state
                n = self.params[39]
                self.capture = [(i, -i, (7 * i) % 30000) for i in range(n)]
                self.state_script[31] = [1, 3]
                self.params[32], self.params[33] = n, 0
            return
        self.params[number] = value

    def handle(self, cmd, data, remote, now):
        """The answers to one frame from the host, as a list of can.Message."""
        out = []
        if cmd == S.SEND_PARAMETER_ID and len(data) == 5:
            self.set(data[0], int.from_bytes(data[1:5], 'big', signed=True))
        elif cmd == S.SEND_GET_PARAMETER_ID and len(data) == 1:
            value = self.get(data[0])
            if self.chatter_node is not None:
                out.append(self.param_frame(data[0], -999, node=self.chatter_node))
            out.append(self.frame(S.RESPOND_PARAMETER_ID, data[:1]) if value is None else
                       self.param_frame(data[0], value))
        elif cmd == S.SEND_RESPOND_IDENTITY_ID and remote:
            out.append(self.frame(cmd, [self.product, self.params[1], 0, 109, 3, 7, 1, 0b0100_1000]))
        elif cmd == S.SEND_RESPOND_TELEMETRY_ID and remote:
            out.append(self.frame(cmd, struct.pack('>hhBBh', 31, 24050, 0b1001_0000, 0b1110_0000, -250)))
        elif cmd == S.SEND_RESPOND_STATE_OF_ERRORS_ID and remote:
            out.append(self.frame(cmd, [0b0000_0000, 0b1100_0000]))
        elif cmd == S.SEND_RESPOND_ENCODER_DATA_ID and remote:
            out.append(self.frame(cmd, struct.pack('>ii', self.position, 0)))
        elif cmd == S.SEND_DATA_PACK_1_ID and len(data) == 8:
            self.last_hold = util.fuse_3_bytes(b'\x00' + bytes(data[0:3]))
            out.append(self.frame(S.RESPOND_DATA_PACK_1_ID, bytes(data[0:3]) + bytes(5)))
        elif cmd == S.SEND_BUFFER_READ_ID and len(data) == 5:
            kind, first, count = data[0], int.from_bytes(data[1:3], 'big'), int.from_bytes(data[3:5], 'big')
            step = 3 if kind in (S.BUFFER_COGGING_TABLE, S.BUFFER_ENCODER_TABLE) else 1
            for index in range(first, min(first + count, S.BUFFER_LENGTHS[kind]), step):
                if index in self.drop_once:
                    self.drop_once.discard(index)
                    continue
                if kind == S.BUFFER_CAPTURE:
                    values = self.capture[index]
                else:
                    values = [index + k for k in range(3)]
                body = b''.join(int(v).to_bytes(2, 'big', signed=True) for v in values)
                out.append(self.frame(S.RESPOND_BUFFER_DATA_ID, ((kind << 13) | index).to_bytes(2, 'big') + body))
        elif cmd == S.SEND_SOUND_ID and len(data) == 8:
            op, channel = data[0], data[1]
            a, b, c = struct.unpack('>HHH', bytes(data[2:8]))
            if op == 0:
                self.queue.clear()
                self.q_running = self.r_playing = False
                self.ring.clear()
            elif op == 2:
                if len(self.queue) >= S.SOUND_QUEUE_FREE:
                    self.queue_drops += 1
                else:
                    self.queue.append((channel, a, b, c))
                    self.queue_max = max(self.queue_max, len(self.queue))
            elif op == 3:
                self.q_running, self.next_due = True, now
            elif op == 4:
                self.r_playing, self.r_underrun, self.r_rate, self.r_due, self.r_last = True, False, a, 0.0, now
            elif op == 5:
                self.r_playing = False
                self.ring.clear()
            elif op == 6:
                self.queue.clear()
                self.q_running = False
        elif cmd == S.SEND_SOUND_SAMPLES_ID:
            for s in data:
                if len(self.ring) >= S.SOUND_BUFFER:
                    self.ring_drops += 1
                else:
                    self.ring.append(s)
                    self.received_samples.append(s)
        return [] if self.silent else out


class FakeBus(Spectral.CanCommunication):
    """CanCommunication without hardware: the frames go to FakeDrive objects, their answers come back."""

    def __init__(self, *drives):
        super().__init__(channel=None)
        self.drives = {d.node: d for d in drives}
        self.rx = collections.deque()
        self.sent = []

    def send_can_message(self, message_id, data=b'', remote_frame=False):
        node, cmd, _ = util.extract_from_can_id(message_id)
        self.sent.append((node, cmd, bytes(data), remote_frame))
        now = time.perf_counter()
        for d in self.drives.values():
            d.update(now)
        drive = self.drives.get(node)
        if drive is not None:
            self.rx.extend(drive.handle(cmd, bytes(data), remote_frame, now))

    def receive_can_messages(self, timeout=None):
        now = time.perf_counter()
        for d in self.drives.values():
            d.update(now)
        if not self.rx:
            # Nothing can arrive while this waits (the fake answers only when it is sent to), so a real adapter
            # would wait out the whole timeout.
            if timeout:
                time.sleep(timeout)
            return None, None
        message = self.rx.popleft()
        return message, CanMessage(*util.extract_from_can_id(message.arbitration_id))


def setup(*drives):
    drives = drives or (FakeDrive(0),)
    bus = FakeBus(*drives)
    motors = [Spectral.SpectralCAN(node_id=d.node, communication=bus) for d in drives]
    return bus, drives, motors


class Parameters(unittest.TestCase):
    def test_get_int_float_unknown(self):
        bus, (drive,), (motor,) = setup()
        self.assertEqual(helpers.get_parameter(motor, S.PARAM_WD_FALLBACK_TIME), 2000)
        kpp = helpers.get_parameter(motor, S.PARAM_KPP)
        self.assertIsInstance(kpp, float)
        self.assertAlmostEqual(kpp, 5.0)
        t0 = time.perf_counter()
        self.assertIsNone(helpers.get_parameter(motor, 200))               # unknown: the drive says so at once
        self.assertLess(time.perf_counter() - t0, 0.1)
        self.assertEqual(motor.parameters[S.PARAM_WD_FALLBACK_TIME], 2000)

    def test_no_answer(self):
        bus, (drive,), (motor,) = setup()
        drive.silent = True
        t0 = time.perf_counter()
        self.assertIsNone(helpers.get_parameter(motor, S.PARAM_TOOL_ID, timeout=0.05, retries=1))
        self.assertGreaterEqual(time.perf_counter() - t0, 0.09)            # asked twice
        self.assertEqual(sum(1 for f in bus.sent if f[1] == S.SEND_GET_PARAMETER_ID), 2)

    def test_stale_answer_and_other_node(self):
        bus, (drive,), (motor,) = setup()
        drive.params[S.PARAM_WD_TIME] = 50
        bus.rx.append(drive.param_frame(S.PARAM_WD_TIME, 123))            # a late answer to an earlier request
        drive.chatter_node = 3                                             # and node 3 answers first, every time
        self.assertEqual(helpers.get_parameter(motor, S.PARAM_WD_TIME), 50)
        other = Spectral.SpectralCAN(node_id=3, communication=bus)
        self.assertEqual(helpers.get_parameter(motor, S.PARAM_WD_TIME, others=[other]), 50)
        self.assertEqual(other.parameters[S.PARAM_WD_TIME], -999)          # unpacked into the right object

    def test_set_parameter(self):
        bus, (drive,), (motor,) = setup()
        self.assertTrue(helpers.set_parameter(motor, S.PARAM_SOUND_LIMIT_MV, 2500))
        self.assertEqual(drive.params[81], 2500)
        self.assertFalse(helpers.set_parameter(motor, S.PARAM_SOUND_LIMIT_MV, 9000))   # refused: stays 2500
        self.assertEqual(motor.parameters[81], 2500)
        self.assertFalse(helpers.set_parameter(motor, S.PARAM_CAPTURE_DIVIDER, 0))     # clamped to 1
        self.assertEqual(motor.parameters[27], 1)
        self.assertIsNone(helpers.set_parameter(motor, S.PARAM_TOOL_ID, 1, check=False))
        self.assertEqual(drive.params[1], 1)
        self.assertIsNone(helpers.set_parameter(motor, S.PARAM_ENCAL, 1))             # an action: not read back
        self.assertEqual(drive.sent_params[-1], (52, 1))
        self.assertTrue(helpers.set_parameter(motor, S.PARAM_KPV, 0.0375))            # a float gain (firmware 109)
        self.assertAlmostEqual(drive.params[9], 0.0375, places=6)
        self.assertAlmostEqual(motor.parameters[9], 0.0375, places=6)
        with self.assertRaises(ValueError):
            helpers.set_parameter(motor, S.PARAM_TOOL_ID, 1.5)
        drive.silent = True
        self.assertIsNone(helpers.set_parameter(motor, S.PARAM_TOOL_ID, 2, timeout=0.02))

    def test_get_parameters_listing(self):
        bus, (drive,), (motor,) = setup()
        values = helpers.get_parameters(motor)
        self.assertEqual(values[S.PARAM_SOUND_LIMIT_MV], 2000)
        self.assertNotIn(83, values)                                       # this fake does not know 83
        known = set(drive.params) | {S.PARAM_SOUND_STATE}
        self.assertEqual(set(values), known & (set(S.PARAMETER_NAMES) | set(S.PARAM_PROFILE)))


class State(unittest.TestCase):
    def test_identity_telemetry_errors_position_features(self):
        bus, (drive,), (motor,) = setup(FakeDrive(0, product=2))
        ident = helpers.read_identity(motor)
        self.assertEqual((ident['product'], ident['firmware_version'], ident['pole_pairs']), ('Spectral Micro', 109, 7))
        self.assertTrue(ident['flags']['settings_block2'] and ident['flags']['anti_cogging_saved'])
        self.assertEqual(helpers.loop_rate(motor), 5000)
        tel = helpers.read_telemetry(motor)
        self.assertEqual((tel['temperature'], tel['voltage'], tel['current']), (31, 24050, -250))
        self.assertTrue(tel['error'] and tel['vbus_error'] and tel['calibrated'] and tel['watchdog_error'])
        self.assertFalse(tel['encoder_error'] or tel['wrong_product'])
        err = helpers.errors(motor)
        self.assertTrue(err['calibrated'] and err['activated'])
        self.assertFalse(err['error'])
        self.assertEqual(helpers.read_position(motor), 12345)
        f = helpers.features(motor)
        self.assertTrue(f['calibrated'] and f['encoder table in use'] and f['anti-cogging on'])
        self.assertFalse(f['sound playing'])
        self.assertEqual(f['after calibration'], 'encoder table and anti-cogging')
        drive.silent = True
        self.assertIsNone(helpers.read_identity(motor, timeout=0.02))
        self.assertIsNone(helpers.features(motor, timeout=0.02))

    def test_hold(self):
        bus, (drive,), (motor,) = setup()
        self.assertEqual(helpers.hold(motor, seconds=0.25, period=0.05), 12345)
        self.assertEqual(drive.last_hold, 12345)
        self.assertGreaterEqual(sum(1 for f in bus.sent if f[1] == S.SEND_DATA_PACK_1_ID), 4)

    def test_wait_for_state(self):
        bus, (drive,), (motor,) = setup()
        drive.state_script[S.PARAM_ENCAL] = [1, 1, 2, 2, 3]
        seen = []
        state = helpers.wait_for_state(motor, S.PARAM_ENCAL, poll=0.01, timeout=5,
                                       on_change=lambda t, s: seen.append(s))
        self.assertEqual(state, 3)
        self.assertEqual(seen, [1, 2, 3])
        drive.state_script[S.PARAM_COGCAL] = [7]                           # busy for ever
        self.assertEqual(helpers.wait_for_state(motor, S.PARAM_COGCAL, poll=0.01, timeout=0.2), 7)
        drive.silent = True
        self.assertEqual(helpers.wait_for_state(motor, S.PARAM_COGCAL, poll=0.01, timeout=5, silent_limit=0.1), None)
        with self.assertRaises(ValueError):
            helpers.wait_for_state(motor, S.PARAM_TOOL_ID)


class Buffers(unittest.TestCase):
    def test_read_buffer_retries_lost_frames(self):
        bus, (drive,), (motor,) = setup()
        drive.capture = [(i, i + 1, i + 2) for i in range(1024)]
        drive.drop_once = set(range(5, 1024, 37))
        got = helpers.read_buffer(motor, S.BUFFER_CAPTURE, 0, 1024)
        self.assertEqual(len(got), 1024)
        self.assertEqual(got[500], (500, 501, 502))
        requests = [f for f in bus.sent if f[1] == S.SEND_BUFFER_READ_ID]
        self.assertGreater(len(requests), 1024 // 64)                      # the lost ones were asked for again

    def test_read_buffer_table(self):
        bus, (drive,), (motor,) = setup()
        drive.drop_once = {3, 60}
        got = helpers.read_buffer(motor, S.BUFFER_ENCODER_TABLE)
        self.assertEqual(sorted(got), list(range(128)))
        self.assertEqual((got[0], got[4], got[127]), (0, 4, 127))

    def test_read_buffer_gives_up(self):
        bus, (drive,), (motor,) = setup()
        drive.silent = True
        self.assertEqual(helpers.read_buffer(motor, S.BUFFER_COGGING_HARMONICS, retries=1), {})

    def test_capture(self):
        bus, (drive,), (motor,) = setup()
        drive.drop_once = {10, 700}
        calls = []
        result = helpers.capture(motor, ('position_error', 'iq_setpoint', 'velocity'), divider=4,
                                 while_waiting=calls.append)
        self.assertIsNotNone(result)
        self.assertEqual(result.signals, ('position_error', 'iq_setpoint', 'velocity'))
        self.assertEqual([len(c) for c in result.channels], [1024] * 3)
        self.assertEqual(result.channels[0][700], 700)
        self.assertEqual(result.trigger_index, 0)
        self.assertAlmostEqual(result.sample_period, 4 / 6250)
        self.assertTrue(calls)
        self.assertEqual([drive.params[n] for n in (24, 25, 26, 27)], [7, 3, 4, 4])
        result = helpers.read_capture(motor)                               # names asked from the drive
        self.assertEqual(result.signals, ('position_error', 'iq_setpoint', 'velocity'))

    def test_unwrap16(self):
        self.assertEqual(helpers.unwrap16([32000, 32767, -32768, -32000]), [32000, 32767, 32768, 33536])


class Sound(unittest.TestCase):
    def test_play_events_refills_the_queue(self):
        bus, (drive,), (motor,) = setup()
        events = [(0, 200 + 10 * k, 250, 8) for k in range(80)] + [(0, 0, 0, 0)]
        self.assertTrue(helpers.play_events(motor, events))
        self.assertEqual(drive.played, events)
        self.assertEqual(drive.queue_drops, 0)
        self.assertLessEqual(drive.queue_max, S.SOUND_QUEUE_FREE)
        self.assertEqual(drive.last_hold, 12345)                           # held where it was

    def test_play_events_two_drives(self):
        bus, drives, motors = setup(FakeDrive(0), FakeDrive(1))
        drives[1].position = -500
        ev = {0: [(0, 300 + k, 200, 5) for k in range(45)], 1: [(k % 4, 400 + k, 200, 0 if k % 4 < 3 else 20)
                                                                 for k in range(48)]}
        self.assertTrue(helpers.play_events(motors, ev))
        self.assertEqual(drives[0].played, ev[0])
        self.assertEqual(drives[1].played, ev[1])
        self.assertEqual(drives[1].last_hold, -500)
        self.assertEqual(drives[0].queue_drops + drives[1].queue_drops, 0)

    def test_play_events_gives_up(self):
        bus, (drive,), (motor,) = setup()
        drive.q_running = False
        drive.handle_orig = drive.handle

        def stuck(cmd, data, remote, now):                                 # the queue never moves (drive idle)
            if cmd == S.SEND_SOUND_ID and data[0] == 3:
                return []
            return drive.handle_orig(cmd, data, remote, now)
        drive.handle = stuck
        t0 = time.perf_counter()
        self.assertFalse(helpers.play_events(motor, [(0, 440, 200, 10)] * 40, timeout=0.5))
        self.assertLess(time.perf_counter() - t0, 1.5)

    def test_stream_samples(self):
        bus, (drive,), (motor,) = setup()
        samples = bytes((k * 7) & 0xFF for k in range(4000))
        progress = []
        underruns = helpers.stream_samples(motor, samples, rate=4000, volume=1500,
                                           on_progress=lambda *a: progress.append(a))
        self.assertEqual(underruns, 0)
        self.assertEqual(drive.ring_drops, 0)
        self.assertEqual(bytes(drive.received_samples), samples)
        self.assertTrue(progress)
        self.assertEqual(drive.last_hold, 12345)

    def test_stream_samples_after_a_stall(self):
        bus, (drive,), (motor,) = setup()
        samples = bytes((k * 3) & 0xFF for k in range(6000))
        stalled = []

        def stall(t):                                                      # the host is held up for 0.4 s once
            if t > 0.3 and not stalled:
                stalled.append(t)
                time.sleep(0.4)
        underruns = helpers.stream_samples(motor, samples, rate=4000, while_playing=stall)
        self.assertEqual(underruns, 1)                                     # it ran dry once and was restarted
        self.assertEqual(drive.ring_drops, 0)                              # no burst into the stopped buffer
        self.assertEqual(bytes(drive.received_samples), samples)
        self.assertFalse(drive.r_playing)                                  # returned after the last sample

    def test_stream_samples_ints_and_no_hold(self):
        bus, (drive,), (motor,) = setup()
        self.assertEqual(helpers.stream_samples(motor, [-128, 0, 127] * 300, rate=5000, hold=False), 0)
        self.assertIsNone(drive.last_hold)
        self.assertEqual(bytes(drive.received_samples), bytes([128, 0, 127] * 300))


if __name__ == '__main__':
    unittest.main()
