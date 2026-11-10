"""Offline tests of what SpectralCAN puts on the wire and takes off it, with the bytes written out by hand
(the firmware's layouts in communication_CAN.cpp). No CAN adapter is needed.

Run from the repository root:  python -m unittest discover -s tests -v
"""

import os
import struct
import sys
import unittest

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import can  # noqa: E402

import Spectral_BLDC as Spectral  # noqa: E402
from Spectral_BLDC.Spectral_BLDC import CanMessage  # noqa: E402

S = Spectral.SpectralCAN


class Wire:
    """Stands in for CanCommunication: keeps the frames a motor sends."""

    def __init__(self):
        self.sent = []

    def send_can_message(self, message_id, data=b'', remote_frame=False):
        self.sent.append((message_id, bytes(data)))
        return True


def reply(node, command, data, remote=False, extended=False):
    message = can.Message(arbitration_id=(node << 7) | (command << 1), data=bytes(data), is_extended_id=extended,
                          is_remote_frame=remote)
    return message, CanMessage(node, command, 0)


class MotionFrames(unittest.TestCase):
    def setUp(self):
        self.wire = Wire()
        self.motor = S(node_id=3, communication=self.wire)

    def test_position_frame_bytes(self):
        self.motor.Send_data_pack_1(Position=-2, Speed=1000, Current=-300)
        frame_id, data = self.wire.sent[-1]
        self.assertEqual(frame_id, (3 << 7) | (S.SEND_DATA_PACK_1_ID << 1))
        self.assertEqual(data, bytes([0xFF, 0xFF, 0xFE, 0x00, 0x03, 0xE8, 0xFE, 0xD4]))

    def test_current_only_frame_bytes(self):
        self.motor.Send_data_pack_1(Current=300)
        self.assertEqual(self.wire.sent[-1][1], bytes([0x01, 0x2C]))

    def test_position_beyond_24_bits_is_refused(self):
        # it used to wrap: 10,000,000 went out as -6,777,216
        with self.assertRaises(ValueError):
            self.motor.Send_data_pack_1(Position=10_000_000, Speed=0, Current=0)
        with self.assertRaises(ValueError):
            self.motor.Send_data_pack_PD(Position=-8_388_609, Speed=0, Current=0)
        self.assertEqual(self.wire.sent, [])
        self.motor.Send_data_pack_1(Position=8_388_607, Speed=0, Current=0)      # the largest that fits
        self.motor.Send_data_pack_1(Position=-8_388_608, Speed=0, Current=0)
        self.assertEqual(len(self.wire.sent), 2)

    def test_current_beyond_16_bits_is_refused(self):
        with self.assertRaises(ValueError):
            self.motor.Send_data_pack_1(Current=40000)
        self.assertEqual(self.wire.sent, [])

    def test_incomplete_arguments_send_nothing(self):
        self.motor.Send_data_pack_1(Position=1000)
        self.assertEqual(self.wire.sent, [])


class NodeIds(unittest.TestCase):
    def test_node_id_range(self):
        for bad in (16, -1, 255, 1.0, None):
            with self.assertRaises(ValueError):
                S(node_id=bad, communication=Wire())
        S(node_id=15, communication=Wire())

    def test_new_can_id_range(self):
        wire = Wire()
        motor = S(node_id=0, communication=wire)
        with self.assertRaises(ValueError):
            motor.Send_CAN_ID(255)
        motor.Send_CAN_ID(7)
        self.assertEqual(wire.sent[-1][1], bytes([7]))


class Parameters(unittest.TestCase):
    def setUp(self):
        self.wire = Wire()
        self.motor = S(node_id=0, communication=self.wire)

    def test_float_parameter_travels_as_a_float(self):
        # it used to travel as the integer 1500, which the drive reads as a float of about 2e-42
        self.motor.Send_Parameter(S.PARAM_CURRENT_LIMIT, 1500)
        self.assertEqual(self.wire.sent[-1][1], bytes([S.PARAM_CURRENT_LIMIT]) + struct.pack('>f', 1500.0))
        self.motor.Send_Parameter(S.PARAM_KPV, 0.05)
        self.assertEqual(self.wire.sent[-1][1], bytes([S.PARAM_KPV]) + struct.pack('>f', 0.05))

    def test_integer_parameter_bytes(self):
        self.motor.Send_Parameter(6, 1000)
        self.assertEqual(self.wire.sent[-1][1], bytes([6, 0x00, 0x00, 0x03, 0xE8]))
        self.motor.Send_Parameter(33, -1)
        self.assertEqual(self.wire.sent[-1][1], bytes([33, 0xFF, 0xFF, 0xFF, 0xFF]))
        with self.assertRaises(ValueError):
            self.motor.Send_Parameter(6, 0.5)

    def test_ids_read_unsigned(self):
        for number in (79, 84, 86, 87):
            self.motor.UnpackData(*reply(0, S.RESPOND_PARAMETER_ID, bytes([number, 0xB6, 0x69, 0xFD, 0x2E])))
            self.assertEqual(self.motor.parameters[number], 0xB669FD2E)
        self.motor.UnpackData(*reply(0, S.RESPOND_PARAMETER_ID, bytes([33, 0xFF, 0xFF, 0xFF, 0xFF])))
        self.assertEqual(self.motor.parameters[33], -1)


class Replies(unittest.TestCase):
    def setUp(self):
        self.motor = S(node_id=0, communication=Wire())
        self.motor.UnpackData(*reply(0, S.RESPOND_DATA_PACK_1_ID, bytes([0x00, 0x30, 0x39, 0, 0, 0, 0, 0])))
        self.assertEqual(self.motor.position, 12345)

    def test_another_nodes_reply_is_ignored(self):
        self.motor.UnpackData(*reply(5, S.RESPOND_DATA_PACK_1_ID, bytes([0x00, 0x00, 0x01, 0, 0, 0, 0, 0])))
        self.assertEqual(self.motor.position, 12345)

    def test_remote_and_extended_frames_are_ignored(self):
        self.motor.UnpackData(*reply(0, S.RESPOND_DATA_PACK_1_ID, b'', remote=True))
        self.motor.UnpackData(*reply(0, S.RESPOND_DATA_PACK_1_ID, bytes(8), extended=True))
        self.assertEqual(self.motor.position, 12345)



class GripperAndState(unittest.TestCase):
    def setUp(self):
        self.motor = S(node_id=0, communication=Wire())

    def test_object_detection_bit_order(self):
        # the firmware packs the value's bit 0 at bit 2 and bit 1 at bit 3 of byte 3 (Gripper_pack_data); 1.20 swapped
        # them. Bit 2 alone is 1 (contact, closing), bit 3 alone is 2 (contact, opening); 0x80 is "calibrated".
        for byte3, expected in ((0x20 | 0x01, 1), (0x10 | 0x01, 2), (0x30 | 0x01, 3), (0x01, 0)):
            self.motor.UnpackData(*reply(0, S.RESPOND_GRIPPER_DATA_PACK_ID, bytes([120, 0xFE, 0xD4, byte3])))
            self.assertEqual(self.motor.gripper_object_detection, expected, hex(byte3))
            self.assertEqual(self.motor.gripper_position, 120)
            self.assertEqual(self.motor.gripper_current, -300)

    def test_gripper_current_beyond_16_bits_is_refused(self):
        with self.assertRaises(ValueError):
            self.motor.Send_gripper_data_pack(100, 50, 40000, 1, 1, 0, 0)

    def test_more_unsigned_parameters(self):
        for number in (80, 85):
            self.motor.UnpackData(*reply(0, S.RESPOND_PARAMETER_ID, bytes([number, 0xFF, 0x00, 0x00, 0x01])))
            self.assertEqual(self.motor.parameters[number], 0xFF000001)

    def test_send_parameter_refuses_nan_infinity_and_fractions(self):
        for bad in (float('nan'), float('inf'), 0.5, 2**31):
            with self.assertRaises(ValueError):
                self.motor.Send_Parameter(6, bad)
        with self.assertRaises(ValueError):
            self.motor.Send_Parameter(256, 1)

    def test_voltage_limit_range(self):
        self.motor.Send_Voltage_Limit(6000)
        self.assertEqual(self.motor.communication.sent[-1][1], (6000).to_bytes(4, 'big'))
        for bad in (-5, 60001, 1.5):
            with self.assertRaises(ValueError):
                self.motor.Send_Voltage_Limit(bad)

    def test_reset_flags_and_error_bit(self):
        self.motor.watchdog_error = 1
        self.motor.wrong_product = 1
        message, unpacked = reply(0, S.RESPOND_HEARTBEAT_ID, b'')
        self.motor.UnpackData(message, unpacked._replace(error_bit=1))
        self.assertIsNone(self.motor.watchdog_error)
        self.assertIsNone(self.motor.wrong_product)
        self.assertEqual(self.motor.error_bit, 1)


class SendResult(unittest.TestCase):
    def test_send_returns_false_and_logs_once_without_a_bus(self):
        comm = Spectral.CanCommunication(channel=None)          # no adapter: no bus object
        with self.assertLogs(level='WARNING') as logs:
            self.assertFalse(comm.send_can_message(1))
            self.assertFalse(comm.send_can_message(1))
        self.assertEqual(len(logs.records), 1)                  # logged once until a send works again
        self.assertIsNotNone(comm.last_send_error)


if __name__ == "__main__":
    unittest.main()
