"""Tests for the Pi-side link framing. Run from Codes/Pi:

    python3 -m unittest test_auv_link -v

These need no hardware: they check the same golden bytes as the STM32 tests.
"""

import struct
import unittest

import auv_link as al


GOLDEN = {
    "heartbeat_seq7": (al.build_heartbeat(7), bytes.fromhex("AA 55 01 01 07 6B")),
    "setpoint_50_m10_100": (al.build_setpoint(50, -10, 100), bytes.fromhex("AA 55 03 02 32 F6 64 EC")),
    "ack_seq7": (al.encode(al.MSG_ACK, bytes([7])), bytes.fromhex("AA 55 01 03 07 41")),
    "telemetry": (
        al.encode(al.MSG_TELEMETRY, bytes([1, 50, 0xF6, 100])),
        bytes.fromhex("AA 55 04 10 01 32 F6 64 E6"),
    ),
}


def feed_all(parser, data):
    frames = []
    for b in data:
        f = parser.feed(b)
        if f:
            frames.append(f)
    return frames


class TestEncoding(unittest.TestCase):
    def test_golden_bytes(self):
        for name, (got, expect) in GOLDEN.items():
            with self.subTest(name=name):
                self.assertEqual(got, expect)

    def test_crc8_check_value(self):
        # CRC-8 (poly 0x07, init 0) of "123456789" is 0xF4
        self.assertEqual(al.crc8(b"123456789"), 0xF4)

    def test_setpoint_is_clamped(self):
        frame = al.build_setpoint(thrust=500, rudder=-999, ballast=-300)
        parser = al.Parser()
        (msg_type, payload), = feed_all(parser, frame)
        self.assertEqual(msg_type, al.MSG_SETPOINT)
        self.assertEqual(struct.unpack("<bbb", payload), (100, -45, -100))

    def test_payload_too_long_rejected(self):
        with self.assertRaises(ValueError):
            al.encode(al.MSG_TELEMETRY, bytes(al.MAX_PAYLOAD + 1))


class TestParsing(unittest.TestCase):
    def test_roundtrip(self):
        frame = al.build_setpoint(50, -10, 100)
        frames = feed_all(al.Parser(), frame)
        self.assertEqual(frames, [(al.MSG_SETPOINT, struct.pack("<bbb", 50, -10, 100))])

    def test_bad_crc_rejected(self):
        bad = bytearray(GOLDEN["heartbeat_seq7"][1])
        bad[-1] ^= 0xFF
        self.assertEqual(feed_all(al.Parser(), bad), [])

    def test_garbage_before_frame(self):
        stream = bytes([0x00, 0xFF, 0xAA, 0xAA, 0x12]) + GOLDEN["heartbeat_seq7"][1]
        frames = feed_all(al.Parser(), stream)
        self.assertEqual(frames, [(al.MSG_HEARTBEAT, bytes([7]))])

    def test_two_frames_back_to_back(self):
        stream = GOLDEN["heartbeat_seq7"][1] + GOLDEN["ack_seq7"][1]
        frames = feed_all(al.Parser(), stream)
        self.assertEqual(frames, [(al.MSG_HEARTBEAT, bytes([7])), (al.MSG_ACK, bytes([7]))])

    def test_oversize_length_rejected_then_recovers(self):
        p = al.Parser()
        self.assertEqual(feed_all(p, bytes([0xAA, 0x55, 0x11])), [])
        self.assertEqual(
            feed_all(p, GOLDEN["heartbeat_seq7"][1]),
            [(al.MSG_HEARTBEAT, bytes([7]))],
        )

    def test_zero_length_payload(self):
        frame = al.encode(0x42)
        self.assertEqual(feed_all(al.Parser(), frame), [(0x42, b"")])


if __name__ == "__main__":
    unittest.main()
