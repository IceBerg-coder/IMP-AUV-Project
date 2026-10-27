"""Raspberry Pi 5 side of the Pi <-> STM32 link. Protocol: Codes/AUV_Link/PROTOCOL.md

Usage (on the Pi, with the Nucleo connected to the UART pins):

    from auv_link import LinkClient
    link = LinkClient("/dev/serial0")
    link.start()
    link.set_setpoint(thrust=0, rudder=0, ballast=0)
    ...
    link.close()

Heartbeats are sent every 500 ms by a background thread. Setpoints are sent
with set_setpoint(); call it at least every 500 ms while in control, otherwise
the STM32 failsafe will trigger after 3 s.
"""

import struct
import threading
import time

SYNC1 = 0xAA
SYNC2 = 0x55
MAX_PAYLOAD = 16

MSG_HEARTBEAT = 0x01
MSG_SETPOINT = 0x02
MSG_ACK = 0x03
MSG_TELEMETRY = 0x10
MSG_LEAK = 0x11

HEARTBEAT_PERIOD_S = 0.5


def crc8(data: bytes, crc: int = 0) -> int:
    """CRC-8, polynomial 0x07, init 0x00 (same as the STM32 side)."""
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode(msg_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([len(payload), msg_type]) + payload
    return bytes([SYNC1, SYNC2]) + body + bytes([crc8(body)])


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(v)))


def build_heartbeat(seq: int) -> bytes:
    return encode(MSG_HEARTBEAT, bytes([seq & 0xFF]))


def build_setpoint(thrust: int, rudder: int, ballast: int) -> bytes:
    payload = struct.pack(
        "<bbb",
        _clamp(thrust, -100, 100),
        _clamp(rudder, -45, 45),
        _clamp(ballast, -100, 100),
    )
    return encode(MSG_SETPOINT, payload)


class Parser:
    """Byte-at-a-time parser. feed() returns (type, payload) when a valid frame completes."""

    WAIT_SYNC1, WAIT_SYNC2, WAIT_LEN, WAIT_TYPE, IN_PAYLOAD, WAIT_CRC = range(6)

    def __init__(self):
        self.reset()

    def reset(self):
        self.state = self.WAIT_SYNC1
        self.len = 0
        self.type = 0
        self.payload = bytearray()
        self.crc = 0

    def feed(self, byte: int):
        s = self.state
        if s == self.WAIT_SYNC1:
            if byte == SYNC1:
                self.state = self.WAIT_SYNC2
        elif s == self.WAIT_SYNC2:
            if byte == SYNC2:
                self.state = self.WAIT_LEN
            elif byte != SYNC1:
                self.state = self.WAIT_SYNC1
        elif s == self.WAIT_LEN:
            if byte > MAX_PAYLOAD:
                self.reset()
            else:
                self.len = byte
                self.crc = crc8(bytes([byte]))
                self.state = self.WAIT_TYPE
        elif s == self.WAIT_TYPE:
            self.type = byte
            self.crc = crc8(bytes([byte]), self.crc)
            self.payload = bytearray()
            self.state = self.WAIT_CRC if self.len == 0 else self.IN_PAYLOAD
        elif s == self.IN_PAYLOAD:
            self.payload.append(byte)
            self.crc = crc8(bytes([byte]), self.crc)
            if len(self.payload) == self.len:
                self.state = self.WAIT_CRC
        elif s == self.WAIT_CRC:
            ok = byte == self.crc
            frame = (self.type, bytes(self.payload)) if ok else None
            self.state = self.WAIT_SYNC1
            return frame
        return None


class LinkClient:
    """Serial link to the STM32. Needs pyserial on the Pi."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200):
        import serial  # imported here so the protocol code can be tested without pyserial

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = Parser()
        self._lock = threading.Lock()
        self._seq = 0
        self._running = False
        self._threads = []
        self.last_telemetry = None  # dict with status, thrust, rudder, ballast
        self.last_leak = None       # dict with wetness, alert
        self.leak_detected = False  # latches True on the first leak alert
        self.last_ack_seq = None

    def start(self):
        self._running = True
        for target in (self._heartbeat_loop, self._read_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

    def close(self):
        self._running = False
        for t in self._threads:
            t.join(timeout=1.0)
        self.ser.close()

    def _write(self, frame: bytes):
        with self._lock:
            self.ser.write(frame)

    def _heartbeat_loop(self):
        while self._running:
            self._seq = (self._seq + 1) & 0xFF
            self._write(build_heartbeat(self._seq))
            time.sleep(HEARTBEAT_PERIOD_S)

    def _read_loop(self):
        while self._running:
            data = self.ser.read(64)
            for b in data:
                frame = self.parser.feed(b)
                if frame:
                    self._handle(*frame)

    def _handle(self, msg_type: int, payload: bytes):
        if msg_type == MSG_ACK and len(payload) == 1:
            self.last_ack_seq = payload[0]
        elif msg_type == MSG_TELEMETRY and len(payload) == 4:
            status, thrust, rudder, ballast = struct.unpack("<Bbbb", payload)
            self.last_telemetry = {
                "failsafe": bool(status & 0x01),
                "link_alive": bool(status & 0x02),
                "leak_latched": bool(status & 0x04),
                "thrust": thrust,
                "rudder": rudder,
                "ballast": ballast,
            }
        elif msg_type == MSG_LEAK and len(payload) == 2:
            wetness, alert = payload
            self.last_leak = {"wetness": wetness, "alert": bool(alert)}
            if alert:
                self.leak_detected = True  # the STM32 latches; the Pi should abort the run

    def set_setpoint(self, thrust: int, rudder: int, ballast: int):
        self._write(build_setpoint(thrust, rudder, ballast))
