import struct
import unittest

from project.src.bus_servo import (
    BusServoController,
    SERVO_MOVE_TIME_WRITE,
    SERVO_OR_MOTOR_MODE_WRITE,
    SERVO_POS_READ,
    SERVO_TEMP_READ,
    SERVO_VIN_READ,
    ServoStatus,
    build_packet,
)


class FakeSerial:
    def __init__(self, responses=None):
        self.responses = bytearray(b"".join(responses or []))
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def read(self, size=1):
        if not self.responses:
            return b""
        data = self.responses[:size]
        del self.responses[:size]
        return bytes(data)

    def flush(self):
        pass

    def close(self):
        self.closed = True


class BusServoProtocolTest(unittest.TestCase):
    def test_build_move_packet(self):
        packet = build_packet(1, SERVO_MOVE_TIME_WRITE, struct.pack("<HH", 500, 1000))
        self.assertEqual(packet, bytes.fromhex("55 55 01 07 01 F4 01 E8 03 16"))

    def test_build_motor_packet(self):
        packet = build_packet(1, SERVO_OR_MOTOR_MODE_WRITE, b"\x01\x00" + struct.pack("<h", 100))
        self.assertEqual(packet, bytes.fromhex("55 55 01 07 1D 01 00 64 00 75"))

    def test_build_negative_motor_packet(self):
        packet = build_packet(1, SERVO_OR_MOTOR_MODE_WRITE, b"\x01\x00" + struct.pack("<h", -100))
        self.assertEqual(packet, bytes.fromhex("55 55 01 07 1D 01 00 9C FF 3E"))

    def test_read_status(self):
        responses = [
            build_packet(1, SERVO_TEMP_READ, b"\x2A"),
            build_packet(1, SERVO_POS_READ, struct.pack("<h", -10)),
            build_packet(1, SERVO_VIN_READ, struct.pack("<H", 7400)),
        ]
        fake = FakeSerial(responses)
        controller = BusServoController("COM3", serial_factory=lambda *_: fake)

        status = controller.read_status(1)

        self.assertEqual(status, ServoStatus(servo_id=1, temperature_c=42, position=-10, voltage_mv=7400))
        self.assertEqual(fake.writes[0], build_packet(1, SERVO_TEMP_READ))
        self.assertEqual(fake.writes[1], build_packet(1, SERVO_POS_READ))
        self.assertEqual(fake.writes[2], build_packet(1, SERVO_VIN_READ))

    def test_move_servo_writes_expected_frame(self):
        fake = FakeSerial()
        controller = BusServoController("COM3", serial_factory=lambda *_: fake)

        controller.move_servo(1, 500, 1000)

        self.assertEqual(fake.writes, [bytes.fromhex("55 55 01 07 01 F4 01 E8 03 16")])

    def test_move_servo_allows_extended_j3_position(self):
        fake = FakeSerial()
        controller = BusServoController("COM3", serial_factory=lambda *_: fake)

        controller.move_servo(3, 1050, 1000)

        self.assertEqual(
            fake.writes,
            [build_packet(3, SERVO_MOVE_TIME_WRITE, struct.pack("<HH", 1050, 1000))],
        )

    def test_set_motor_speed_writes_expected_frame(self):
        fake = FakeSerial()
        controller = BusServoController("COM3", serial_factory=lambda *_: fake)

        controller.set_motor_speed(1, 100)

        self.assertEqual(fake.writes, [bytes.fromhex("55 55 01 07 1D 01 00 64 00 75")])


if __name__ == "__main__":
    unittest.main()
