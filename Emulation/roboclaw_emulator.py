import serial
import struct
import threading
import time
import logging

logging.basicConfig(
    level=logging.DEBUG, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)


class RoboClawEmulator:
    def __init__(self, port):
        self.ser = serial.Serial(port, baudrate=115200, timeout=0.1)
        self.logger = logging.getLogger(f"RoboClaw-{port}")
        self.m1_speed = 0
        self.m2_speed = 0
        self.m1_encoder = 0
        self.m2_encoder = 0
        self.main_battery_voltage = 240  # 24.0V
        self.logic_battery_voltage = 50  # 5.0V
        self.running = True
        self.logger.info(f"RoboClaw emulator initialized on port {port}")

    def calculate_crc16(self, packet):
        crc = 0
        for byte in packet:
            crc = crc ^ (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
        return crc & 0xFFFF

    def handle_command(self, command):
        self.logger.debug(f"Received command: {command.hex()}")
        if not command or len(command) < 2:
            self.logger.warning("Received invalid command (too short)")
            return None

        cmd = command[1]
        if cmd == 0x15:  # Read Firmware Version
            version = "RoboClaw v4.1.34"
            response = (
                version.encode()
                + b"\x00"
                + struct.pack(">H", self.calculate_crc16(version.encode() + b"\x00"))
            )
            self.logger.debug(f"Read Firmware Version: {version}")
            return response
        elif cmd == 0x18:  # Read Main Battery Voltage
            response = struct.pack(">H", self.main_battery_voltage) + struct.pack(
                ">H", self.calculate_crc16(struct.pack(">H", self.main_battery_voltage))
            )
            self.logger.debug(
                f"Read Main Battery Voltage: {self.main_battery_voltage/10.0}V"
            )
            return response
        elif cmd == 0x19:  # Read Logic Battery Voltage
            response = struct.pack(">H", self.logic_battery_voltage) + struct.pack(
                ">H",
                self.calculate_crc16(struct.pack(">H", self.logic_battery_voltage)),
            )
            self.logger.debug(
                f"Read Logic Battery Voltage: {self.logic_battery_voltage/10.0}V"
            )
            return response
        elif cmd == 0x25:  # Read Encoder Counts
            response = (
                struct.pack(">I", self.m1_encoder)
                + struct.pack(">I", self.m2_encoder)
                + struct.pack(
                    ">H",
                    self.calculate_crc16(
                        struct.pack(">II", self.m1_encoder, self.m2_encoder)
                    ),
                )
            )
            self.logger.debug(
                f"Read Encoder Counts: M1={self.m1_encoder}, M2={self.m2_encoder}"
            )
            return response
        else:
            self.logger.warning(f"Unhandled command: {cmd}")
            return b"\x00\x00"  # Default response for unhandled commands

    def run(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    command = self.ser.read(self.ser.in_waiting)
                    response = self.handle_command(command)
                    if response:
                        self.ser.write(response)
                        self.logger.debug(f"Sent response: {response.hex()}")
                time.sleep(0.01)
            except Exception as e:
                self.logger.error(f"Error in emulator loop: {str(e)}")

    def stop(self):
        self.running = False
        self.logger.info("Emulator stopped")


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: python roboclaw_emulator.py <serial_port>")
        sys.exit(1)

    port = sys.argv[1]
    emulator = RoboClawEmulator(port)
    emulator_thread = threading.Thread(target=emulator.run)
    emulator_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        emulator.stop()
        emulator_thread.join()
        print("Emulator stopped.")
