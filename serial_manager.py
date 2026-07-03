import serial
import time


class SerialManager:
    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2)  # Wait for ESP32 to boot and calibrate IMU
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self):
        self.serial.close()

    def send_goals(self, desired_heading_deg, lateral_error_mm):
        """
        Send the navigation goals to ESP32.
        - desired_heading_deg: absolute heading in map frame (North=0, East=90, ...)
        - lateral_error_mm: lateral offset from tag in mm (positive = tag to right)
        """
        cmd_head = f"HEAD {desired_heading_deg:.2f}\n"
        cmd_lat = f"LAT {lateral_error_mm:.1f}\n"
        self.serial.write(cmd_head.encode())
        self.serial.write(cmd_lat.encode())

    def start_move(self):
        self.serial.write(b"MOVE\n")

    def stop_move(self):
        self.serial.write(b"STOP\n")

    def set_speed(self, speed_mm_s):
        cmd = f"SPD {speed_mm_s:.1f}\n"
        self.serial.write(cmd.encode())

    def drain_input(self):
        """Discard any pending debug output from ESP32"""
        self.serial.reset_input_buffer()
