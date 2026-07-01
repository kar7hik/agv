import serial


class SerialManager:
    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(
            port,
            baudrate,
            timeout=0.2,
        )

        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def close(self):
        self.serial.close()

    def send_velocity(
        self,
        velocity_mps,
        desired_heading_deg,
        lateral_error_m,
    ):
        command = (
            f"VEL "
            f"{velocity_mps:.3f} "
            f"{desired_heading_deg:.2f} "
            f"{lateral_error_m:.4f}\n"
        )

        self.serial.write(command.encode())

        return self.wait_for_ack()

    def enable(self):
        self.serial.write(b"EN\n")
        return self.wait_for_ack()

    def disable(self):
        self.serial.write(b"DIS\n")
        return self.wait_for_ack()

    def stop(self):
        self.serial.write(b"STOP\n")
        return self.wait_for_ack()

    def ping(self):
        self.serial.write(b"PING\n")
        return self.wait_for_ack()

    def zero_heading(self):
        self.serial.write(b"ZERO\n")
        return self.wait_for_ack()

    def calibrate(self):
        self.serial.write(b"CAL\n")
        return self.wait_for_ack()

    def request_status(self):
        self.serial.write(b"STATUS\n")
        return self.read_line()

    def wait_for_ack(self):
        while True:
            line = self.read_line()

            if line is None:
                return False

            if line == "ACK":
                return True

            if line.startswith("ERR"):
                return False

    def read_line(self):
        if self.serial.in_waiting == 0:
            return None

        return self.serial.readline().decode().strip()
