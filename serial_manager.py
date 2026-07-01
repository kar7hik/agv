import serial


class SerialManager:
    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(
            port,
            baudrate,
            timeout=0.1,
        )

    def close(self):
        self.serial.close()

    def send_navigation(
        self,
        velocity,
        heading_error,
        lateral_error,
    ):
        command = (
            f"NAV," f"{velocity:.3f}," f"{heading_error:.2f}," f"{lateral_error:.4f}\n"
        )

        self.serial.write(command.encode())

    def read(self):
        if self.serial.in_waiting == 0:
            return None

        return self.serial.readline().decode().strip()
