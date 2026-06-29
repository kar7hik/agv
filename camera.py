from picamera2 import Picamera2


class Camera:
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.camera = Picamera2()

    def start(self):
        config = self.camera.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.camera.configure(config)
        self.camera.start()

    def get_frame(self):
        return self.camera.capture_array()

    def release(self):
        self.camera.stop()
