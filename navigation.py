import config
import geometry


class Navigation:
    def __init__(self):
        self.reset()

    def reset(self):
        self.current = None
        self.target = None

        self.desired_heading = None

        self.heading_error = None
        self.lateral_error = None
        self.velocity = 0.0

    def set_target(self, landmark):
        self.target = landmark

    def set_velocity(self, velocity):
        self.velocity = velocity

    def update(self, localization):
        if not localization.valid():
            return

        self.current = localization.landmark["id"]

        self.next = self.world.get_next_landmark(self.current, self.target)

        if self.next is None:
            return

        self.desired_heading = self.world.get_heading(self.current, self.next)

        if self.desired_heading is None:
            return

        self.heading_error = geometry.normalize_angle(
            self.desired_heading - localization.heading
        )

        self.lateral_error = localization.lateral

    def valid(self):
        return self.heading_error is not None and self.lateral_error is not None
