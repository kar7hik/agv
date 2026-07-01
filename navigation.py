import geometry


class Navigation:
    def __init__(self, world):
        self.world = world
        self.reset()

    def reset(self):
        """
        Reset the navigation state
        """
        self.current = None
        self.target = None
        self.next = None

        self.path = []
        self.path_index = 0

        self.desired_heading = None
        self.lateral_error = None

        self.velocity = 0.0

    def set_target(self, landmark_id):
        self.target = landmark_id

        self.path = []
        self.path_index = 0
        self.next = None

    def set_velocity(self, velocity):
        self.velocity = velocity

    def update(self, localization):
        self.current = None
        self.next = None
        self.desired_heading = None
        self.lateral_error = None

        if self.target is None:
            return

        if not localization.valid():
            return

        self.current = localization.landmark["id"]

        if self.current == self.target:
            self.next = None
            self.desired_heading = 0.0
            self.lateral_error = 0.0
            return

        if not self.path:
            self.path = self.world.find_path(self.current, self.target)
            self.path_index = 0
            if not self.path:
                return

        # Robot deviated from planned path.
        elif self.current != self.path[self.path_index]:
            self.path = self.world.find_path(self.current, self.target)
            self.path_index = 0
            if not self.path:
                return

        # Advance along path.
        if (
            self.path_index + 1 < len(self.path)
            and self.current == self.path[self.path_index + 1]
        ):
            self.path_index += 1

        # Determine next landmark.
        self.next = self.world.get_next_landmark(self.path, self.path_index)

        if self.next is None:
            return

        # Compute the desired heading
        self.desired_heading = self.world.get_heading(self.current, self.next)

        # Compute the lateral error
        self.lateral_error = localization.lateral

    def valid(self):
        return self.desired_heading is not None and self.lateral_error is not None
