class Navigation:
    def __init__(self, world):
        self.world = world
        self.current = None
        self.target = None

    def set_target(self, landmark):
        self.target = landmark

    def update(self, detections):
        if len(detections) == 0:
            return

        tag_id = detections[0].tag_id
        landmark = self.world.find_landmark(tag_id)

        if landmark is None:
            return

        self.current = landmark
