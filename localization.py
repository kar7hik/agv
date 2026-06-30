class Localization:
    def __init__(self, world):
        self.world = world

        self.landmark = None
        self.helper = None
        self.tag = None

    def update(self, detections):
        self.landmark = None
        self.helper = None
        self.tag = None

        if len(detections) == 0:
            return

        # We will use the first detected tag for computation
        # Later we can choose the best tag
        detection = detections[0]

        result = self.world.find_landmark(detection.tag_id)

        if result is None:
            print(f"Unknown tag: {detection.tag_id}")
            return

        self.tag = detection.tag_id
        self.landmark = result["landmark"]
        self.helper = result["helper"]

    def valid(self):
        return self.landmark is not None
