class Navigation:
    def __init__(self):
        self.current = None
        self.target = None

    def set_target(self, landmark):
        self.target = landmark

    def update(self, localization):
        if not localization.valid():
            return

        self.current = localization.landmark["id"]
