import json


class Map:
    def __init__(self):
        self.name = ""
        self.rows = 0
        self.columns = 0
        self.landmark_spacing_m = 0.0
        self.helper_spacing_m = 0.0

        self.landmarks = {}
        self.tag_lookup = {}

    def load(self, filename):
        with open(filename, "r") as file:
            data = json.load(file)

        self.name = data["name"]

        grid = data["grid"]
        self.rows = grid["rows"]
        self.columns = grid["columns"]
        self.landmark_spacing_m = grid["landmark_spacing_m"]
        self.helper_spacing_m = grid["helper_spacing_m"]

        self.landmarks.clear()
        self.tag_lookup.clear()

        for landmark in data["landmarks"]:
            landmark_id = landmark["id"]
            row = landmark["row"]
            column = landmark["column"]

            landmark["neighbors"] = []

            landmark["x"] = column * self.landmark_spacing_m
            landmark["y"] = row * self.landmark_spacing_m

            self.landmarks[landmark_id] = landmark

            for position, tag_id in landmark["tags"].items():
                self.tag_lookup[tag_id] = {
                    "landmark_id": landmark_id,
                    "position": position,
                }
        self.generate_neighbors()

    def generate_neighbors(self):
        lookup = {}

        for landmark in self.landmarks.values():
            lookup[(landmark["row"], landmark["column"])] = landmark["id"]

        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]
