import json


class Map:
    def __init__(self):
        with open("map.json", "r") as file:
            self.data = json.load(file)

    def get_landmark(self, id):
        for landmark in self.data["landmarks"]:
            if landmark["id"] == id:
                return landmark

        return None

    def find_landmark(self, tag_id):
        for landmark in self.data["landmarks"]:
            for helper in landmark["tags"].values():
                if helper == tag_id:
                    return landmark

        return None