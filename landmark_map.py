import json


class LandmarkMap:
    def __init__(self, filename):
        with open(filename, "r") as file:
            self.data = json.load(file)

    def get_landmark(self, id):
        for landmark in self.data["landmarks"]:
            if landmark["id"] == id:
                return landmark

        return None

    def find_landmark(self, tag_id):
        for landmark in self.data["landmarks"]:
            for helper_tag_id in landmark["tags"].values():
                if helper_tag_id == tag_id:
                    return landmark

        return None
