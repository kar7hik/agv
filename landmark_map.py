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
            for helper, helper_tag in landmark["tags"].items():
                if helper_tag == tag_id:
                    return {"landmark": landmark, "helper": helper}

        return None
