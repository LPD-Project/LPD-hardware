class MapObject:
    def __init__(self, class_name, confidence, left, top, right, bottom):
        self.class_name = class_name
        self.confidence = confidence
        self.left = left#*2.109375 # y
        self.top = top#*3.75 # x
        self.right = right#*2.109375 # y
        self.bottom = bottom#*3.75 # x
