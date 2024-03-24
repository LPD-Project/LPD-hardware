class DetectedObjectList:
    def __init__(self):
        self.objects = []

    def add_object(self, obj):
        self.objects.append(obj)

    def clear(self):
        self.objects.clear()

    def __len__(self):
        return len(self.objects)

    def __getitem__(self, index):
        return self.objects[index]