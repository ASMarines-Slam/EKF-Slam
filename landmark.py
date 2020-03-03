class Landmark:
    __count = 0

    def __init__(self, xm, ym, signature, matched_index=-1):
        self.x_measured = xm
        self.y_measured = ym
        self.x_local = null
        self.y_local = null
        self.signature = signature
        self.matched_index = matched_index
        self.x_global = null
        self.y_global = null
        Landmark.count += 1

    def get_count(self):
        return self.__count

    def measured_to_local(self, xm, ym):
        pass

    def local_to_global(self, xl, yl):
        pass

