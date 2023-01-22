

class KmeansInfo:
    cluster_centers = None
    blue_center = None
    background_centers = None
    yellow_center = None
    red_center = None

    def __init__(self, cluster_centers, blue_center, yellow_center, red_center, background_centers):
        self.cluster_centers = cluster_centers
        self.blue_center = blue_center
        self.yellow_center = yellow_center
        self.red_center = red_center
        self.background_centers = background_centers