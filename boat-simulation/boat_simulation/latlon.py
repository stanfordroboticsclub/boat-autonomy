import numpy as np

class LatLon(object):
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon

    def __str__(self):
        return f"[{self.lat}, {self.lon}]"

    def __getitem__(self, item):
        if item == 0:
            return self.lat
        elif item == 1:
            return self.lon
        else:
            raise KeyError(f"index {item} out of bounds for type LatLon")

    __repr__ = __str__

    def addDist(self, dx, dy):
        # pos is a LatLon object, dx is in meters, dy is in meters

        new_lat = self.lat + (dy / 110574)  # 110574 meters per deg latitude
        new_lon = self.lon + (dx / (111320 * np.cos(np.deg2rad(new_lat))))

        return LatLon(new_lat, new_lon)

    @staticmethod
    def dist(pos1, pos2):
        # uses the Haversine formula, returns dist in meters

        r = 6371e3 # radius of Earth in meters

        phi1 = np.deg2rad(pos1.lat)
        phi2 = np.deg2rad(pos2.lat)
        dphi = phi2 - phi1

        dlambda = np.deg2rad(pos2.lon - pos1.lon)

        a = np.sin(dphi/2)**2 + (np.cos(phi1) * np.cos(phi2) * np.sin(dlambda/2)**2)
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))

        return r * c