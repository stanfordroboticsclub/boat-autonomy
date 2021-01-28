import numpy as np
from time import time

# heavily based off of https://github.com/stanfordroboticsclub/RoverIMU/blob/master/compass.py
class ComplementaryFilter(object):
    """Complementary filter to estimate orientation."""

    def __init__(self, new_reading_weight):
        super(ComplementaryFilter, self).__init__()
        self.last_sin = None
        self.last_cos = None
        self.last_gyro_read = None
        self.new_weight = new_reading_weight
        self.old_weight = 1 - self.new_weight


    def update_magnetometer(self, heading):
        h_rad = np.deg2rad(heading)
        h_sin = np.sin(h_rad)
        h_cos = np.cos(h_rad)
        if self.last_sin is None:
            self.last_sin = h_sin
            self.last_cos = h_cos
        else:
            self.last_sin = self.new_weight*h_sin + self.old_weight*self.last_sin
            self.last_cos = self.new_weight*h_cos + self.old_weight*self.last_cos


    def update_gyro(self, omega):
        if self.last_gyro_read is not None and self.last_sin is not None:
            h_deg = np.rad2deg(np.arctan2(self.last_sin, self.last_cos))
            h_deg += omega * (time() - self.last_gyro_read)

            h_rad = np.deg2rad(h_deg)
            self.last_sin = np.sin(h_rad)
            self.last_cos = np.cos(h_rad)
        self.last_gyro_read = time()


    def get_heading(self):
        return np.rad2deg(np.arctan2(self.last_sin, self.last_cos))
