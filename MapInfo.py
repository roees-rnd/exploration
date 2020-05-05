
import numpy as np
import copy


class MapInfo:
    def __init__(self, width, height, res, x, y, z):
        self.map_sizeX = width
        self.map_sizeY = height
        self.map_resolution = res
        self.map_delta_x = -x
        self.map_delta_y = -y
        self.map_delta_z = -z
        self.map = None

    def set_map(self, map_data):
        self.map = np.reshape(copy.deepcopy(map_data),
                              (self.map_sizeY, self.map_sizeX)).T

    def xy_to_ij(self, x, y):
        i = int(np.floor((y + self.map_delta_y) / self.map_resolution))
        j = int(np.floor((x + self.map_delta_x) / self.map_resolution))
        return i, j

    def ij_to_xy(self, i, j):
        x = j*self.map_resolution - self.map_delta_x
        y = i*self.map_resolution - self.map_delta_y
        return x, y
