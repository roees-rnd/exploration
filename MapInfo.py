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
        self.map= None

    def set_map(self, map_data):
        self.map = np.reshape(copy.deepcopy(map_data), (self.map_sizeX,self.map_sizeY))
