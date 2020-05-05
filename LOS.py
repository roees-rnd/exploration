
import numpy as np
import copy
from scipy.signal import convolve2d
import matplotlib.pyplot as plt

LOS_DISTANCE_IN_PIXERL_TOLERANCE = 0


def create_obsMapForLOS(mapData):
    grid = np.reshape(copy.deepcopy(mapData.data),
                      (mapData.info.width, mapData.info.height))
    matrix_LOS = convolve2d(grid, np.ones(
        [2, 2]), mode='same', boundary='fill', fillvalue=1)
    abs_mat = np.abs(grid)
    matrix_obsmap = (convolve2d(np.abs(abs_mat), np.ones(
        [1, 1]), mode='same', boundary='fill', fillvalue=1) > 0)

    # fig = plt.figure()
    # ax1 = fig.add_subplot(111)
    # ax1.imshow(matrix_obsmap)
    # plt.show()

    return matrix_obsmap


def is_LOS(x1, y1, x2, y2, obsmap):
    """check line of sight between source and target considering DSM.
    Returns True if LOS exists, False otherwise."""
    LOS_DISTANCE_IN_PIXERL_TOLERANCE = 0

    # if(obsmap[x1,y1]==True or obsmap[x2,y2]==True):
    if(obsmap[y1, x1] == True or obsmap[y2, x2] == True):
        return False

    # calc distance:
    dxy = int(np.rint(np.sqrt(np.square(x1-x2) + np.square(y1-y2))))

    # if distance is 0- LOS exists
    if dxy <= LOS_DISTANCE_IN_PIXERL_TOLERANCE:
        return True

    x_ind = np.rint(np.linspace(x1, x2, dxy)).astype(int)
    y_ind = np.rint(np.linspace(y1, y2, dxy)).astype(int)

    if False:
        # plot obsmap and points:
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        obsmap2 = np.abs(obsmap)
        obsmap2[y_ind, x_ind] = 100
        ax1.imshow(obsmap2)
        # plt.scatter(x1, y1, s=25, c='red', marker='o')
        # plt.scatter(x2, y2, s=25, c='red', marker='o')
        plt.show()

    values_along_los = obsmap[y_ind, x_ind]
    if np.sum(values_along_los) > 0:
        return False

    return True
