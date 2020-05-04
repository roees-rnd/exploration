import numpy as np
import copy
import time
from scipy.signal import convolve2d
from scipy.spatial import Delaunay
import cv2 as cv
import matplotlib.pyplot as plt

def xy_to_ij(x, y, x_lim, y_lim, res):
    i = int(np.floor((x - x_lim[0]) / res))
    j = int(np.floor((y - y_lim[0]) / res))
    return i, j

def k_means_los(n_of_centers, matrix_convmat, res, drone_pos, win_range, x_lim, y_lim, kmeans_flag, currdronespos):
    
    res_factor = 2 #TODO: Inbal: check if this factor is still relavent
    matrix = copy.deepcopy(np.array(matrix_convmat))
    if win_range:
        x_min = np.maximum(drone_pos[0] - win_range, x_lim[0] + (res_factor * res))
        x_max = np.minimum(drone_pos[0] + win_range, x_lim[1] - (res_factor * res))
        y_min = np.maximum(drone_pos[1] - win_range, y_lim[0] + (res_factor * res))
        y_max = np.minimum(drone_pos[1] + win_range, y_lim[1] - (res_factor * res))

    else:
        x_min = x_lim[0] + (res_factor * res)
        x_max = x_lim[1] - (res_factor * res)
        y_min = y_lim[0] + (res_factor * res)
        y_max = y_lim[1] - (res_factor * res)

    i_min, j_min = xy_to_ij(x_min, y_min, x_lim, y_lim, res)
    i_max, j_max = xy_to_ij(x_max, y_max, x_lim, y_lim, res)

    # n = 3 # 0.2 meter around the agent #TODO: Inbal: check if this factor is still relavent
    # abs_mat = np.abs(matrix)
    # convmat = convolve2d(abs_mat, np.ones([n,n]), mode='same', boundary='fill', fillvalue=1)

    ##### Delaunay triangulation ####

    # abs_mat = abs_mat.astype(np.uint8)
    # _, thresh = cv.threshold(abs_mat, 0, 255, cv.THRESH_BINARY)
    # _, contours, _ = cv.findContours(thresh, cv.RETR_CCOMP, cv.CHAIN_APPROX_SIMPLE)
    # cv.drawContours(abs_mat, contours, -1, (0, 255, 0), 3)

    # minval = np.min(convmat[convmat > 0])
    # mapenv = np.where(convmat == minval)
    # mapenv = np.array(zip(mapenv[0], mapenv[1])).tolist()
    # # Delaunay triangulations
    # tri = Delaunay(mapenv)
    #
    # fig = plt.figure()
    # fig.add_subplot(1, 2, 1)
    # plt.imshow(convmat)
    # for i, p in enumerate(mapenv):
    #     plt.plot(p[1], p[0], 'o')  # contour of the available area
    # plt.title('convmat')
    # fig.add_subplot(1, 2, 2)
    # plt.triplot(np.array(mapenv)[:, 0], np.array(mapenv)[:, 1], tri.simplices)
    # plt.plot(np.array(mapenv)[:, 0], np.array(mapenv)[:, 1], 'o')
    # for j, s in enumerate(tri.simplices):
    #     p = tri.points[s].mean(axis=0)
    #     plt.text(p[0], p[1], '#%d' % j, ha='center')  # label triangles
    # plt.title('Delaunay triangulation')

    allowed_samp_idxs = np.where(matrix[i_min:i_max, j_min:j_max] == 0)
    allowed_samp = np.array(zip(allowed_samp_idxs[0] + i_min, allowed_samp_idxs[1] + j_min)).tolist()

    # # # start = time.time()
    # allowed_samp_idxs = np.where(matrix[i_min:i_max, j_min:j_max] == 0)
    # allowed_samp_temp = np.array(zip(allowed_samp_idxs[0], allowed_samp_idxs[1])).tolist()
    # allowed_samp = []
    # for idx, elem in enumerate(allowed_samp_temp):
    #     if (matrix[elem[0] + i_min - 1][elem[1] + j_min - 1] == 0 and
    #             matrix[elem[0] + i_min][elem[1] + j_min - 1] == 0 and
    #             matrix[elem[0] + i_min + 1][elem[1] + j_min - 1] == 0 and
    #             matrix[elem[0] + i_min + 1][elem[1] + j_min] == 0 and
    #             matrix[elem[0] + i_min + 1][elem[1] + j_min + 1] == 0 and
    #             matrix[elem[0] + i_min][elem[1] + j_min + 1] == 0 and
    #             matrix[elem[0] + i_min - 1][elem[1] + j_min + 1] == 0 and
    #             matrix[elem[0] + i_min - 1][elem[1] + j_min] == 0):
    #         allowed_samp.append([elem[0] + i_min, elem[1] + j_min])
    # # # end = time.time()
    # # # t3 = (end - start) * 1000  # msec


    idxs = list(range(len(allowed_samp)))
    np.random.shuffle(idxs)
    nodes_idxs = idxs[:n_of_centers]
    nodes = [allowed_samp[n] for n in nodes_idxs if allowed_samp[n] not in currdronespos]
    nodes = np.array(nodes)

    if kmeans_flag:
        data = np.array(allowed_samp)
        data = np.reshape(data, (len(data), 2))

        # Number of clusters
        k = len(nodes)
        # Number of training data
        n = data.shape[0]
        # Number of features in the data
        c = data.shape[1]

        centers_old = np.zeros(nodes.shape)  # to store old centers
        centers_new = copy.deepcopy(nodes)  # Store new centers

        clusters = np.zeros(n)
        distances = np.zeros((n, k))

        error = np.linalg.norm(np.subtract(centers_new, centers_old))

        # When, after an update, the estimate of that center stays the same, exit loop
        #while error > res:
        while error > res:
            # Measure the distance to every center
            for i in range(k):
                distances[:, i] = np.linalg.norm(data - nodes[i], axis=1)
            # Assign all training data to closest center
            clusters = np.argmin(distances, axis=1)

            centers_old = copy.deepcopy(centers_new)
            # Calculate mean for every cluster and update the center
            for i in range(k):
                centers_new[i] = np.mean(data[clusters == i], axis=0)
            error = np.linalg.norm(centers_new - centers_old)

        nodes = [centers_new[n] for n in range(len(centers_new)) if matrix[centers_new[n][0]][centers_new[n][1]] == 0]
        nodes = np.array(nodes)

    return nodes


def remove_nodes_close_to_v(nodes, v, d):
    if len(nodes)==0:
        return v
    list_of_legal_nodes = [] 
    for node in nodes:
        if np.linalg.norm([node[0]-v[0][0], node[1]-v[0][1]])>=d:
            list_of_legal_nodes.append(node)

    if len(list_of_legal_nodes)==0:
        nodes = np.concatenate([v, nodes], axis=0)
        return nodes
    
    return list_of_legal_nodes

def spread_virtual_vertices(n_of_centers, matrix, res, drone_pos, win_range, x_lim, y_lim, kmeans_flag, currdronespos):
    num_of_known_cells = np.shape(np.where(matrix != -1))[1]
    num_of_cells = np.shape(matrix)[0]*np.shape(matrix)[1]
    num_of_centers = np.ceil(float(num_of_known_cells)/float(num_of_cells)*100)

    if num_of_centers<10:
        nodes = k_means_los(n_of_centers, matrix, res, drone_pos, win_range, x_lim, y_lim, kmeans_flag, currdronespos)
        return nodes

    
    res_factor = 2 #TODO: Inbal: check if this factor is still relavent
    matrix = copy.deepcopy(np.array(matrix))
    if win_range:
        x_min = np.maximum(drone_pos[0] - win_range, x_lim[0] + (res_factor * res))
        x_max = np.minimum(drone_pos[0] + win_range, x_lim[1] - (res_factor * res))
        y_min = np.maximum(drone_pos[1] - win_range, y_lim[0] + (res_factor * res))
        y_max = np.minimum(drone_pos[1] + win_range, y_lim[1] - (res_factor * res))

    else:
        x_min = x_lim[0] + (res_factor * res)
        x_max = x_lim[1] - (res_factor * res)
        y_min = y_lim[0] + (res_factor * res)
        y_max = y_lim[1] - (res_factor * res)

    i_min, j_min = xy_to_ij(x_min, y_min, x_lim, y_lim, res)
    i_max, j_max = xy_to_ij(x_max, y_max, x_lim, y_lim, res)

    # n = 3 # 0.2 meter around the agent #TODO: Inbal: check if this factor is still relavent
    # abs_mat = np.abs(matrix)
    # convmat = convolve2d(abs_mat, np.ones([n,n]), mode='same', boundary='fill', fillvalue=1)

    # allowed_samp_idxs = np.where(convmat[i_min:i_max, j_min:j_max] == 0)
    # allowed_samp = np.array(zip(allowed_samp_idxs[0] + i_min, allowed_samp_idxs[1] + j_min)).tolist()

    i_jump = 10#(i_max-i_min)/5
    j_jump =10#(j_max-j_min)/5
    nodes = list()
    for i_curr in range(i_min, i_max, i_jump):
        for j_curr in range(j_min, j_max, j_jump):
            if matrix[i_curr][j_curr]==0: #TODO: Inbal: if free=[0,100] change this to be a value for free
                nodes.append(np.array([i_curr, j_curr]))
    
    curr_pos = currdronespos[0]
    nodes.append(np.array([curr_pos[0], curr_pos[1]]))
    return np.array(nodes)
