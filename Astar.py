#!/usr/bin/env python
import math
import numpy as np
from bresenham import bresenham
from k_means_los import k_means_los, spread_virtual_vertices
import time
import matplotlib.pyplot as plt
import copy
from scipy.signal import convolve2d


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind


class Astar:

    def __init__(self, x_lim, y_lim, matrix, res, tf_prefix, num_of_temp_nodes=25):
        self.x_lim = x_lim
        self.y_lim = y_lim

        self.matrix = matrix
        abs_mat = np.abs(matrix)
        self.matrix_LOS =  convolve2d(abs_mat, np.ones([2,2]), mode='same', boundary='fill', fillvalue=1)
        self.matrix_obsmap = None
        
        self.res = res
        self.tf_prefix = tf_prefix
        self.scanning_range = 25
        self.num_of_temp_nodes = num_of_temp_nodes
        self.no_path_flag = False

    def PlanningAlg(self, sx, sy, gx, gy):

        # sx: start x position [m]
        # sy: start y position [m]
        # gx: goal x position [m]
        # gy: goal y position [m]

        nstart = Node(sx, sy, 0, -1)
        ngoal = Node(gx, gy, 0, -1)

        # Derive obstacle map from the Grid
        obmap, minx, miny, maxx, maxy, xw, yw = self.calc_obstacle_map()
        self.matrix_obsmap = (convolve2d(np.abs(obmap), np.ones([3, 3]), mode='same', boundary='fill', fillvalue=1)>0)

        #print obmap:
        # import cv2
        # obmap = obmap.astype(np.uint8)  #convert to an unsigned byte
        # obmap*=255
        # cv2.imshow('Indices',obmap)
        # cv2.waitKey()


        # import matplotlib.pyplot as plt
        # fig = plt.figure(45645)
        # plt.imshow(np.transpose(obmap), origin='lower')
        # g_i, g_j = self.xy_to_ij(gx, gy)
        # s_i, s_j = self.xy_to_ij(sx, sy)
        # plt.scatter(s_j,s_i,s=25, c='red', marker='o')
        # plt.scatter(g_j,g_i,s=25, c='blue', marker='o')
        # # plt.scatter(mx*10,my*10,s=25, c='green', marker='o')
        # # plt.scatter(sx/self.res,sy/self.res,s=25, c='red', marker='o')
        # # plt.scatter(gx/self.res,gy/self.res,s=25, c='blue', marker='o')
        # plt.show()




        # If the path free exit
        g_i, g_j = self.xy_to_ij(gx, gy)
        s_i, s_j = self.xy_to_ij(sx, sy)
        if self.is_path_free(s_i, s_j, g_i, g_j):#, obmap):
            self.no_path_flag = False
            Astar_path = []
            return Astar_path

        # Choose motion nodes 
        # start_time = time.time()
        mx, my = self.get_motion_nodes(sx, sy, gx, gy)
        # end_time = time.time()
        # t = (end_time - start_time) * 1000  # msec
        # print("k_means took: ", t, " msec")

        # # Save obmat to file for debugging
        # image = copy.deepcopy(obmap)
        # for i in range(len(mx)):
        #     t = self.xy_to_ij(mx[i], my[i])
        #     image[t[0]][t[1]] = 100
        # for i in range(-3,3):
        #     for j in range(-3, 3):
        #         image[g_j + j, g_i + i] = 50
        #         image[s_j + j, s_i + i] = 50
        # plt.imsave('map.png', image)
        # ###################################

        openset, closedset = dict(), dict()
        openset[self.calc_index(nstart, xw, minx, miny)] = nstart


        # print("\nstart run of Astar")
        start_time = time.time()
        count_time_for_edges = 0
        num_of_edges_checked_full_run = 0
        while 1:

            try:
                c_id = min(openset, key=lambda o: openset[o].cost)
            except:
                self.no_path_flag = True
                Astar_path = []
                return Astar_path

            current = copy.deepcopy(openset[c_id])

            if abs(current.x - gx) <= self.res and abs(current.y - gy) <= self.res:
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = copy.deepcopy(current)

            start_time_motion = time.time()
            motion, num_of_edges_checked = self.get_motion_model(mx, my, current, obmap, ngoal)
            end_time_motion = time.time()
            count_time_for_edges += (end_time_motion - start_time_motion) * 1000
            num_of_edges_checked_full_run += num_of_edges_checked

            # expand search grid based on motion model
            for i in range(len(motion)):
                node = Node(int(motion[i][0]),
                            int(motion[i][1]),
                            current.cost + motion[i][2], c_id)

                n_id = self.calc_index(node, xw, minx, miny)

                if n_id in closedset:
                    continue

                if not self.verify_node(node, obmap, minx, miny, maxx, maxy):
                    continue

                if n_id not in openset:
                    openset[n_id] = node  # Discover a new node
                else:
                    if openset[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        openset[n_id] = node

        Astar_path = self.calc_fianl_path(ngoal, closedset)
        self.no_path_flag = False
        end_time = time.time()
        t = (end_time - start_time) * 1000  # msec
        # print("edges calc took: ", count_time_for_edges, " msec")
        # print("number of edges checked: ",num_of_edges_checked_full_run, ", avrage time for calc edge took: ", num_of_edges_checked_full_run/count_time_for_edges)
        # print("Astar took: ", t, " msec")
        # print("\n")

        return Astar_path

    def calc_heuristic(self, n1, n2):
        w = 1.0  # weight of heuristic
        d = w * np.linalg.norm(np.subtract([n1.x, n1.y], [n2.x, n2.y]))

        return d

    def verify_node(self, node, obmap, minx, miny, maxx, maxy):

        if node.x < minx:
            return False
        elif node.y < miny:
            return False
        elif node.x >= maxx:
            return False
        elif node.y >= maxy:
            return False

        i, j = self.xy_to_ij(node.x, node.y)
        if obmap[i][j]:
            return False

        return True

    def calc_obstacle_map(self):

        minx = self.x_lim[0]
        miny = self.y_lim[0]
        maxx = self.x_lim[1]
        maxy = self.y_lim[1]

        xwidth = round(maxx - minx)
        ywidth = round(maxy - miny)

        obmap = self.matrix != 0

        return obmap, minx, miny, maxx, maxy, xwidth, ywidth

    def calc_index(self, node, xwidth, xmin, ymin):
        return (node.y - ymin) * xwidth + (node.x - xmin)

    def get_motion_nodes(self, sx, sy, gx, gy):
        mx, my = [], []
        si, sj = self.xy_to_ij(sx, sy)
        gi, gj = self.xy_to_ij(gx, gy)
        win_range = 2 * np.linalg.norm(np.subtract([sx, sy], [gx, gy]))
        nodes = k_means_los(self.num_of_temp_nodes, self.matrix_obsmap, self.res, [sx, sy], win_range,
                        self.x_lim, self.y_lim, False, [[si, sj], [gi, gj]])
        # nodes = spread_virtual_vertices(self.num_of_temp_nodes, self.matrix_obsmap, self.res, [sx, sy], win_range,
        #                 self.x_lim, self.y_lim, False, [[si, sj], [gi, gj]])
        for idx, elem in enumerate(nodes):
            tempx, tempy = self.ij_to_xy(elem[0], elem[1])
            mx.append(tempx)
            my.append(tempy)
        mx.append(gx)
        my.append(gy)


        return mx, my

    def get_motion_model(self, mx, my, nstart, obmap, ngoal):

        okways = []
        num_of_edges_checked = 0
        s_i, s_j = self.xy_to_ij(nstart.x, nstart.y)
        for elem in zip(mx, my):
            if np.abs(nstart.x-elem[0])<self.scanning_range:
                if np.abs(nstart.y-elem[1])<self.scanning_range:
                    g_i, g_j = self.xy_to_ij(elem[0], elem[1])
                    if self.is_path_free(s_i, s_j, g_i, g_j):#, convmat):
                        dist_s_g = np.linalg.norm(np.array([nstart.x, nstart.y]) - np.array(elem))
                        dcost = dist_s_g + self.calc_heuristic(ngoal, nstart)
                        okways.append([elem[0], elem[1], dcost])
                    num_of_edges_checked+=1


        #print("cehcked ", num_of_edges_checked, " edges")
        return okways, num_of_edges_checked

    def is_path_free(self, si, sj, gi, gj):
        return self.is_LOS(si, sj, gi, gj)#, self.matrix_obsmap)

    def is_LOS(self, x1,y1,x2,y2):
        """check line of sight between source and target considering DSM.
        Returns True if LOS exists, False otherwise."""
        obsmap = self.matrix_LOS
        LOS_DISTANCE_IN_PIXERL_TOLERANCE = 0

        if(obsmap[x1,y1]==True or obsmap[x2,y2]==True):
            return False

        #calc distance:
        dxy = int(np.rint(np.sqrt(np.square(x1-x2) + np.square(y1-y2))))

        #if distance is 0- LOS exists
        if dxy <= LOS_DISTANCE_IN_PIXERL_TOLERANCE:
            return True

        x_ind = np.rint(np.linspace(x1, x2, dxy)).astype(int)
        y_ind = np.rint(np.linspace(y1, y2, dxy)).astype(int)

        values_along_los = obsmap[x_ind, y_ind]
        if np.sum(values_along_los)>0:
            return False

        return True

#     def is_path_free(self, si, sj, gi, gj):#, obmap_convmat):

#         # # plot obmap and convmap
#         # import matplotlib.pyplot as plt
#         # fig = plt.figure(45645)
#         # plt.subplot(1,2,1)
#         # plt.imshow(np.transpose(obmap), origin='lower')
#         # plt.subplot(1,2,2)
#         # plt.imshow(np.transpose(convmat), origin='lower')
#         # plt.show()

#         ok_way = True
#         bpath = list(bresenham(si, sj, gi, gj))

#         # # start = time.time()
#         # for ii, elem in enumerate(bpath[1:-1]):
#         #     if (obmap[elem[0]][elem[1]] or
#         #             obmap[elem[0] - 1][elem[1] - 1] or
#         #             obmap[elem[0]][elem[1] - 1] or
#         #             obmap[elem[0] + 1][elem[1] - 1] or
#         #             obmap[elem[0] + 1][elem[1]] or
#         #             obmap[elem[0] + 1][elem[1] + 1] or
#         #             obmap[elem[0]][elem[1] + 1] or
#         #             obmap[elem[0] - 1][elem[1] + 1] or
#         #             obmap[elem[0] - 1][elem[1]]):
#         #         ok_way = False
#         #         break
#         # # end = time.time()
#         # # t2 = (end - start) * 1000  # msec


#         for ii, elem in enumerate(bpath[1:-1]):
#             if (self.matrix_LOS[elem[0]][elem[1]]):
#                 return False
#  #               break
#         # end = time.time()
#         # t2 = (end - start) * 1000  # msec

#  #       return ok_way
#         return True

    def calc_fianl_path(self, ngoal, closedset):
        # generate final path
        final_path = [[ngoal.x, ngoal.y]]
        # rx, ry = [ngoal.x], [ngoal.y]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            final_path.append([n.x, n.y])
            pind = n.pind

        return list(reversed(final_path))

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i*self.res + self.res/2
        y = self.y_lim[0] + j*self.res + self.res/2
        return x, y

def build_trj(pos, env_limits, res, matrix, goal, tf_prefix, num_of_temp_nodes=25):
    num_of_temp_nodes
    x_lim = env_limits[0:2]
    y_lim = env_limits[2:4]
    astar = Astar(x_lim, y_lim, matrix, res, tf_prefix, num_of_temp_nodes)

    gx = goal[0]
    gy = goal[1]

    astar_movement = astar.PlanningAlg(pos[0], pos[1], gx, gy)
    Astar_Movement = astar_movement[1:-1]

    return Astar_Movement, astar.no_path_flag