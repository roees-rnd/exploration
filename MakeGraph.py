import numpy as np
import networkx as nx
from bresenham import bresenham
from k_means_los import k_means_los, remove_nodes_close_to_v, spread_virtual_vertices
from scipy.spatial import distance
import copy
from scipy.signal import convolve2d


class MakeGraph:
    def __init__(self, number_of_nodes, Grid, res, x_lim, y_lim):
        self.number_of_nodes = number_of_nodes
        self.Grid = Grid
        self.res = res
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.sensor_range = 200  # cm
        self.i_min, self.j_min = self.xy_to_ij(self.x_lim[0], self.y_lim[0])
        self.i_max, self.j_max = self.xy_to_ij(self.x_lim[1], self.y_lim[1])

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0]) / self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y

    def EstTOF(self, pos):
        output_list = np.array([])
        directions_vec = [0, np.pi / 8, np.pi / 4, 3 * np.pi / 8, np.pi / 2, 5 * np.pi / 8,
                          3 * np.pi / 4, 7 * np.pi / 8, np.pi, 9 * np.pi / 8, 5 * np.pi / 4,
                          11 * np.pi / 8, 3 * np.pi / 2, 13 * np.pi / 8, 7 * np.pi / 4, 15 * np.pi / 8] #TODO: inbal: this is np longer the sensore in use!
        for phi in directions_vec:
            output = self.EstRevealedArea(pos, phi)
            if len(output) > 0 and len(output_list) > 0:
                output_list = np.concatenate([output_list, np.array(output)], axis=0)
            elif len(output) > 0 and len(output_list) == 0:
                output_list = output
        return output_list

    def find_greatest_distance(self, array, point):
        dists = distance.cdist(array, point)
        return np.max(dists)

    def EstRevealedArea(self, node_pos, node_dir):
        tof_target_xy = np.add(node_pos, np.round(np.multiply(self.sensor_range, [np.cos(node_dir), np.sin(node_dir)])))
        tof_target_i, tof_target_j = self.xy_to_ij(tof_target_xy[0], tof_target_xy[1])
        node_pos_i, node_pos_j = self.xy_to_ij(node_pos[0], node_pos[1])
        est_revealed_arr = list()
        linepath = list(bresenham(node_pos_i, node_pos_j, int(tof_target_i), int(tof_target_j)))
        for ii, elem in enumerate(linepath[1:]):
            if (elem[0] > 0 and elem[0] < np.shape(self.Grid)[0] and elem[1] > 0 and elem[1] < np.shape(self.Grid)[1]):
                if self.Grid[elem[0]][elem[1]] == 1:
                    break
                if self.Grid[elem[0]][elem[1]] == -1:
                    est_revealed_arr.append([elem[0], elem[1]])
            else:
                break
        return est_revealed_arr

    # def actMakeGraph(self, currdronespos, Grid):
    #     self.Grid = Grid
    #     estnodesflag = False
    #     G = nx.Graph()
    #     while estnodesflag == False:
    #         nodesidxs_ij = k_means_los(self.number_of_nodes, self.Grid, self.res, [], [], self.x_lim, self.y_lim,
    #                                    True, currdronespos)
    #         # nodesidxs_ij = spread_virtual_vertices(self.number_of_nodes, self.Grid, self.res, [], [], self.x_lim, self.y_lim,
    #         #                            True, currdronespos)
    #         # # The first n nodes will be the current positions of the drones.
    #         # # The visibility graph will be built considering these nodes.
    #         # nodesidxs_ij = np.concatenate([currdronespos, nodesidxs_ij], axis=0) if len(nodesidxs_ij) > 0 \
    #         #     else currdronespos
    #         largest_distance = self.find_greatest_distance(nodesidxs_ij, currdronespos)
    #         #d = np.min([largest_distance, 5]) #Inbal: change const to param!
    #         nodesidxs_ij = remove_nodes_close_to_v(nodesidxs_ij, currdronespos, 2) #Inbal: change const to param!

    #         all_est_lists = []
    #         for k in range(len(nodesidxs_ij)):
    #             G.add_node(k)
    #             x_idx, y_idx = self.ij_to_xy(nodesidxs_ij[k][0], nodesidxs_ij[k][1])
    #             G.node[k]['xy_idx'] = [x_idx, y_idx]
    #             list_exp_nodes = self.EstTOF([x_idx, y_idx])
    #             all_est_lists.append(len(list_exp_nodes))
    #             G.node[k]['estexpnodes'] = list_exp_nodes
    #             G.node[k]['ij_idx'] = nodesidxs_ij[k]
    #         if max(all_est_lists) > 0:
    #             estnodesflag = True

    #     for ki in range(G.number_of_nodes()):
    #         for kj in range(G.number_of_nodes()):
    #             node_i_idx = G.node[ki]['ij_idx']
    #             node_j_idx = G.node[kj]['ij_idx']
    #             if np.array_equal(node_i_idx, node_j_idx):
    #                 continue
    #             linepath = list(bresenham(node_i_idx[0], node_i_idx[1], node_j_idx[0], node_j_idx[1]))
    #             freepathflag = True
    #             for ii, elem in enumerate(linepath[1:-1]):
    #                 if self.Grid[elem[0]][elem[1]] != 0:
    #                     freepathflag = False
    #                     break
    #             if freepathflag:
    #                 G.add_edge(ki, kj)
    #     return G, nodesidxs_ij




    def actMakeGraph(self, currdronespos, Grid): #Inbal: check if this is the Grid needed here!!!
        self.Grid = Grid
        estnodesflag = False
        G = nx.Graph()
        convGrid = self.convGrid(copy.deepcopy(self.Grid), 5)
        while estnodesflag == False:
            nodesidxs_ij = k_means_los(self.number_of_nodes, self.Grid, self.res, [], [], self.x_lim, self.y_lim,
                                       True, currdronespos)
            # The first n nodes will be the current positions of the drones.
            # The visibility graph will be built considering these nodes.
            nodesidxs_ij = np.concatenate([currdronespos, nodesidxs_ij], axis=0) if len(nodesidxs_ij) > 0 \
                else currdronespos

            all_est_lists = []
            for k in range(len(nodesidxs_ij)):
                G.add_node(k)
                x_idx, y_idx = self.ij_to_xy(nodesidxs_ij[k][0], nodesidxs_ij[k][1])
                G.node[k]['xy_idx'] = [x_idx, y_idx]
                list_exp_nodes = self.EstTOF([x_idx, y_idx])
                all_est_lists.append(len(list_exp_nodes))
                G.node[k]['estexpnodes'] = list_exp_nodes
                G.node[k]['ij_idx'] = nodesidxs_ij[k]
            if max(all_est_lists) > 0:
                estnodesflag = True

        for ki in range(G.number_of_nodes()):
            for kj in range(G.number_of_nodes()):
                node_i_idx = G.node[ki]['ij_idx']
                node_j_idx = G.node[kj]['ij_idx']
                if np.array_equal(node_i_idx, node_j_idx):
                    continue
                linepath = list(bresenham(node_i_idx[0], node_i_idx[1], node_j_idx[0], node_j_idx[1]))
                freepathflag = True
                for ii, elem in enumerate(linepath[1:-1]):
                    #inbal: when exploring check this!
                    # if convGrid[elem[0]][elem[1]] != 0:
                    #     freepathflag = False
                    #     break
                    if self.Grid[elem[0]][elem[1]] != 0:
                        freepathflag = False
                        break
                if freepathflag:
                    G.add_edge(ki, kj)
        return G, nodesidxs_ij


    def convGrid(self, grid, win):
        convmat = convolve2d(np.abs(grid), np.ones([win, win]), mode='same', boundary='fill', fillvalue=1)
        return convmat





