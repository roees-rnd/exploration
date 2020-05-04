#!/usr/bin/env python
import numpy as np
import Astar
import math
import networkx as nx
import copy
from MCTS import MCTS
from MultiAgentOptPathSearch import MultiAgentOptPathSearch
from MakeGraph import MakeGraph
from scipy.signal import convolve2d


class LocalMissionPlanner:
    def __init__(self, env_limits, tf_prefix, res, curpos, grid, curyaw, ExploreNewAreaFlag):
        self.env_limits = env_limits
        self.tf_prefix = tf_prefix
        self.res = res
        self.curpos = curpos
        self.nextpos = copy.deepcopy(curpos)
        self.takeoffpos = copy.deepcopy(curpos)
        self.grid = grid
        self.curyaw = curyaw
        self.nextyaw = copy.deepcopy(curyaw)
        self.traj = []
        self.ExploreNewAreaFlag = ExploreNewAreaFlag
        self.NumNodesENA = 20
        self.sensor_range = 200 #[cm]
        self.x_lim = self.env_limits[0:2]
        self.y_lim = self.env_limits[2:4]
        self.makegraph = MakeGraph(self.NumNodesENA, self.grid, self.res, self.x_lim, self.y_lim)
        self.win_range = 300 #[cm]
        self.graph = nx.Graph()
        self.nodesidxs_ij = []
        self.TSflag = True
        self.nofsteps = 2
        self.min_rev_tiles = 2
        if self.TSflag:
            self.mcts = MCTS(100, 3)
            self.multiagentoptpathsearch = MultiAgentOptPathSearch(100, 3, 1)

    def TaskAssignment(self, dronepos, grid, droneyaw, traj, num_of_iter):

        # Parameters for local mission planner
        self.curpos = dronepos
        try:
            self.nextpos = [traj[0]]
        except:
            self.nextpos = dronepos
        self.curyaw = droneyaw
        self.traj = traj
        self.grid = grid

        # Activate explore new area (ExploreNewArea) and then route to the selected location (MaintainSearchState)
        if self.ExploreNewAreaFlag:
            if num_of_iter > 1 and len(self.traj) == 0:
                self.ExploreNewArea()
        self.MaintainSearchState()

        # Find trajectory angles
        self.traj_yaw = self.RotationAngleManager()

    def RotationAngleManager(self):
        angle_arr = []
        vec = np.subtract(self.nextpos, self.curpos)
        if np.linalg.norm(vec) > 0:
            vec_dir = np.divide(vec, np.linalg.norm(vec))
            angle_arr.append(math.atan2(1, vec_dir[0][1] / vec_dir[0][0]))
        else:
            angle_arr.append(self.curyaw)
        if len(self.traj) > 1:
            for x in range(len(self.traj)-1):
                vec = np.subtract(self.traj[x+1], self.traj[x])
                if np.linalg.norm(vec) > self.res:
                    vec_dir = np.divide(vec, np.linalg.norm(vec))
                    angle_arr.append(math.atan2(1, vec_dir[1] / vec_dir[0]))
            angle_arr.append(angle_arr)
        return angle_arr

    def MaintainSearchState(self):

        Astar_Movement, no_path_flag = Astar.build_trj(self.curpos, self.env_limits, self.res, self.grid, self.nextpos,
                                         self.tf_prefix)
        if Astar_Movement:
            self.traj = np.concatenate((Astar_Movement, self.traj), axis=0)

        if self.ExploreNewAreaFlag and no_path_flag:
            if len(self.traj) > 1:
                self.traj = np.delete(self.traj, 0, 0)
            else:
                self.traj = []

    def ReturnToHome(self):

        self.nextpos = self.takeoffpos
        Astar_Movement, no_path_flag = Astar.build_trj(self.curpos, self.env_limits, self.res, self.grid, self.nextpos,
                                         self.tf_prefix)

        if Astar_Movement:
            self.traj = np.concatenate((Astar_Movement, self.traj), axis=0)

        if self.ExploreNewAreaFlag and no_path_flag:
            if len(self.traj) > 1:
                self.traj = np.delete(self.traj, 0, 0)
            else:
                self.traj = []

    def ExploreNewArea(self):
        # start = time.time()
        currdronespos_i, currdronespos_j = self.xy_to_ij(self.curpos[0][0], self.curpos[0][1])
        currdronespos = [[currdronespos_i, currdronespos_j]]
        self.graph, self.nodesidxs_ij = self.makegraph.actMakeGraph(currdronespos, self.grid)
        # end = time.time()
        # t1 = (end - start)*1000 # msec
        if self.TSflag:
            # Find the next trajectory which is composed from n points
            #### MCTS ####
            self.traj, len_rev_tiles = self.mcts.act_mcts(self.graph)
            if len_rev_tiles < self.min_rev_tiles:
                self.traj = self.choose_random_node()
            #### MCTS ####
            # #### DFS ####
            # sol_dict = self.multiagentoptpathsearch.act_OptPathSearch(self.graph)
            # for i in range(len(sol_dict)):
            #     if len(sol_dict[i]) > 0 and len(self.multiagentoptpathsearch.revtiles) >= self.min_rev_tiles:
            #         self.traj = sol_dict[i]
            #     elif self.multiagentoptpathsearch.revtiles < self.min_rev_tiles:
            #         self.traj = self.choose_random_node()
            # #### DFS ####

        else:
            # Find the next trajectory which is composed from one point
            goal_node_idx = sorted(self.graph.node, key=lambda o: len(self.graph.node[o]['estexpnodes']), reverse=True)
            goal_node_idx = goal_node_idx[0:self.nofsteps]
            if len(self.graph.node[goal_node_idx[0]]['estexpnodes']) >= self.min_rev_tiles:
                self.traj = [list(self.graph.node[x]['xy_idx']) for x in goal_node_idx if x != 0]
            else:
                self.traj = self.choose_random_node()

    def choose_random_node(self):
        tiles = []
        x_max = int(math.floor((self.x_lim[1] - 1) / self.res))
        y_max = int(math.floor((self.y_lim[1] - 1) / self.res))
        unrevealed_tiles = np.where(self.grid == -1)
        for idx, elem in enumerate(zip(unrevealed_tiles[0], unrevealed_tiles[1])):
            if (elem[0] < x_max and elem[0] > 0 and elem[1] < y_max and elem[1] > 0):
                if self.grid[elem[0]][elem[1] + 1] == 0:
                    tiles.append([elem[0],elem[1] + 1])
                elif self.grid[elem[0]][elem[1] - 1] == 0:
                    tiles.append([elem[0], elem[1] - 1])
                elif self.grid[elem[0] + 1][elem[1]] == 0:
                    tiles.append([elem[0] + 1, elem[1]])
                elif self.grid[elem[0]- 1][elem[1]] == 0:
                    tiles.append([elem[0] - 1, elem[1]])
        if len(tiles)==0:
            print("len(tiles)==0 in LocalMissionPlanner line 155")
        tidx = np.random.choice(len(tiles), 1)
        goal_noad = self.ij_to_xy(tiles[tidx[0]][0], tiles[tidx[0]][1])
        return [list(goal_noad)]

    def Land(self):
        pass

    def FindCommunication(self):
        pass

    def ImproveOdometry(self):
        pass

    def xy_to_ij(self, x, y):
        i = int(np.floor((x - self.x_lim[0])/self.res))
        j = int(np.floor((y - self.y_lim[0]) / self.res))
        return i, j

    def ij_to_xy(self, i, j):
        x = self.x_lim[0] + i * self.res + self.res / 2
        y = self.y_lim[0] + j * self.res + self.res / 2
        return x, y

    def convGrid(self, grid, win):
        convmat = convolve2d(np.abs(grid), np.ones([win, win]), mode='same', boundary='fill', fillvalue=1)
        return convmat



