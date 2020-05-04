import numpy as np
import copy
import math
# MultiAgentOptPathSearch search for the optimal path of n steps that reveals maximal area for multiple agents

class node:
    def __init__(self, state, parent, Reward, was_expand, nstate, revealed_tiles, visited):
        self.state = state
        self.parent = parent
        self.Reward = Reward
        self.was_expand = was_expand
        self.nstate = nstate
        self.revealed_tiles = revealed_tiles
        self.visited = visited

class MultiAgentOptPathSearch:
    def __init__(self, niter, nsteps, numofagents):
        # number of allowed iterations
        self.niter = niter
        # The number of steps
        self.nsteps = nsteps
        # Random threshold to move to a different state in the tree
        self.randthr = 0.2
        # Total number of agents
        self.numofagents = numofagents
        self.min_rev_tiles = 1

    def act_OptPathSearch(self, Graph):
        self.G = Graph
        # all possible connections to each node in the graph (visibility graph)
        self.connections_dict = self.get_connection_list()
        # The final solution for all agents will be saved here
        MAsolution = dict()
        # Array which saves all the chosen nodes for previous agents
        NodesPath = dict()
        # List of the revealed tiles from chosen paths
        self.revtiles = np.array([])

        for niter in range(self.numofagents):
            # The index of each node in the dictionary
            self.ncount = 0
            solution = np.array([])
            nodes_set = dict()
            p_id = -1
            # The first state is the current position of the drone.
            nodes_set[self.ncount] = node(state=0, parent=p_id, Reward=0, was_expand=0,
                                          nstate=0, revealed_tiles=[], visited=0)
            nodes_set[self.ncount].revealed_tiles = self.get_revealed_tiles(nodes_set, self.ncount)
            nodes_set[self.ncount].Reward = len(nodes_set[self.ncount].revealed_tiles)
            self.ncount += 1

            for iter in range(self.niter):
                c_id = self.choose_node(nodes_set, p_id)
                nodes_set[c_id].visited = 1
                nodes_set = self.expand_node(nodes_set, c_id, NodesPath)
                p_id = copy.deepcopy(c_id)

            s_id = max(nodes_set, key=lambda o: nodes_set[o].Reward)
            solution, selected_nodes = self.get_final_solution(nodes_set, s_id)
            if len(nodes_set[s_id].revealed_tiles) > self.min_rev_tiles and len(selected_nodes) > 0:
                MAsolution[niter] = copy.deepcopy(solution)
                NodesPath[niter] = copy.deepcopy(selected_nodes)
                if len(self.revtiles) > 0 and len(nodes_set[s_id].revealed_tiles) > 0:
                    self.revtiles = np.concatenate([self.revtiles, nodes_set[s_id].revealed_tiles], axis=0)
                elif len(nodes_set[s_id].revealed_tiles) > 0 and len(self.revtiles) == 0:
                    self.revtiles = nodes_set[s_id].revealed_tiles
            else:
                solnodes = []
                for i in NodesPath:
                    solnodes += NodesPath[i]
                goal_node_idx = sorted(self.G.node, key=lambda o: len(self.G.node[o]['estexpnodes']), reverse=True)
                goal_node_idx = [x for x in goal_node_idx if x not in solnodes]
                solution = self.G.node[goal_node_idx[0]]['xy_idx']
                MAsolution[niter] = copy.deepcopy([solution])
                NodesPath[niter] = copy.deepcopy([goal_node_idx[0]])
                if len(self.revtiles) > 0 and len(self.G.node[goal_node_idx[0]]['estexpnodes']) > 0:
                    self.revtiles = np.concatenate([self.revtiles, self.G.node[goal_node_idx[0]]['estexpnodes']], axis=0)
                elif len(self.G.node[goal_node_idx[0]]['estexpnodes']) > 0 and len(self.revtiles) == 0:
                    self.revtiles = self.G.node[goal_node_idx[0]]['estexpnodes']

        return MAsolution

    def get_final_solution(self, nodes_set, s_id):
        pind = s_id
        states = list()
        traj = list()
        selected_nodes = list()
        while pind != -1:
            traj.append(self.G.node[nodes_set[pind].state]['xy_idx'])
            selected_nodes.append(nodes_set[pind].state)
            pind = nodes_set[pind].parent
        return list(reversed(traj))[1:], list(reversed(selected_nodes))[1:]

    def get_revealed_tiles(self, nodes_set, c_id):
        revealed_tiles = list()
        # Include previous revealed tiles
        if nodes_set[c_id].parent != -1:
            revealed_tiles = copy.deepcopy(nodes_set[nodes_set[c_id].parent].revealed_tiles)
        # If the chosen nodes reveals new area than include it
        if len(revealed_tiles) == 0 and len(self.G.node[nodes_set[c_id].state]['estexpnodes']) > 0:
            temp_arr = self.G.node[nodes_set[c_id].state]['estexpnodes']
        # If the chosen node doesnt reveal new area than include the initial reward
        elif len(self.G.node[nodes_set[c_id].state]['estexpnodes']) == 0 and len(revealed_tiles) > 0:
            temp_arr = revealed_tiles
        else:
            temp_arr = np.concatenate((revealed_tiles, self.G.node[nodes_set[c_id].state]['estexpnodes']), axis=0)
        try:
            revealed_tiles = np.unique(temp_arr, axis=0)
        except:
            revealed_tiles = copy.deepcopy(temp_arr)
        return revealed_tiles

    def choose_node(self, nodes_set, p_id):
        relevant_nodes = dict()
        expFlag = False
        rndchoiceFlag = False
        if np.random.random() < self.randthr:
            rndchoiceFlag = True
        # The priority is to choose nodes that were expended with current parent id.
        for i, elem in enumerate(nodes_set):
            if nodes_set[elem].nstate < self.nsteps and \
                    nodes_set[elem].was_expand==0 and \
                    nodes_set[elem].visited==0 and \
                    np.array_equal(nodes_set[elem].parent, p_id):
                relevant_nodes[elem] = nodes_set[elem]
        try:
            c_id = max(relevant_nodes, key=lambda o: relevant_nodes[o].Reward)
        except:
            for i, elem in enumerate(nodes_set):
                if nodes_set[elem].nstate < self.nsteps and nodes_set[elem].was_expand==0:
                    relevant_nodes[elem] = nodes_set[elem]
            if rndchoiceFlag:
                c_id = np.random.choice(list(relevant_nodes.keys()))
            else:
                c_id = max(relevant_nodes, key=lambda o: relevant_nodes[o].Reward)
        return c_id

    def expand_node(self, nodes_set, c_id, NodesPath):
        # Current node is expanded
        nodes_set[c_id].was_exp = 1
        # The number of generated state
        gnstate = nodes_set[c_id].nstate + 1
        # Gather all chosen nodes (solutions for other agents) for this step
        forbidden_nodes = []
        for m in NodesPath:
            if len(NodesPath[m]) > gnstate:
                forbidden_nodes.append(NodesPath[m][gnstate-1])
        # Extracting all the possible connections for each node in the state
        allowed_nodes = self.connections_dict[nodes_set[c_id].state]
        allowed_nodes = [x for x in allowed_nodes if x not in forbidden_nodes]
        for i, elem in enumerate(allowed_nodes):
            rev_set = []
            # Create new node
            nodes_set[self.ncount] = node(state=elem, parent=c_id, Reward=np.inf,
                                          was_expand=0, nstate=gnstate, revealed_tiles=[], visited=0)
            nodes_set[self.ncount].revealed_tiles = self.get_revealed_tiles(nodes_set, self.ncount)
            # Check the approximated number of revealed tiles of the chosen path for one agent and compare it with
            # the revealed tiled of the previous chosen path
            if len(self.revtiles) > 0:
                rev_set = [x for x in nodes_set[self.ncount].revealed_tiles if x not in self.revtiles]
            else:
                rev_set = copy.deepcopy(nodes_set[self.ncount].revealed_tiles)
            nodes_set[self.ncount].Reward = len(rev_set)
            self.ncount += 1
        return nodes_set

    def get_connection_list(self):
        connections_dict = dict()
        list_of_nodes = self.G.nodes()
        for i in range(len(list_of_nodes)):
            connections_list = [n for n in self.G.neighbors(list(list_of_nodes.keys())[i])]
            connections_dict[list(list_of_nodes.keys())[i]] = connections_list
        return connections_dict