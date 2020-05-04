import numpy as np
import copy
import math


class node:
    def __init__(self, state, parent, Q_func, was_expand, ucb, nstate, revealed_tiles, estrolres, nvisits):
        self.state = state
        self.parent = parent
        self.Q_func = Q_func
        self.was_expand = was_expand
        self.ucb = ucb
        self.nstate = nstate
        self.revealed_tiles = revealed_tiles
        self.estrolres = estrolres
        self.nvisits = nvisits

class MCTS:
    def __init__(self, niter, mctssteps):
        # number of allowed iterations
        self.niter = niter
        # The index of each node in the dictionary
        self.ncount = 0
        # The number of MCTS steps
        self.mctssteps = mctssteps

    def act_mcts(self, Graph):
        self.G = Graph
        # all possible connections to each node in the graph (visibility graph)
        self.connections_dict = self.get_connection_list()
        nodes_set = dict()
        p_id = -1
        # The first state is the current position of the drone.
        nodes_set[self.ncount] = node(state=0, parent=p_id, Q_func=0, was_expand=0,
                                      ucb=np.inf, nstate=1, revealed_tiles=[], estrolres=0, nvisits=0)
        nodes_set[self.ncount].revealed_tiles = self.get_revealed_tiles(nodes_set, self.ncount)
        self.ncount += 1

        for iter in range(self.niter):
            current, c_id = self.choose_node(nodes_set, p_id)
            if current.nvisits == 0:
                # Rollout is performed in Q function calculation
                nodes_set = self.backpropogation(nodes_set, c_id)
            else:
                nodes_set = self.expand_node(nodes_set, c_id)
                current, c_id = self.choose_node(nodes_set, c_id)
                # Rollout is performed in Q function calculation
                nodes_set = self.backpropogation(nodes_set, c_id)
            p_id = c_id

        s_id = max(nodes_set, key=lambda o: nodes_set[o].Q_func)
        solution = self.get_final_solution(nodes_set, s_id)
        len_rev_tiles = len(nodes_set[s_id].revealed_tiles)
        return solution, len_rev_tiles

    def get_final_solution(self, nodes_set, s_id):
        pind = s_id
        states = list()
        traj = list()
        while pind != -1:
            traj.append(self.G.node[nodes_set[pind].state]['xy_idx'])
            pind = nodes_set[pind].parent
        return list(reversed(traj))[1:]

    def get_revealed_tiles(self, nodes_set, c_id):
        revealed_tiles = list()
        if nodes_set[c_id].parent != -1:
            revealed_tiles = copy.deepcopy(nodes_set[nodes_set[c_id].parent].revealed_tiles)
        if len(revealed_tiles) == 0 and len(self.G.node[nodes_set[c_id].state]['estexpnodes']) > 0:
            temp_arr = self.G.node[nodes_set[c_id].state]['estexpnodes']
        elif len(self.G.node[nodes_set[c_id].state]['estexpnodes']) == 0 and len(revealed_tiles) > 0:
            temp_arr = revealed_tiles
        else:
            temp_arr = np.concatenate((revealed_tiles, self.G.node[nodes_set[c_id].state]['estexpnodes']), axis=0)
        try:
            revealed_tiles = np.unique(temp_arr, axis=0)
        except:
            revealed_tiles = copy.deepcopy(temp_arr)
        return revealed_tiles

    def backpropogation(self, nodes_set, c_id):
        nodes_set[c_id].nvisits += 1
        nodes_set[c_id].Q_func = self.calc_Q_func(nodes_set, c_id)
        nodes_set = self.calc_ucb(nodes_set, c_id)
        cn_id = c_id
        while nodes_set[cn_id].parent != -1:
            cn_id = nodes_set[cn_id].parent
            nodes_set[cn_id].nvisits += 1
            nodes_set = self.calc_ucb(nodes_set, cn_id)
        return nodes_set

    def calc_Q_func(self, nodes_set, c_id):
        # Give more weight to the future when the agent at the beginning of path planning
        # and les when it approaches to the end
        alpha = 0.5
        gamma = 0.9
        try:
            parent_Q_func = nodes_set[nodes_set[c_id].parent].Q_func
        except:
            parent_Q_func = 0
        # Compute new Q function
        future_reward = self.est_future_reward(nodes_set, c_id)
        reward_for_step = len(nodes_set[c_id].revealed_tiles)
        Q_func = (1-alpha)*parent_Q_func + alpha*(reward_for_step + gamma*future_reward)
        return Q_func

    def est_future_reward(self, nodes_set, c_id):
        next_states = self.connections_dict[nodes_set[c_id].state]
        if len(next_states) == 0:
            return 0
        sorted_nodes = sorted(self.G.node, key=lambda o: len(self.G.node[o]['estexpnodes']), reverse=True)
        for elemidx, elem in enumerate(sorted_nodes):
            if elem in next_states:
                max_node_idx = elem
                break
        revealed_tiles = copy.deepcopy(nodes_set[c_id].revealed_tiles)
        if len(revealed_tiles) == 0 and len(self.G.node[max_node_idx]['estexpnodes']) > 0:
            temp_arr = self.G.node[max_node_idx]['estexpnodes']
        elif len(self.G.node[max_node_idx]['estexpnodes']) == 0 and len(revealed_tiles) > 0:
            temp_arr = revealed_tiles
        else:
            temp_arr = np.concatenate((revealed_tiles, self.G.node[max_node_idx]['estexpnodes']), axis=0)
        try:
            revealed_tiles = np.unique(temp_arr, axis=0)
        except:
            revealed_tiles = copy.deepcopy(temp_arr)
        return len(revealed_tiles)

    def choose_node(self, nodes_set, p_id):
        relevant_nodes = dict()
        # The priority is to choose nodes that were expended with current parent id.
        for i, elem in enumerate(nodes_set):
            if nodes_set[elem].nstate < self.mctssteps and \
                    nodes_set[elem].was_expand==0 and \
                    np.array_equal(nodes_set[elem].parent, p_id):
                relevant_nodes[elem] = nodes_set[elem]
        try:
            c_id = max(relevant_nodes, key=lambda o: relevant_nodes[o].ucb)
        except:
            for i, elem in enumerate(nodes_set):
                if nodes_set[elem].nstate < self.mctssteps and nodes_set[elem].was_expand==0:
                    relevant_nodes[elem] = nodes_set[elem]
            c_id = max(relevant_nodes, key=lambda o: relevant_nodes[o].ucb)
        current = relevant_nodes[c_id]
        return current, c_id

    def expand_node(self, nodes_set, c_id):
        # Current node is expanded
        nodes_set[c_id].was_exp = 1
        current = copy.deepcopy(nodes_set[c_id])
        # Extracting all the possible connections for each node in the state
        allowed_nodes = self.connections_dict[nodes_set[c_id].state]
        for i, elem in enumerate(allowed_nodes):
            # Create new node
            nodes_set[self.ncount] = node(state=elem, parent=c_id, Q_func=0, was_expand=0, ucb=np.inf,
                                          nstate=current.nstate + 1, revealed_tiles=[], estrolres=0, nvisits=0)
            nodes_set[self.ncount].revealed_tiles = self.get_revealed_tiles(nodes_set, self.ncount)
            self.ncount += 1
        return nodes_set

    def get_connection_list(self):
        connections_dict = dict()
        list_of_nodes = self.G.nodes()
        for i in range(len(list_of_nodes)):
            connections_list = [n for n in self.G.neighbors(list(list_of_nodes.keys())[i])]
            connections_dict[list(list_of_nodes.keys())[i]] = connections_list
        return connections_dict

    def calc_ucb(self, nodes_set, c_id):
        nodes_set[c_id].ucb = len(nodes_set[c_id].revealed_tiles) + math.sqrt(2) * math.sqrt(
            (np.log(nodes_set[c_id].nstate) / nodes_set[c_id].nvisits))

        return nodes_set