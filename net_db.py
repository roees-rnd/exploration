import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import MapInfo as mapi
import LOS


class net_db:
    def __init__(self):
        self.G = nx.Graph()
        self.last_node = None
        self._min_dist = 0.7
        self.r = np.array([])
        self.map_info = None

    def add_node(self, xy=(0, 0), is_door=False):
        added_node = False
        if self.G.number_of_nodes() == 0:  # no nodes yet
            self.G.add_node(xy)
        else:
            # Distance from last node:
            weight = np.linalg.norm([xy[0]-self.last_node[0],xy[1]-self.last_node[1]])
            # weight = np.sqrt(
            #     np.square(xy[0]-self.last_node[0])+np.square(xy[1]-self.last_node[1]))
            # If we are far enough from last node:
            if weight > self._min_dist:
                # If current position is far enough from all other nodes:
                nae, req = self.nodes_are_eq(xy, self._min_dist/2)
                if len(nae) < 1:  # no close nodes
                    if self.map_info is None:
                        self.G.add_node(xy)
                        self.G.add_edge(xy, self.last_node, weight=weight)
                    else:
                        added_node = self.add_node_map(xy)
                        if not added_node:
                            self.G.add_edge(xy, self.last_node, weight=weight)
                else:
                    if not self.G.has_edge(self.last_node, nae[0]):
                        # w = np.sqrt(
                        #     np.square(nae[0][0]-self.last_node[0])+np.square(nae[0][1]-self.last_node[1]))
                        w = req[0]
                        self.G.add_edge(self.last_node, nae[0], weight=w)
                    self.last_node = nae[0]
                    return False
            else:
                return False

        attr = {xy: {'is_door': is_door}}
        self.G.add_node(xy)
        nx.set_node_attributes(self.G, attr)

        self.last_node = xy

        return True

    def add_node_map(self, xy):
        node_added = False
        if self.map_info is None:
            return False
        
        #  TODO: if node is close to any other node, then return without adding
        eq_nodes, _ = self.nodes_are_eq(xy, thresh=0.2)
        if len(eq_nodes)>0:
            return False


        new_ij = self.map_info.xy_to_ij(xy[0], xy[1])
        for n in list(self.G.nodes()):
            n_ij = self.map_info.xy_to_ij(n[0], n[1])
            if LOS.is_LOS(new_ij[0], new_ij[1], n_ij[0], n_ij[1], np.abs(self.map_info.map)>0):
                # w = np.sqrt(np.square(n[0]-xy[0])+np.square(n[1]-xy[1]))
                w = np.linalg.norm([n[0]-xy[0],n[1]-xy[1]])
                self.G.add_edge(xy, n, weight=w)
                if node_added is False:
                    nx.set_node_attributes(self.G, {xy: {'is_door': False}})
                    node_added = True
        return node_added

    # def connect_node(self, map, xy):

    def show_graph(self):
        pos = {x: list(x) for x in self.G.nodes}

        # nodes
        nx.draw_networkx_nodes(self.G, pos, node_size=700)

        # edges
        nx.draw_networkx_edges(self.G, pos,
                               width=6, alpha=0.5, edge_color='b', style='dashed')

        labels = dict((n, n) for n in self.G.nodes())

        # labels
        nx.draw_networkx_labels(self.G, pos, font_size=20,
                                font_family='sans-serif', labels=labels)

        plt.axis('off')
        plt.show()
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()

    def get_all_nodes(self):
        pos = [list(x) for x in self.G.nodes]
        is_door = [x[1]['is_door'] for x in self.G.nodes(data=True)]
        return np.array(pos), is_door

    def get_all_edges(self):
        return [e for e in self.G.edges]

    def nodes_are_eq(self, na, thresh=0.3):
        diff = np.array(list(self.G.nodes))-np.array(na)
        # np.sqrt(np.sum(np.square(diff), axis=1))
        rng = np.linalg.norm(diff, axis=1)
        return [n for n, r in zip(list(self.G.nodes), rng) if r < thresh], [r for n, r in zip(list(self.G.nodes), rng) if r < thresh]

    def get_path(self, src, trg):
        node_list = nx.dijkstra_path(self.G, src, trg)
        weights  = [self.G.get_edge_data(node_list[i],node_list[i+1])['weight'] for i in range(len(node_list)-1) ]
        return node_list, weights

if __name__ == "__main__":
    import net_db
    import pickle
    import networkx as nx
    from nav_msgs.msg import OccupancyGrid as og
    ogMsg = og()
    ogMsg.data = np.zeros((512, 512, 1))

    if True:
        with open('ndb.pickle', 'rb') as handle:
            ndb = pickle.load(handle)
    else:
        ndb = net_db.net_db()
        ndb.add_node((0, 0.1), False)
        ndb.add_node((-5, 0.1), False)
        ndb.add_node((2, 0.1), True)

        map_info = mapi.MapInfo(width=512, height=512, res=0.1, x=0, y=0, z=0)
        # map_data = np.reshape(np.zeros((512, 512, 1), dtype=np.uint8), (512*512,) )#  np.array([0]*512*512, dtype=np.uint8)
        map_info.set_map(ogMsg.data)
        ndb.add_node_map(map_info, (0.2, 5))
        ndb.show_graph()
    
    AAA = ndb.get_all_nodes()
    node_list, weights = ndb.get_path(tuple(AAA[0][1]), tuple(AAA[0][16]))
    edges  = [(node_list[i],node_list[i+1]) for i in range(len(node_list)-1) ]
    pos = {x: list(x) for x in ndb.G.nodes}
    
    nx.draw_networkx_nodes(ndb.G, pos, node_size=500)
    nx.draw_networkx_nodes(ndb.G, pos, nodelist=node_list, node_color='r', node_size=800)
    nx.draw_networkx_edges(ndb.G, pos,
                            width=6, alpha=0.5, edge_color='b', style='solid')
    nx.draw_networkx_edges(ndb.G, pos, edgelist=edges,
                            width=8, alpha=0.5, edge_color='r', style='solid')
    plt.show()
    

    print("end")
