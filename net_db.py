import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import MapInfo as mapi
import LOS


class net_db:

    def __init__(self):
        self.G = nx.Graph()
        self.last_node = None
        self.num_nodes = 0
        self._min_dist = 0.7
        self.r = np.array([])
        self.map_info = None

    def add_node(self, xy=(0, 0), is_door=False):
        if self.num_nodes == 0:  # no nodes yet
            self.G.add_node(xy)
        else:
            # Distance from last node:
            weight = np.sqrt(
                np.square(xy[0]-self.last_node[0])+np.square(xy[1]-self.last_node[1]))
            # If we are far enough from last node:
            if weight > self._min_dist:
                # If current position is far enough from all other nodes:
                nae = self.nodes_are_eq(xy, self._min_dist/2)
                if len(nae) < 1:  # no close nodes
                    if self.map_info is None:
                        self.G.add_node(xy)
                        self.G.add_edge(xy, self.last_node, weight=weight)
                    else:
                        self.add_node_map(self.map_info, xy)

                else:
                    if not self.G.has_edge(self.last_node, nae[0]):
                        w = np.sqrt(
                            np.square(nae[0][0]-self.last_node[0])+np.square(nae[0][1]-self.last_node[1]))
                        self.G.add_edge(self.last_node, nae[0], weight=w)
                    self.last_node = nae[0]
                    return False
            else:
                return False

        attr = {xy: {'is_door': is_door}}
        self.G.add_node(xy)
        nx.set_node_attributes(self.G, attr)

        self.last_node = xy
        self.num_nodes += 1
        return True

    def add_node_map(self, map_info, xy):
        node_added = False
        new_ij = map_info.xy_to_ij(xy[0], xy[1])
        for n in list(self.G.nodes()):
            n_ij = map_info.xy_to_ij(n[0], n[1])
            if LOS.is_LOS(new_ij[0], new_ij[1], n_ij[0], n_ij[1], np.abs(map_info.map)>0):
                w = np.sqrt(np.square(n[0]-xy[0])+np.square(n[1]-xy[1]))
                self.G.add_edge(xy, n, weight=w)
                if node_added is False:
                    nx.set_node_attributes(self.G, {xy: {'is_door': False}})
                    node_added = True
                    self.num_nodes += 1

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
        return [n for n, r in zip(list(self.G.nodes), rng) if r < thresh]


if __name__ == "__main__":
    import net_db
    from nav_msgs.msg import OccupancyGrid as og
    ogMsg = og()
    ogMsg.data = np.zeros((512, 512, 1))
    ndb = net_db.net_db()
    ndb.add_node((0, 0.1), False)
    ndb.add_node((-5, 0.1), False)
    ndb.add_node((2, 0.1), True)

    map_info = mapi.MapInfo(width=512, height=512, res=0.1, x=0, y=0, z=0)
    # map_data = np.reshape(np.zeros((512, 512, 1), dtype=np.uint8), (512*512,) )#  np.array([0]*512*512, dtype=np.uint8)
    map_info.set_map(ogMsg.data)
    ndb.add_node_map(map_info, (0.2, 5))
    ndb.show_graph()
    print("end")

# if True:
#     G = nx.Graph()
#     if False:
#         G.add_edge('A', 'B', weight=4)
#         G.add_edge('B', 'D', weight=2)
#         G.add_edge('A', 'C', weight=3)
#         G.add_edge('C', 'D', weight=4)
#         print(nx.shortest_path(G, 'A', 'D', weight='weight'))

#         attrs = {'A': {'isDoor': False}}
#         nx.set_node_attributes(G, attrs)
#         print("Node A isDoor attribute = " + str(G.nodes['A']['isDoor']))

#     else:
#         A = (1.2, 3)
#         B = (3.5, 6)
#         C = (7.8, 2.1)
#         D = (5, -2)
#         G.add_edge(nodes['A']['xy'], nodes['B']['xy'], weight=4)
#         G.add_edge(B, D, weight=2)
#         G.add_edge(nodes['A']['xy'], C, weight=3)
#         G.add_edge(C, D, weight=4, attr1=False)
#         print(nx.shortest_path(G, A, D, weight='weight'))
#         # G.get_edge_data(A,B)
#         # G.update(edges=)

#         attrs = {A: {'isDoor': False}}
#         nx.set_node_attributes(G, attrs)
#         print("Node A isDoor attribute = " + str(G.nodes[A]['isDoor']))

#     # G.add_node()
# else:
#     G = nx.cubical_graph()
#     pos = nx.spring_layout(G)
#     print(nx.shortest_path(G, 1, 5))


# labels = nx.get_node_attributes(G, 'isDoor')
# nx.draw(G)
# # nx.draw(G, pos=nx.spring_layout(G))
# plt.draw()
# plt.show()
