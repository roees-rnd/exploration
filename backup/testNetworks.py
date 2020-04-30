import matplotlib.pyplot as plt

import networkx as nx


if True:
    G = nx.Graph()
    if False:
        G.add_edge('A', 'B', weight=4)
        G.add_edge('B', 'D', weight=2)
        G.add_edge('A', 'C', weight=3)
        G.add_edge('C', 'D', weight=4)
        print(nx.shortest_path(G, 'A', 'D', weight='weight'))

        attrs = {'A': {'isDoor': False}}
        nx.set_node_attributes(G, attrs)
        print("Node A isDoor attribute = " + str(G.nodes['A']['isDoor']))

    else:
        A = (1.2, 3)
        B = (3.5, 6)
        C = (7.8, 2.1)
        D = (5, -2)
        # G.add_edge(nodes['A']['xy'], nodes['B']['xy'], weight=4)
        G.add_edge(A, B, weight=4)
        G.add_edge(B, D, weight=2)
        G.add_edge(A, C, weight=3)
        G.add_edge(C, D, weight=4, attr1=False)
        print(nx.shortest_path(G, A, D, weight='weight'))
        # G.get_edge_data(A,B)
        # G.update(edges=)

        attrs = {A: {'isDoor': False}}
        nx.set_node_attributes(G, attrs)
        print("Node A isDoor attribute = " + str(G.nodes[A]['isDoor']))

    # G.add_node()
else:
    G = nx.cubical_graph()
    pos = nx.spring_layout(G)
    print(nx.shortest_path(G, 1, 5))


labels = nx.get_node_attributes(G, 'isDoor')
nx.draw(G)
# nx.draw(G, pos=nx.spring_layout(G))
plt.draw()
plt.show()
