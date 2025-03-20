import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

G = nx.Graph()
V = (1,2)
G.add_edges_from([V,(2,3),(1,3),(2,4)])
labelmap = dict(zip(G.nodes(), ["A", "B", "C", "D"]))
nx.draw(G, labels=labelmap, with_labels=True)
plt.show()