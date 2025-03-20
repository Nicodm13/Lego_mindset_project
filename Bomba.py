import networkx as nx
import matplotlib.pyplot as plt


class GraphWithCoordinates:
    def __init__(self):
        self.graph = nx.Graph()
        self.positions = {}

    def add_node(self, name, x, y):
        self.graph.add_node(name, pos=(x, y))
        self.positions[name] = (x, y)

    def add_edge(self, node1, node2):
        #if node1 in self.graph and node2 in self.graph:
       self.graph.add_edge(node1, node2)
        #else:
         #   raise ValueError("Both nodes must exist in the graph.")

    def draw_graph(self):
        plt.figure(figsize=(60, 60))
        nx.draw(self.graph, self.positions, with_labels=True, node_color='lightblue', edge_color='gray', node_size=700)
        plt.show()


# Example usage
graph = GraphWithCoordinates()

import random

# Generate 30 nodes with random coordinates
defined_nodes = {chr(65 + i): (random.randint(0, 10), random.randint(0, 10)) for i in range(800)}

# Add nodes using a loop
for name, (x, y) in defined_nodes.items():
    graph.add_node(name, x, y)

# Generate random edges between nodes
edges = [(random.choice(list(defined_nodes.keys())), random.choice(list(defined_nodes.keys()))) for _ in range(40)]

# Add edges using a loop
for node1, node2 in edges:
    if node1 != node2:  # Avoid self-loops
        graph.add_edge(node1, node2)


graph.draw_graph()
