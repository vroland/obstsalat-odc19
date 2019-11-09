# convert a dataset to a graph of places,
# for which node distances can be determined.

import sys
import json
from collections import namedtuple
import math

Graph = namedtuple("Graph", ["nodes", "edges", "distance_matrix"])
Node  = namedtuple("Node", ["id", "location", "metadata"])
Edge  = namedtuple("Edge", ["u", "v"])


path = "../../Dresden_EPSG_4326/Tourismus/Sehenswürdigkeiten.json"
name = path.split("/")[-1].split(".")[0]

raw = json.load(open(path))["features"]

nodes = []
for feature in raw:
    location = feature["geometry"]["coordinates"]
    metadata = feature["properties"]
    ident = name + str(metadata["id"])
    node = Node(ident, location, metadata)
    nodes.append(node)

def dist(u, v):
    return int(100*math.sqrt((u.location[0] - v.location[0])**2 + (u.location[1] - v.location[1])**2)) + 1

edges = [Edge(u, v) for u in nodes for v in nodes]
distance_matrix = [[dist(u,v) for v in nodes] for u in nodes]
graph = Graph(nodes, edges, distance_matrix)

