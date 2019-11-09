# convert a dataset to a graph of places,
# for which node distances can be determined.

import sys
import json
from collections import namedtuple

Graph = namedtuple("Graph", ["nodes", "edges"])
Node  = namedtuple("Node", ["id", "location", "metadata"])
Edge  = namedtuple("Edge", ["u", "v", "weight"])


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
    return u.location[0]**2 + u.location[1]**2

edges = [Edge(u, v, dist(u,v)) for u in nodes for v in nodes]
graph = Graph(nodes, edges)

