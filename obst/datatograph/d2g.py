# convert a dataset to a graph of places,
# for which node distances can be determined.

import sys
import json
from collections import namedtuple
import math

from openrouteservice import Client
from openrouteservice.distance_matrix import distance_matrix
from openrouteservice.directions import directions

Graph = namedtuple("Graph", ["nodes", "edges", "distance_matrix"])
Node  = namedtuple("Node", ["id", "location", "metadata"])
Edge  = namedtuple("Edge", ["u", "v"])


path = "../../geojson/Sehenswürdigkeiten.geojson"
name = path.split("/")[-1].split(".")[0]

raw = json.load(open(path))["features"]

nodes = []
for feature in raw:
    location = feature["geometry"]["coordinates"]
    metadata = feature["properties"]
    ident = name + str(metadata["id"])
    node = Node(ident, location, metadata)
    nodes.append(node)

# Api Client with custom base URL
client = Client(base_url="http://localhost:8080/ors/")

def dist_mtrx(nodes):
    nodes = [node.location for node in nodes]
    route = client.distance_matrix(locations=nodes)  # remove list splice for big matrices
    dm = [list(map(lambda x: int(x/60), dur)) for dur in route["durations"]]
    return dm

def get_route(route):
    for coord_pair in zip(route, route[1:]):
        section = directions(coord_pair)
    # append sections to whole route

    return

def dist(u, v):
    return int(100*math.sqrt((u.location[0] - v.location[0])**2 + (u.location[1] - v.location[1])**2)) + 1

edges = [Edge(u, v) for u in nodes for v in nodes]
dm = dist_mtrx(nodes)
# distance_matrix = [[dist(u,v) for v in nodes] for u in nodes]
graph = Graph(nodes, edges, dm)
print (dm)
