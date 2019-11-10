# convert a dataset to a graph of places,
# for which node distances can be determined.

import sys
import json
from collections import namedtuple
import math

from openrouteservice import Client
from openrouteservice.distance_matrix import distance_matrix
from openrouteservice.directions import directions

# Api Client with custom base URL
client = Client(base_url="http://localhost:8080/ors/")

Graph = namedtuple("Graph", ["nodes", "distance_matrix", "name"])
Node  = namedtuple("Node", ["id", "location", "metadata"])

graphs_dir = "../../graphs/"

def file_to_graph(path):
    name = path.split("/")[-1].split(".")[0]

    raw = json.load(open(path))["features"]

    nodes = []
    for feature in raw:
        location = feature["geometry"]["coordinates"]
        metadata = feature["properties"]
        ident = name + str(metadata["id"])
        node = Node(ident, location, metadata)
        nodes.append(node)

    def dist_mtrx(nodes):
        nodes = [node.location for node in nodes]
        route = client.distance_matrix(locations=nodes, profile="foot-walking")  # remove list splice for big matrices
        dm = [list(map(lambda x: max(1, int(x/60)), dur)) for dur in route["durations"]]
        return dm

    def get_route(route):
        for coord_pair in zip(route, route[1:]):
            section = directions(coord_pair)
        # append sections to whole route

        return

    def dist(u, v):
        return int(100*math.sqrt((u.location[0] - v.location[0])**2 + (u.location[1] - v.location[1])**2)) + 1

    dm = dist_mtrx(nodes)
    # distance_matrix = [[dist(u,v) for v in nodes] for u in nodes]
    graph = Graph(nodes, dm, name)
    print ("loaded", path, "(", name, ")", "with", len(graph.nodes), "nodes!")
    return graph


def load_graph(raw):
    obj = json.loads(raw)
    nodes = [Node(*n) for n in obj[0]]
    graph = Graph(nodes, *obj[1:])
    return graph

paths = [
        "../../geojson/bereinigte_Toiletten.geojson",
        "../../geojson/Zukunftsstadt.geojson",
        "../../geojson/sehenswuerdigkeiten.geojson"
]

if __name__ == "__main__":
    for path in paths:
        graph = file_to_graph(path)
        with open(graphs_dir + graph.name + ".json", "w") as f:
            f.write(json.dumps(graph))
