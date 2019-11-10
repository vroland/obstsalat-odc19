# Copyright 2010-2018 Google LLC
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# [START program]
"""Simple travelling salesman problem between cities."""

# http://localhost:8000/13.737930691215155;51.049717431564794/10

# [START import]
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys
from http.server import *
import json
import math
import os

sys.path.append("../datatograph")
from d2g import load_graph
from d2g import client, directions

graphs = {}

def load_graphs():
    for root, dirs, files in os.walk("../../graphs/", topdown=False):
        for name in files:
            with open(os.path.join(root, name)) as f:
                source = f.read()
            graph = load_graph(source)
            graphs[graph.name] = graph
            print ("loaded", graph.name, "with", len(graph.nodes), "nodes!")

# [START data_model]
def create_data_model(graph, start):
    """Stores the data for the problem."""
    def dist(u):
        u = graph.nodes[u]
        return math.sqrt((start[0]-u.location[0])**2 + (start[1]-u.location[1])**2)

    start_node = sorted(list(range(len(graph.nodes))), key=lambda n: dist(n))[0]
    print ("start node:", start_node, graph.nodes[start_node])
    data = {}
    data['start_coords'] = start
    data['first_node_coords'] = graph.nodes[start_node].location
    data['graph'] = graph
    data['base_costs'] = graph.distance_matrix
    data['time_matrix'] = graph.distance_matrix
    #data['time_windows'] = [(i, 2000) for i, _ in enumerate(graph.nodes)]
    data['num_vehicles'] = 1
    data['depot'] = start_node
    return data
    # [END data_model]

def section_geometry(start, end):
    section = directions(client, (start, end), profile="foot-walking", geometry=True, format="geojson")
    return section["features"][0]["geometry"]["coordinates"]

def solution_to_json(data, manager, routing, assignment):
    """Prints assignment on console."""
    # Display dropped nodes.

    if not assignment:
        return json.dumps(None)

    dropped_nodes = []
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if assignment.Value(routing.NextVar(node)) == node:
            dropped_nodes.append(manager.IndexToNode(node))

    print('Dropped nodes:', dropped_nodes, "(", len(dropped_nodes), ")")
    # Display routes


    previous_index = routing.Start(0) # vehicle 0
    index = manager.IndexToNode(assignment.Value(routing.NextVar(previous_index)))
    route_output = [previous_index]

    while not routing.IsEnd(index):
        route_output.append(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
    route_output.append(manager.IndexToNode(index))

    print (route_output, "nodes:", len(data['graph'].nodes))
    print('Objective: {} miles'.format(assignment.ObjectiveValue()))


    route_time = 0
    answer = {}
    answer["nodes"] = { i : data['graph'].nodes[i] for i in route_output }
    answer["route"] = route_output
    answer["trips"] = {}
    round_trip_list = route_output + [data['depot']]
    answer["trips"]["start"] = {}
    answer["trips"]["start"]["time"] = 0
    answer["trips"]["start"]["waypoints"] = section_geometry(
        start=data['start_coords'],
        end=data['first_node_coords']
    )


    for i, n in enumerate(round_trip_list[:-2]):
        m = round_trip_list[i + 1]
        answer["trips"][n] = {}
        trip = answer["trips"][n]
        trip["time"] = data['time_matrix'][n][m]
        route_time += trip["time"]
        trip["waypoints"] = []
        geometry = section_geometry(
            start=data['graph'].nodes[n].location,
            end=data['graph'].nodes[m].location)
        trip["waypoints"].extend(geometry)
        # trip["waypoints"].append(data['graph'].nodes[n].location)
        # trip["waypoints"].append(data['graph'].nodes[m].location)

    answer["round_time"] = route_time
    print("round_time:", route_time)
    print(route_output)
    return json.dumps(answer, indent=4)


def find_route(graph, start, timeout):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(graph, start)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['base_costs']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    def base_cost_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['base_costs'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(base_cost_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    time_callback_index = routing.RegisterTransitCallback(time_callback)

    # Add Capacity constraint.
    #def time_callback(from_index):
    #    """Returns the demand of the node."""
    #    # Convert from routing variable Index to demands NodeIndex.
    #    from_node = manager.IndexToNode(from_index)
    # return data['time_matrix'][from_node]


    ######## Maximum number of stations ####################
    #time_callback_index = routing.RegisterTransitCallback(time_callback)
    #routing.AddDimensionWithVehicleCapacity(
    #    time_callback_index,
    #    0,  # null capacity slack
    #    [30],  # vehicle maximum capacities
    #    True,  # start cumul to zero
    #'Capacity')

    #for v_idx in range(data.num_vehicles):
    #    duration_dimension.CumulVar(routing.End(v_idx)).SetMax(25)


    routing.AddDimension(
        time_callback_index,
        0,  # null capacity slack
        timeout,  # vehicle maximum capacities
        True,  # start cumul to zero
        'MaxDistance'
    )

    #time_dimension = routing.GetDimensionOrDie('MaxDistance')

    #routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(0)))
    #routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(0)))

    # Allow to drop nodes.
    penalty = 100
    for node in range(len(data['graph'].nodes)):
        if node != data['depot']:
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    #search_parameters.time_limit.seconds = 10
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    #search_parameters.local_search_metaheuristic = (
    #    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
    #search_parameters.log_search = True

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    #print ("cumul len:", assignment.Max(time_dimension.CumulVar(routing.End(0))))
    return solution_to_json(data, manager, routing, assignment)



class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        try:
            _, graph_name, lonlat, timeout = self.path.split("/")
        except:
            print("invalid path:", self.path)
            self.send_response(404)
            self.wfile.write("invalid path".encode("utf-8"))
            return
        if not graph_name in graphs:
            self.send_response(404)
            self.wfile.write("invalid graph".encode("utf-8"))
            return

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        print ("path:", self.path)
        timeout = int(timeout)
        lonlat = list(map(float, lonlat.split(";")))
        graph = graphs[graph_name]
        print ("using timeout:", timeout)
        self.wfile.write(find_route(graph, lonlat, timeout).encode("utf-8"))

if __name__ == '__main__':

    load_graphs()

    def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler):
        server_address = ('', 8000)
        httpd = server_class(server_address, handler_class)
        httpd.serve_forever()

    run()
