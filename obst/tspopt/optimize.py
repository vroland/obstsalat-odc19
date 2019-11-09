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

sys.path.append("../datatograph")
from d2g import graph

# [END import]


# [START data_model]
def create_data_model(start):
    """Stores the data for the problem."""
    def dist(u):
        u = graph.nodes[u]
        return math.sqrt((start[0]-u.location[0])**2 + (start[1]-u.location[1])**2)

    start_node = sorted(list(range(len(graph.nodes))), key=lambda n: dist(n))[0]
    print ("start node:", start_node, graph.nodes[start_node])
    data = {}
    data['graph'] = graph
    data['base_costs'] = [[0 for u in graph.nodes] for v in graph.nodes]
    data['time_matrix'] = graph.distance_matrix
    #data['time_windows'] = [(i, 2000) for i, _ in enumerate(graph.nodes)]
    data['num_vehicles'] = 1
    data['depot'] = start_node
    return data
    # [END data_model]


def solution_to_json(data, manager, routing, assignment):
    """Prints assignment on console."""
    # Display dropped nodes.
    dropped_nodes = []
    for node in range(routing.Size()):
        if routing.IsStart(node) or routing.IsEnd(node):
            continue
        if assignment.Value(routing.NextVar(node)) == node:
            dropped_nodes.append(manager.IndexToNode(node))

    print('Dropped nodes:', dropped_nodes, "(", len(dropped_nodes), ")")
    # Display routes

    index = routing.Start(0) # vehicle 0
    route_time = 0
    route_output = []
    previous_index = data['depot']
    while not routing.IsEnd(index):
        node_index = manager.IndexToNode(index)
        #route_load += data['demands'][node_index]
        route_output.append(node_index) #= ' {0} Load({1}) -> '.format(node_index, route_load)
        route_time += data['time_matrix'][previous_index][node_index]
        previous_index = node_index
        index = assignment.Value(routing.NextVar(index))

    answer = {}
    answer["round_time"] = route_time
    answer["nodes"] = { i : data['graph'].nodes[i] for i in route_output }
    answer["route"] = route_output
    answer["trips"] = {}
    round_trip_list = route_output + [data['depot']]
    for i, n in enumerate(round_trip_list[:-1]):
        m = round_trip_list[i + 1]
        answer["trips"][n] = {}
        trip = answer["trips"][n]
        trip["time"] = data['time_matrix'][n][m]
        trip["waypoints"] = []
        trip["waypoints"].append(data['graph'].nodes[n].location)
        trip["waypoints"].append(data['graph'].nodes[m].location)
    print("round_time:", route_time)
    print(route_output)
    return json.dumps(answer, indent=4)


def find_route(start, timeout):
    """Solve the CVRP problem."""
    # Instantiate the data problem.
    data = create_data_model(start)

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

    # Allow to drop nodes.
    penalty = 10000
    for node in range(0, len(data['time_matrix'])):
        if node != data['depot']:
            routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    return solution_to_json(data, manager, routing, assignment)



class SimpleHTTPRequestHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        try:
            _, lonlat, timeout = self.path.split("/")
        except:
            print("invalid path:", self.path)
            self.send_response(404)
            return

        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        print ("path:", self.path)
        timeout = int(timeout)
        lonlat = list(map(float, lonlat.split(";")))
        print ("using timeout:", timeout)
        self.wfile.write(find_route(lonlat, timeout).encode("utf-8"))

if __name__ == '__main__':

    def run(server_class=HTTPServer, handler_class=SimpleHTTPRequestHandler):
        server_address = ('', 8000)
        httpd = server_class(server_address, handler_class)
        httpd.serve_forever()

    run()
