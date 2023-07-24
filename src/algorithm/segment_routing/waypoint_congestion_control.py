import time

import networkit as nk
import numpy as np

from algorithm.generic_sr import GenericSR


class WaypointCongestionControl(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        # topology info
        self.__capacities = self.__extract_capacity_dict(links)  # dict with {(u,v):c, ..}
        self.__links = list(self.__capacities.keys())  # list with [(u,v), ..]
        self.__n = len(nodes)
        self.__capacity_map = None

        # demand segmentation and aggregate to matrix
        # store the ids of demands still to route
        self.__demands = demands # {idx: (s,t,d), ...}
        self.__demand_ids_left = [i for i in range(len(self.__demands))]

        # initial weights
        self.__weights = weights if weights else {(u, v): 1. for u, v in self.__links}

        # networKit graph
        self.__g = None

        # more maps to be filled via distributing demands
        self.__flow_map = {link: 0 for link in self.__links}
        self.__free_capacity_map = {(u, v): self.__capacities[(u, v)] for u, v in self.__links} # cap_map - flow_map

        # manage potential waypoints and congestion map
        self.__sp_dict = {}  # (s, t) -> (path, cost)
        self.__waypoint_path_dict = {(s, t): None for s, t, _ in self.__demands}  # standard waypointless entries
        self.__waypoint_path_list_per_demand = {d_id: [] for d_id in
                                                self.__demand_ids_left}  # demand -> [(wp, [path_links], dist, attr)]
        self.__linkCongestionMap = None
        self.__potential_flow_map = {link: 0 for link in self.__links}

        self.__init_graph()
        self.__init_capacity_map()

        self.init_waypoint_path_list_per_demand()
        self.init_congestion_map()
        return

    @staticmethod
    def __extract_capacity_dict(links):
        """ Converts the list of link/capacities into a capacity dict (compatibility reasons)"""
        return {(u, v): c for u, v, c in links}

    def __init_capacity_map(self):
        self.__capacity_map = np.ones((self.__n, self.__n), np.float)
        self.__free_capacity_map = np.ones((self.__n, self.__n), np.float)
        for u, v in self.__links:
            self.__capacity_map[u][v] = self.__capacities[u, v]
            self.__free_capacity_map[u][v] = self.__capacities[u, v]

    def __init_graph(self):
        """ Create networKit graph, add weighted edges and create spsp (some pairs shortest path) object """
        self.__g = nk.Graph(weighted=True, directed=True, n=self.__n)
        for u, v in self.__links:
            self.__g.addEdge(u, v, self.__weights[u, v])

    @staticmethod
    def get_links_on_paths(path1, path2) -> []:
        # takes 2 paths (indexed structures, here tuples) and gives out the links on that path
        link_list = []
        if path1 is not None and len(path1) > 1:
            for i in range(len(path1) - 1):
                link_list.append((path1[i], path1[i + 1]))
        if path2 is not None and len(path2) > 1:
            for i in range(len(path2) - 1):
                link_list.append((path2[i], path2[i + 1]))
        return link_list

    def init_waypoint_path_list_per_demand(self):
        # builds a dict of demand -> [(wp_node, [links_on_path], distance, attractiveness_of_path)]
        waypoint_path_list_complete = {d_id: [] for d_id in self.__demand_ids_left}
        for d_id in self.__demand_ids_left:
            (s, t, d) = self.__demands[d_id]
            # first, fill in the direct routes with wp = None
            if (s, t) in self.__waypoint_path_dict and self.__waypoint_path_dict[(s, t)] is not None:
                self.__waypoint_path_list_per_demand[d_id].append(self.__waypoint_path_dict[(s, t)])
            elif (s, t) in self.__sp_dict and self.__sp_dict[(s, t)] is not None:
                sp = self.__sp_dict[(s, t)]
                self.__waypoint_path_dict[(s, t)] = (None, self.get_links_on_paths(sp[0], None), sp[1], None)
                self.__waypoint_path_list_per_demand[d_id].append(self.__waypoint_path_dict[(s, t)])
            else:
                dijkstra = nk.distance.Dijkstra(self.__g, s, True, False, t)
                dijkstra.run()
                path = dijkstra.getPath(t)
                cost = dijkstra.distance(t)
                self.__sp_dict[(s, t)] = (path, cost)
                self.__waypoint_path_dict[(s, t)] = (None, self.get_links_on_paths(path, None), cost, None)
                self.__waypoint_path_list_per_demand[d_id].append(self.__waypoint_path_dict[(s, t)])

            # now add all the routes with wps that qualify (wp not on sp)
            sp_cost = self.__sp_dict[(s, t)][1]
            nodes_on_sp = self.__sp_dict[(s, t)][0]
            for potential_wp_id in range(self.__n):
                if potential_wp_id in nodes_on_sp:
                    continue
                dijkstra1 = nk.distance.Dijkstra(self.__g, s, True, False, potential_wp_id)
                dijkstra1.run()
                path1 = dijkstra1.getPath(potential_wp_id)
                cost1 = dijkstra1.distance(potential_wp_id)
                dijkstra2 = nk.distance.Dijkstra(self.__g, potential_wp_id, True, False, t)
                dijkstra2.run()
                path2 = dijkstra2.getPath(t)
                cost2 = dijkstra2.distance(t)
                self.__waypoint_path_list_per_demand[d_id].append((
                    potential_wp_id,
                    self.get_links_on_paths(path1, path2),
                    cost1 + cost2,
                    None
                ))

            # build the list that includes a score for this wp for the demand (sum of all scores for a demand = 1)
            inverse_weight_sum = 0
            for pot_path in self.__waypoint_path_list_per_demand[d_id]:
                inverse_weight_sum += 1 / pot_path[2]
            for pot_path in self.__waypoint_path_list_per_demand[d_id]:
                (wp, ls, cost, attr) = pot_path
                waypoint_path_list_complete[d_id].append((wp, ls, cost, 1 / (cost * inverse_weight_sum)))

        self.__waypoint_path_list_per_demand = waypoint_path_list_complete
        return

    def init_congestion_map(self):
        # builds a dict of link -> congestion_factor (how much demand there is for this link, 1 = probably 100% util.)
        for d_id in self.__demand_ids_left:
            potential_paths = self.__waypoint_path_list_per_demand[d_id]
            for wp_path in potential_paths:
                for link in wp_path[1]:
                    self.__potential_flow_map[link] += wp_path[3] * self.__demands[d_id][2]
        self.__linkCongestionMap = {link: (1000000 if (self.__free_capacity_map[link] <= 0) else
                                           (self.__potential_flow_map[link] / self.__free_capacity_map[link]))
                                    for link in self.__links}

    def update_congestion_map(self, demand_id):
        # updates the congestion_map by removing the part of the removed demand
        for wp_path in self.__waypoint_path_list_per_demand[demand_id]:
            for link in wp_path[1]:
                self.__potential_flow_map[link] -= wp_path[3] * self.__demands[demand_id][2]
        self.__linkCongestionMap = {link: (1000000 if (self.__free_capacity_map[link] <= 0) else
                                           (self.__potential_flow_map[link] / self.__free_capacity_map[link]))
                                    for link in self.__links}

    def get_congestion_factor(self, wp_path):
        # calculates the score of the path, lower is better / less congested path
        congestion_factor = 0
        for link in wp_path[1]:
            congestion_factor += self.__linkCongestionMap[link]
        return congestion_factor

    def solve(self) -> dict:
        t_start = time.process_time()
        pt_start = time.process_time()  # count process time (e.g. sleep excluded)

        waypoints = dict()

        # distribute the best demand as long as there are demands left to distribute
        # update stuff in between
        while len(self.__demand_ids_left) > 0:
            d_id0 = self.__demand_ids_left[0]
            path0 = self.__waypoint_path_list_per_demand[d_id0][0]
            best_choice = (self.get_congestion_factor(path0), d_id0, path0)
            for d_id in self.__demand_ids_left:
                for path in self.__waypoint_path_list_per_demand[d_id]:
                    congestion_factor = self.get_congestion_factor(path)
                    if congestion_factor < best_choice[0]:
                        best_choice = (congestion_factor, d_id, path)

            d_id = best_choice[1]
            (s, t, d) = self.__demands[d_id]
            waypoint = best_choice[2][0]
            waypoints[d_id] = [(s, t)] if waypoint is None else [(s, waypoint), (waypoint, t)]
            self.update_congestion_map(d_id)
            self.__demand_ids_left.remove(d_id)
            self.__waypoint_path_list_per_demand.pop(d_id)
            flow = self.__demands[d_id][2]
            for link in best_choice[2][1]:
                self.__free_capacity_map[link] -= flow
                self.__flow_map[link] += flow

        overload_objective = 100*sum({(i, j): max([self.__flow_map[(i, j)] - self.__capacity_map[(i, j)], 0])
                                      for i, j in self.__links}.values()) / sum({(i, j): self.__capacity_map[(i, j)]
                                                                                 for i, j in self.__links}.values())
        utilization = {(i, j): self.__flow_map[(i, j)] / self.__capacities[(i, j)] for i, j in
                       self.__links}

        pt_duration = time.process_time() - pt_start
        t_duration = time.time() - t_start

        solution = {
            "objective_overload": overload_objective,
            "objective": max(utilization.values()),
            "execution_time": t_duration,
            "process_time": pt_duration,
            "waypoints": waypoints,
            "weights": self.__weights,
            "loads": None,
        }

        pt_duration = time.process_time() - pt_start
        exe_time = time.process_time() - t

        # update execution time
        solution["execution_time"] = exe_time
        solution["process_time"] = pt_duration
        return solution

    def get_name(self):
        """ returns name of algorithm """
        return f"minimize"
