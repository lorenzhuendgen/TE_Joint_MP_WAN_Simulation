"""
    Derived from demand_first_waypoints.py and waypoint_multipath_optimized.py.
    Follows the same procedure as demand_first_waypoints, assigning each demand the ideal waypoint
    in regard to the mlu at the moment at first. After that, the locally ideal (computed at the moment) split fraction
    for every assigned waypoint is computed, potentially reducing the amount routed via the waypoint from 1.0.
    The ideal split fraction is computed using properties of the mlu and link utilization functions
    as shown in the thesis.
    Still uses ECMP.
    Link weights are as given or all set to 1.
"""

import time

import networkit as nk
import numpy as np

from algorithm.generic_sr import GenericSR


class WaypointMultipathTwoPhaseOptimization(GenericSR):
    BIG_M = 10 ** 9

    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        # topology info
        self.__capacities = self.__extract_capacity_dict(links)  # dict with {(u,v):c, ..}
        self.__links = list(self.__capacities.keys())  # list with [(u,v), ..]
        self.__n = len(nodes)
        self.__capacity_map = None

        # demand segmentation and aggregate to matrix
        # store all target nodes for Some pairs shortest path algorithm
        self.__demands = demands

        # initial weights
        self.__weights = weights if weights else {(u, v): 1. for u, v in self.__links}

        # networKit graph and some pairs shortest path (SPSP) algorithm
        self.__g = None
        self.__apsp = None

        self.__init_graph()
        self.__init_capacity_map()
        return

    @staticmethod
    def __extract_capacity_dict(links):
        """ Converts the list of link/capacities into a capacity dict (compatibility reasons)"""
        return {(u, v): c for u, v, c in links}

    def __init_capacity_map(self):
        self.__capacity_map = np.ones((self.__n, self.__n), np.float)
        for u, v in self.__links:
            self.__capacity_map[u][v] = self.__capacities[u, v]

    def __init_graph(self):
        """ Create networKit graph, add weighted edges and create spsp (some pairs shortest path) object """
        self.__g = nk.Graph(weighted=True, directed=True, n=self.__n)
        for u, v in self.__links:
            self.__g.addEdge(u, v, self.__weights[u, v])
        self.__apsp = nk.distance.APSP(self.__g)

    def __compute_distances(self):
        """ Recomputes the shortest path for 'some' pairs """
        self.__apsp.run()
        return self.__apsp.getDistances()

    def __get_shortest_path_fraction_map(self, distances):
        link_fraction_map = np.zeros((self.__n, self.__n, self.__n, self.__n), np.float)

        for s in range(self.__n):
            # iterate over nodes sorted by distance
            u_map = dict(zip(range(self.__n), np.array(distances[s]).argsort()))

            for t in range(self.__n):
                if s == t:
                    continue
                node_fractions = np.zeros(self.__n, np.float)
                node_fractions[s] = 1

                for u_idx in range(self.__n - 1):
                    u = u_map[u_idx]
                    fraction = node_fractions[u]
                    if not fraction:
                        continue

                    successors = list(v for v in self.__g.iterNeighbors(u) if
                                      self.__weights[(u, v)] == distances[u][t] - distances[v][t])

                    new_fraction = fraction / len(successors) if len(successors) != 0 else fraction
                    for v in successors:
                        link_fraction_map[s][t][u][v] = new_fraction
                        node_fractions[v] += new_fraction if v != t else 0.
        return link_fraction_map

    def __get_flow_map(self, sp_fraction_map):
        flow_map = np.zeros((self.__n, self.__n), np.float)
        for s, t, d in self.__demands:
            flow_map += sp_fraction_map[s][t] * d
        return flow_map

    def __compute_utilization(self, flow_map):
        util_map = (flow_map / self.__capacity_map)
        objective = np.max(util_map)
        return util_map, objective

    def __update_flow_map(self, sp_fraction_map, flow_map, s, t, d, waypoint, multipath_split=1.0):
        """ If multipath_split is <1, only that portion of the traffic is rerouted via the waypoint """
        new_flow_map = flow_map - sp_fraction_map[s][t] * (d*multipath_split)
        new_flow_map += sp_fraction_map[s][waypoint] * (d*multipath_split)
        new_flow_map += sp_fraction_map[waypoint][t] * (d*multipath_split)
        return new_flow_map

    def __filter_obsolete_links(self, new_util_arr, marker_max_new_util):
        """ Filters out the data for links, that cant be intersected with anymore. """
        filter_arr = []
        for new_util in new_util_arr:
            if new_util <= marker_max_new_util:
                filter_arr.append(False)
            else:
                filter_arr.append(True)
        return filter_arr

    def __get_ideal_split(self, current_util_map, new_util_map):
        """
            Finds the ideal split fraction for the routing that results in the new_util_map with the current_util_map
            serving as the starting point (split = 0).
            Iterates over the intersections of link utilization functions until the global minimum of the mlu is found.
        """
        split = dict()
        current_link_util = []
        new_link_util = []
        marker_max_current_util = 0
        marker_max_new_util = 0
        split_marker = 0
        for (u, v) in self.__links:
            current_util = current_util_map[u][v]
            new_util = new_util_map[u][v]
            current_link_util.append(current_util)
            new_link_util.append(new_util)
            if current_util > marker_max_current_util:
                marker_max_current_util = current_util
                marker_max_new_util = new_util
            elif current_util == marker_max_current_util and new_util > marker_max_new_util:
                marker_max_new_util = new_util
        current_util_arr = np.array(current_link_util)
        new_util_arr = np.array(new_link_util)

        filter_arr = self.__filter_obsolete_links(new_util_arr, marker_max_new_util)
        current_util_arr = current_util_arr[filter_arr]
        new_util_arr = new_util_arr[filter_arr]
        util_change_arr = new_util_arr - current_util_arr

        while split_marker < 1 and marker_max_current_util > marker_max_new_util and current_util_arr.size > 0:
            intersections = (marker_max_current_util - current_util_arr)/(new_util_arr - marker_max_new_util)
            min_intersection = 1
            index = 0
            temp_marker_max_new_util = 0
            for intersection in intersections:
                if intersection < min_intersection:
                    min_intersection = intersection
                    marker_max_current_util = current_util_arr[index]
                    temp_marker_max_new_util = new_util_arr[index]
                elif intersection == min_intersection and new_util_arr[index] > temp_marker_max_new_util:
                    marker_max_current_util = current_util_arr[index]
                    temp_marker_max_new_util = new_util_arr[index]
                index += 1
            if temp_marker_max_new_util > 0:
                marker_max_new_util = temp_marker_max_new_util
            if min_intersection < 1:
                filter_arr = self.__filter_obsolete_links(new_util_arr, marker_max_new_util)
                current_util_arr = current_util_arr[filter_arr]
                new_util_arr = new_util_arr[filter_arr]
                util_change_arr = util_change_arr[filter_arr]
            split_marker = min_intersection

        if marker_max_current_util > marker_max_new_util:
            split_marker = 1
            marker_max_current_util = marker_max_new_util
        split["split"] = split_marker
        split["objective"] = marker_max_current_util
        return split

    def __demands_first_waypoints(self):
        """ main procedure """
        distances = self.__compute_distances()
        sp_fraction_map = self.__get_shortest_path_fraction_map(distances)
        best_flow_map = self.__get_flow_map(sp_fraction_map)
        best_util_map, best_objective = self.__compute_utilization(best_flow_map)
        current_best_flow_map = best_flow_map
        current_best_util_map = best_util_map
        current_best_objective = best_objective
        waypoint_list = []

        waypoints = dict()
        sorted_demand_idx_map = dict(zip(range(len(self.__demands)), np.array(self.__demands)[:, 2].argsort()[::-1]))
        for d_map_idx in range(len(self.__demands)):
            d_idx = sorted_demand_idx_map[d_map_idx]
            s, t, d = self.__demands[d_idx]
            current_best_waypoint = None
            for waypoint in range(self.__n):
                if waypoint == s or waypoint == t:
                    continue
                flow_map = self.__update_flow_map(sp_fraction_map, best_flow_map, s, t, d, waypoint)
                util_map, objective = self.__compute_utilization(flow_map)

                if objective < current_best_objective:
                    current_best_flow_map = flow_map
                    current_best_util_map = util_map
                    current_best_objective = objective
                    current_best_waypoint = waypoint

            if current_best_waypoint is not None:
                waypoints[d_idx] = ([(s, current_best_waypoint), (current_best_waypoint, t)], 1)
                waypoint_list.append((d_idx, current_best_waypoint))
            else:
                waypoints[d_idx] = ([(s, t)], 0)
            best_flow_map = current_best_flow_map
            best_util_map = current_best_util_map
            best_objective = current_best_objective

        # second optimization phase, reevaluating split fractions
        for (d_idx, waypoint) in waypoint_list:
            s, t, d = self.__demands[d_idx]
            test_flow_map = self.__update_flow_map(sp_fraction_map, best_flow_map, s, t, d, waypoint, -1)
            test_util_map, test_objective = self.__compute_utilization(test_flow_map)

            split = self.__get_ideal_split(test_util_map, best_util_map)
            split_flow_map = self.__update_flow_map(sp_fraction_map, best_flow_map, s, t, d, waypoint, split["split"]-1)
            split_util_map, split_objective = self.__compute_utilization(split_flow_map)

            if split_objective < best_objective:
                best_flow_map = split_flow_map
                best_util_map = split_util_map
                best_objective = split_objective
                waypoints[d_idx] = ([(s, waypoint), (waypoint, t)], split["split"])

        self.__loads = {(u, v): best_util_map[u][v] for u, v, in self.__links}
        return self.__loads, waypoints, best_objective

    def solve(self) -> dict:
        """ compute solution """

        self.__start_time = t_start = time.time()  # sys wide time
        pt_start = time.process_time()  # count process time (e.g. sleep excluded and count per core)
        loads, waypoints, objective = self.__demands_first_waypoints()
        pt_duration = time.process_time() - pt_start
        t_duration = time.time() - t_start

        solution = {
            "objective": objective,
            "execution_time": t_duration,
            "process_time": pt_duration,
            "waypoints": waypoints,
            "weights": self.__weights,
            "loads": loads,
        }

        return solution

    def get_name(self):
        """ returns name of algorithm """
        return f"waypoint_multipath_two_phase_optimization"
