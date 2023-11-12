"""
    Derived from demand_first_waypoints.py and waypoint_multipath_optimized.py.
    Follows the same procedure as demand_first_waypoints, assigning each demand the ideal waypoint
    in regard to the mlu at the moment at first. After that, a better split fraction combination for all set waypoints
    is computed via global optimization.
    The global optimization is done using properties of the mlu and link utilization functions in n dimensions
    as shown in the thesis.
    Still uses ECMP.
    Link weights are as given or all set to 1.
"""

import time

import networkit as nk
import numpy
import numpy as np

from algorithm.generic_sr import GenericSR


class WaypointMultipathGlobalOptimization(GenericSR):
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

    def __find_optimal_wp_split(self, util_gradient_matrix, util_base_arr, start_flow_map, waypoint_list, sp_fraction_map):
        """  """
        link_count = util_base_arr.size
        demand_count = util_gradient_matrix[0].size
        current_split_arr = np.array([1]*demand_count)
        current_flow_map = start_flow_map
        for it in range(link_count):
            # find links with max
            link_util_arr = util_base_arr.copy()
            for link in range(util_base_arr.size):
                link_util_arr[link] += np.dot((current_split_arr - 1), util_gradient_matrix[link])
            max_util = np.max(link_util_arr)
            max_idx = []
            for link in range(util_base_arr.size):
                if link_util_arr[link] == max_util:
                    max_idx.append(link)
            # find direction of steep(est) descent
            gradient_arr = []
            # at borders the gradient potentially needs to be adjusted
            for idx in max_idx:
                link_gradient = util_gradient_matrix[idx]
                for demand in range(demand_count):
                    if current_split_arr[demand] == 1:
                        link_gradient[demand] = max(link_gradient[demand], 0)
                    if current_split_arr[demand] == 0:
                        link_gradient[demand] = min(link_gradient[demand], 0)
                gradient_arr.append(link_gradient)
            descent_opposite_direction = np.average(gradient_arr, axis=0)
            norm_descent = np.linalg.norm(descent_opposite_direction)
            if norm_descent == 0:
                return current_split_arr
            descent = np.min(np.dot(gradient_arr, descent_opposite_direction))/norm_descent
            for gradient in gradient_arr:
                norm_gradient = np.linalg.norm(gradient)
                if norm_gradient == 0:
                    continue
                test_descent = np.min(np.dot(gradient_arr, gradient))/norm_gradient
                if test_descent > descent:
                    descent_opposite_direction = np.array(gradient)
                    descent = test_descent
            if descent <= 0:
                return current_split_arr

            # find first intersection in the direction of descent
            distance_intersection_arr = []
            for demand in range(demand_count):
                if descent_opposite_direction[demand] > 0:
                    distance_intersection_arr.append(-current_split_arr[demand]/descent_opposite_direction[demand])
                if descent_opposite_direction[demand] < 0:
                    distance_intersection_arr.append((1-current_split_arr[demand])/descent_opposite_direction[demand])
            if len(distance_intersection_arr) == 0:
                return current_split_arr
            distance_to_intersection = np.max(distance_intersection_arr)
            intersection_split_arr = current_split_arr + distance_to_intersection*descent_opposite_direction

            # find ideal split between current split and intersection split
            split_diff_arr = intersection_split_arr - current_split_arr
            intersection_flow_map = current_flow_map.copy()
            for index in range(len(split_diff_arr)):
                d_id, wp = waypoint_list[index]
                s, t, d = self.__demands[d_id]
                intersection_flow_map = self.__update_flow_map(sp_fraction_map, intersection_flow_map,
                                                               s, t, d, wp, split_diff_arr[index])
            current_util_map, current_objective = self.__compute_utilization(current_flow_map)
            intersection_util_map, intersection_objective = self.__compute_utilization(intersection_flow_map)
            ideal_split = self.__get_ideal_split(current_util_map, intersection_util_map)
            split = ideal_split["split"]
            new_flow_map = (1-split)*current_flow_map + split*intersection_flow_map
            new_util_map, new_objective = self.__compute_utilization(new_flow_map)
            if new_objective >= current_objective:  # safety measure to ensure the objective goes down
                return current_split_arr

            current_flow_map = new_flow_map
            current_split_arr = (1-split)*current_split_arr + split*intersection_split_arr
        return current_split_arr

    def __demands_first_waypoints(self):
        """ main procedure """
        distances = self.__compute_distances()
        sp_fraction_map = self.__get_shortest_path_fraction_map(distances)
        best_flow_map = self.__get_flow_map(sp_fraction_map)
        best_util_map, best_objective = self.__compute_utilization(best_flow_map)
        current_best_flow_map = best_flow_map
        current_best_util_map = best_util_map
        current_best_objective = best_objective
        ideal_flow_map = best_flow_map.copy()

        waypoints = dict()
        waypoint_list = []
        waypoint_gradient_matrix_arr = []
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

                if objective < best_objective:
                    current_best_flow_map = flow_map
                    current_best_util_map = util_map
                    current_best_objective = objective
                    current_best_waypoint = waypoint

            if current_best_waypoint is not None:
                gradient_list = []
                for u, v in self.__links:
                    gradient_list.append(best_util_map[u][v]-current_best_util_map[u][v])
                waypoint_gradient_matrix_arr.append(gradient_list)
                waypoints[d_idx] = (s, t, current_best_waypoint, d, 1)
                waypoint_list.append((d_idx, current_best_waypoint))
            else:
                waypoints[d_idx] = (s, t, None, d, 0)
            best_flow_map = current_best_flow_map
            best_util_map = current_best_util_map
            best_objective = current_best_objective

        # start of global multipath optimization
        waypoint_gradient_matrix = np.array(waypoint_gradient_matrix_arr).transpose()
        util_list = []
        for u, v in self.__links:
            util_list.append(best_util_map[u][v])
        optimal_split = None
        if waypoint_gradient_matrix.size > 0:
            optimal_split = self.__find_optimal_wp_split(waypoint_gradient_matrix, np.array(util_list),
                                                         best_flow_map, waypoint_list, sp_fraction_map)

        if optimal_split is not None:
            for wp_idx in range(optimal_split.size):
                (d_idx, _) = waypoint_list[wp_idx]
                (s, t, wp, d, _) = waypoints[d_idx]
                ideal_flow_map = self.__update_flow_map(sp_fraction_map, ideal_flow_map, s, t, d, wp,
                                                        optimal_split[wp_idx])
            ideal_util_map, ideal_objective = self.__compute_utilization(ideal_flow_map)
            self.__loads = {(u, v): ideal_util_map[u][v] for u, v, in self.__links}
        else:
            ideal_util_map, ideal_objective = self.__compute_utilization(best_flow_map)
            self.__loads = {(u, v): ideal_util_map[u][v] for u, v, in self.__links}
        return self.__loads, waypoints, ideal_objective

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
        return f"waypoint_multipath_global_optimization"
