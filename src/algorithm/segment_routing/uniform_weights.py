import time
from typing import List

from algorithm.generic_sr import GenericSR
from algorithm.segment_routing.equal_split_shortest_path import EqualSplitShortestPath


class UniformWeights(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None,
                 demand_priorities: List[bool] = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        self.__nodes = nodes  # [i, ...]
        self.__links = links  # [(i,j,c), ...]
        self.__demands = demands  # {idx: (s,t,d), ...}
        self.__weights = {(i, j): 1 for i, j, c in self.__links}
        self.__waypoints = waypoints
        self.__demand_priorities = demand_priorities if demand_priorities else [False for _ in demands]

    def solve(self) -> dict:
        """ set weights to inverse capacity and use shortest path algorithm """

        # add random waypoint for each demand
        t = time.process_time()
        pt_start = time.process_time()  # count process time (e.g. sleep excluded)

        post_processing = EqualSplitShortestPath(nodes=self.__nodes, links=self.__links, demands=self.__demands,
                                                 split=True, weights=self.__weights, waypoints=self.__waypoints,
                                                 demand_priorities=self.__demand_priorities)
        solution = post_processing.solve()

        pt_duration = time.process_time() - pt_start
        exe_time = time.process_time() - t

        # update execution time
        solution["execution_time"] = exe_time
        solution["process_time"] = pt_duration
        return solution

    def get_name(self):
        """ returns name of algorithm """
        return f"uniform_weights"
