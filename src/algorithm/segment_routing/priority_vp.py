import time
import networkx as nx

from algorithm.generic_sr import GenericSR
from algorithm.segment_routing.sr_utility import SRUtility


class PriorityVp(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None,
                 demand_priorities : list = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        self.__nodes = nodes  # [i, ...]
        self.__links = links  # [(i,j,c), ...]
        self.__demands = demands  # {idx: (s,t,d), ...}
        self.__demand_priorities = [None] * len(demands)
        self.__weights = {(i, j): 1 for i, j, c in self.__links}
        self.__waypoints = waypoints


    def solve(self) -> dict:
        """ set weights to inverse capacity and use shortest path algorithm """
        for i in range(len(self.__demands)//2):
            self.__demand_priorities[i] = True
        # add random waypoint for each demand
        t = time.process_time()
        pt_start = time.process_time()  # count process time (e.g. sleep excluded)
        newDemands = []
        newBools = []
        for demand in self.__demands:
            index = self.__demands.index(demand)
            if self.__demand_priorities[index]:
                s, t, d = demand
                newDemands.append((s,t,d/2))
                newDemands.append((s,t,d/2))
                newBools.append(True)
                newBools.append(True)
            else:
                newDemands.append(demand)
                newBools.append(False)
        self.__demands = newDemands
        self.__demand_priorities = newBools
        post_processing = EqualSplitShortestPathDoubled(nodes=self.__nodes, links=self.__links, demands=self.__demands,
                                                 split=True, weights=self.__weights, waypoints=self.__waypoints, priorities = self.__demand_priorities)
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

class EqualSplitShortestPathDoubled(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None,
                 split: bool = True, priorities :list = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        self.__nodes = nodes
        self.__links = links  # list with [(i,j,c)]
        self.__linksWithoutC = [(i, j) for i, j, _ in self.__links]
        if waypoints is not None:
            segmented_demands = SRUtility.get_segmented_demands(waypoints, demands)
            self.__demands = {idx: (s, t, d) for idx, (s, t, d) in enumerate(segmented_demands)}  # dict {idx:(s,t,d)}
            self.__segments = waypoints  # dict with {idx:(p,q)}
        else:
            self.__demands = {idx: (s, t, d) for idx, (s, t, d) in enumerate(demands)}
            self.__segments = {idx: [(p, q)] for idx, (p, q, _) in enumerate(demands)}

        self.__weights = weights if weights else {(i, j): 1 for i, j, _ in links}
        self.__priorities = priorities

        self.__split = split
        self.__all_shortest_paths_generators = dict()
        self.__all_shortest_paths = dict()
        self.__nx_graph = nx.DiGraph()
        self.__flow_sum = dict()

        self.__create_nx_graph()
        self.__init_flow_sum_map()
        return

    def __create_nx_graph(self):
        for i, j, c in self.__links:
            w = int(self.__weights[i, j]) * 100  # '*100' to reduce computing errors
            self.__nx_graph.add_edge(i, j, weight=w, capacity=c)
        return

    def __get_all_shortest_paths_generator(self):
        for s in self.__nx_graph.nodes:
            for t in self.__nx_graph.nodes:
                if s == t:
                    continue
                self.__all_shortest_paths_generators[s, t] = nx.all_shortest_paths(
                    self.__nx_graph, source=s, target=t, weight='weight')
        return

    def __init_flow_sum_map(self):
        for i, j, _ in self.__links:
            self.__flow_sum[(i, j)] = 0
        return

    def __add_demand_val_to_path(self, path: list, demand: float):
        for idx in range(len(path) - 1):
            i = path[idx]
            j = path[idx + 1]

            self.__flow_sum[(i, j)] += demand
        return

    def __update_weight_used_path(self, path:list, newWeight: int):
        for idx in range(len(path)-1):
            i = path[idx]
            j = path[idx +1]
            x = self.__linksWithoutC.index((i,j))
            self.__weights[x] = newWeight
        return

    def __count_same_paths(self, path:list):
        c = 0
        n = 0
        z = len(self.__links)
        for idx in range(len(path)-1):
            i = path[idx]
            j = path[idx +1]
            x = self.__linksWithoutC.index((i,j))
            c = c + self.__weights[x]
        c= c-z
        while c>0:
            n=n+1
            c = c-z
        return n


    def __add_demand_update_objective(self, src, dst, demand):
        if (src, dst) not in self.__all_shortest_paths:
            self.__all_shortest_paths[src, dst] = list(self.__all_shortest_paths_generators[src, dst])

        if self.__split:
            n_splits = len(self.__all_shortest_paths[src, dst])
            split_demand = demand / n_splits
            for shortest_path in self.__all_shortest_paths[src, dst]:
                self.__add_demand_val_to_path(shortest_path, split_demand)
        else:
            # take first shortest path if multiple
            shortest_path = self.__all_shortest_paths[src, dst][0]
            self.__add_demand_val_to_path(shortest_path, demand)
        return



    def solve(self) -> dict:
        """
        Computes shortest path routes and determines max link utilization and the utilization map
        on fixed weight settings. This algorithms supports postprocessing of several
        algorithms e.g. segment_lp_gurobi_relax
        """

        t_start = time.time()  # sys wide time
        pt_start = time.process_time()  # count process time (e.g. sleep excluded)
        self.__get_all_shortest_paths_generator()
        i=0
        c=0
        pathsCounter = 0
        for idx in self.__demands:
            if not self.__priorities[i]:
                s, t, d = self.__demands[idx]
                self.__add_demand_update_objective(s, t, d)
                i = i+1
            else:
                if not c:
                    s, t, d = self.__demands[idx]
                    self.__add_demand_update_objective(s, t, d)
                    shortestPath = self.__all_shortest_paths[s,t][0]
                    self.__update_weight_used_path(shortestPath, len(self.__links)) #update weights used to sum of all weights
                    i = i + 1
                    c = 1
                else:
                    s, t, d = self.__demands[idx]
                    self.__add_demand_update_objective(s, t, d) #return sum of weights og links
                    shortestPath = self.__all_shortest_paths[s, t][0]
                    self.__update_weight_used_path(shortestPath,1)
                    pathsCounter = pathsCounter + self.__count_same_paths(shortestPath)
                    i = i + 1
                    c=0
        pt_duration = time.process_time() - pt_start
        t_duration = time.time() - t_start
        utilization = {(i, j): self.__flow_sum[i, j] / self.__nx_graph[i][j]["capacity"] for i, j, _ in
                       self.__links}
        average_Paths = pathsCounter/len(self.__priorities)
        #calculate average of same used links
        solution = {
            "objective": max(utilization.values()),
            #"objective": average_Paths,
            "execution_time": t_duration,
            "process_time": pt_duration,
            "waypoints": self.__segments,
            "weights": self.__weights,
            "loads": utilization,
        }
        return solution

    def get_name(self):
        """ returns name of algorithm """
        return f"equal_split_shortest_paths"