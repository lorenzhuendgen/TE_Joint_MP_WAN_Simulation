from __future__ import annotations

import time
from typing import List, Tuple

from algorithm.generic_sr import GenericSR


class PriorityBased(GenericSR):
    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None,
                 demand_priorities: List[bool] = None, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)

        self.__nodes = nodes  # [i, ...]
        self.__links = links  # [(i,j,c), ...]
        self.__capacity_links = list(links)
        self.__weights = {(i, j): 1 for i, j, c in self.__links}
        self.__waypoints = waypoints
        self.__demand_priorities = demand_priorities if demand_priorities else [False for _ in demands]
        self.__all_demands = demands  # Only for debugging
        self.__normal_demands = [d for i, d in enumerate(demands) if not self.__demand_priorities[i]]  # {idx: (s,t,d), ...}
        self.__prio_demands = [d for i, d in enumerate(demands) if self.__demand_priorities[i]]  # {idx: (s,t,d), ...}
        self.__segments = {idx: [(p, q)] for idx, (p, q, _) in enumerate(demands)}
        self.__link_priorities = [False for _ in self.__links]
        self.__route_handler = RouteHandler(self.__capacity_links, self.__link_priorities)

    def solve(self) -> dict:
        """
        Routes the priority demands first and then the normal demand
        Tries to not overload links where priority demand is being routed
        """

        t_start = time.time()
        pt_start = time.process_time()

        self._route_priority_demands()
        self._route_normal_demands()

        pt_duration = time.process_time() - pt_start
        t_duration = time.time() - t_start

        utilization = self._calc_utilization()

        solution = {
            "objective": max(utilization.values()),
            "execution_time": t_duration,
            "process_time": pt_duration,
            "waypoints": self.__segments,
            "weights": self.__weights,
            "loads": utilization,
            "secondary_objective": self._calc_prio_mlu(),
        }

        print(f'* Prio MLU: {solution["secondary_objective"]}')

        return solution

    def _calc_prio_mlu(self) -> float:
        mlu = None
        for i, l in enumerate(self.__links):
            if self.__link_priorities[i]:
                cl = self.__capacity_links[i]
                lu = float((l[2] - cl[2]) / l[2])
                if mlu is None or lu > mlu:
                    mlu = lu

        return mlu

    def _calc_utilization(self) -> dict[Tuple[int, int], float]:
        utilization = dict()
        for i, l in enumerate(self.__links):
            cl = self.__capacity_links[i]
            lu = float((l[2] - cl[2]) / l[2])
            utilization[(l[0], l[1])] = lu

        return utilization

    def _route_normal_demands(self):
        for d in self.__normal_demands:
            pos_routes = self.__route_handler.get_routes(d[0], d[1])
            best_route = None
            best_cap_dif = None
            overloads_prio = False
            for r in pos_routes:
                rem_cap = r.calc_remaining_capacity()
                cap_dif = abs(rem_cap - d[2])
                if r.calc_prio_percentage() == 0:
                    if best_route is None or overloads_prio or cap_dif < best_cap_dif:
                        best_route = r
                        best_cap_dif = cap_dif
                else:
                    if rem_cap > d[2]:
                        if best_route is None or cap_dif < best_cap_dif:
                            best_route = r
                            best_cap_dif = cap_dif
                    else:
                        if best_route is None or (overloads_prio and cap_dif < best_cap_dif):
                            best_route = r
                            best_cap_dif = cap_dif
                            overloads_prio = True

            # Should only happen when there's NO route between src and dst
            if best_route is None:
                print(f'[!] No route between source and dest of {d}')
                input("Press ENTER to continue...")
                continue

            best_route.route_demand(d)

    def _route_priority_demands(self):
        for prd in self.__prio_demands:
            # Calculate routes with the highest priority link percentage and suitable capacity
            max_prio_percentage = 0
            prio_routes = []
            pos_routes = self.__route_handler.get_routes(prd[0], prd[1])
            for r in pos_routes:
                if r.calc_remaining_capacity() >= prd[2]:
                    prio_percentage = r.calc_prio_percentage()
                    if len(prio_routes) == 0 or prio_percentage > max_prio_percentage:
                        prio_routes = [r]
                    elif prio_percentage == max_prio_percentage:
                        prio_routes.append(r)

            # If no routes are found, abort
            if len(pos_routes) == 0:
                print(f"[!] Can't route demand {prd[0]} -> {prd[1]}")
                input("Press ENTER to continue...")
                continue

            # If no routes with enough capacity are found, we need to overload
            if len(prio_routes) == 0:
                min_link_utilization = None
                for r in pos_routes:
                    orig_capacity = PrioRoute.calc_max_capacity_for_links_by_indices(self.__links, r.route_link_ids)
                    new_capacity = (orig_capacity - r.calc_remaining_capacity()) + prd[2]
                    link_utilization = float(new_capacity / orig_capacity)
                    if min_link_utilization is None or link_utilization < min_link_utilization:
                        min_link_utilization = link_utilization
                        prio_routes = [r]

            # If multiple routes got same percentage, take the one with higher capacity
            route = prio_routes[0]
            capacity = prio_routes[0].calc_remaining_capacity()
            if len(prio_routes) > 1:
                for r in prio_routes:
                    c = r.calc_remaining_capacity()
                    if c > capacity:
                        route = r
                        capacity = c

            # Finally route the demand on this route
            route.route_demand(prd, priority=True)

    def get_name(self) -> str:
        """ returns name of algorithm """
        return f"priority_based"


class RouteHandler:

    def __init__(self, capacity_links: List[Tuple[int, int, float]], link_priorities: List[bool]):
        self.routes = dict()  # {(s,t): [PrioRoute, ...], ...}
        self.capacity_links = capacity_links
        self.link_priorities = link_priorities

    def get_routes(self, source: int, target: int) -> List[PrioRoute]:
        routes = None
        for st in self.routes.keys():
            if st[0] == source and st[1] == target:
                routes = self.routes[st]

        # Lazy calculating routes when needed
        if routes is None:
            routes = self.calc_routes(source, target)
            self.routes[(source, target)] = routes

        return routes

    def calc_routes(self, source: int, target: int) -> List[PrioRoute]:
        return self.calc_routes_rec(
            source,
            target,
            [],
            PrioRoute(source, target, self.link_priorities, self.capacity_links),
        )

    def calc_routes_rec(self, cur_node: int, target: int, found_routes: List[PrioRoute],
                        cur_route: PrioRoute) -> List[PrioRoute]:
        if cur_node == target:
            new_found_routes = list(found_routes)
            new_found_routes.append(cur_route)
            return new_found_routes

        for i, l in enumerate(self.capacity_links):
            if l[0] != l[1]:  # Forbid loops
                if l[0] == cur_node:
                    if l[1] not in cur_route.visited_nodes:
                        found_routes = self.calc_routes_rec(l[1], target, found_routes, cur_route.with_new_link(i))

        return found_routes


class PrioRoute:

    def __init__(self, source: int, target: int, link_priorities: List[bool],
                 capacity_links: List[Tuple[int, int, float]], route_link_ids: List[int] = None,
                 visited_nodes: set[int] = None):
        self.source = source
        self.target = target
        self.link_priorities = link_priorities
        self.capacity_links = capacity_links
        self.route_link_ids = route_link_ids if route_link_ids else []

        assert route_link_ids is None or (visited_nodes is not None), "Error PrioRoute: If route_link_ids is given, visited_nodes need to be set too!"

        self.visited_nodes = visited_nodes if route_link_ids else set()

    # Probably not needed
    def get_inverted(self) -> PrioRoute:
        return PrioRoute(
            self.target,
            self.source,
            self.link_priorities,
            self.capacity_links,
            self.route_link_ids.reverse(),
        )

    def with_new_link(self, new_link: int) -> PrioRoute:
        link = self.capacity_links[new_link]
        new_route_links = list(self.route_link_ids)
        new_route_links.append(new_link)
        new_visited_nodes = set(self.visited_nodes)
        new_visited_nodes.update([link[0], link[1]])

        return PrioRoute(
            self.source,
            self.target,
            self.link_priorities,
            self.capacity_links,
            route_link_ids=new_route_links,
            visited_nodes=new_visited_nodes,
        )

    def route_demand(self, prd: Tuple[int, int, float], priority: bool = False):
        for l in self.route_link_ids:
            link = self.capacity_links[l]
            self.capacity_links[l] = (link[0], link[1], link[2] - prd[2])
            self.link_priorities[l] = self.link_priorities[l] or priority

    def calc_prio_percentage(self) -> float:
        if len(self.route_link_ids) == 0:
            raise ValueError('Can not calculate prio percentage on empty links list!')

        all_links = 0
        prio_links = 0
        for l in self.route_link_ids:
            if self.link_priorities[l]:
                prio_links += 1
            all_links += 1

        return float(prio_links / all_links)

    def calc_remaining_capacity(self) -> float:
        return PrioRoute.calc_max_capacity_for_links_by_indices(self.capacity_links, self.route_link_ids)

    @staticmethod
    def calc_max_capacity_for_links_by_indices(links: List[Tuple[int, int, float]], indices: List[int]) -> float:
        min_c = None
        for l in indices:
            c = links[l][2]
            if min_c is None or c < min_c:
                min_c = c

        return min_c

    def __str__(self) -> str:
        s = ""
        for l in self.route_link_ids:
            s += f"-> {self.capacity_links[l][0]}-{self.capacity_links[l][1]} ({l})"

        s += f"\t\t==> [{self.calc_remaining_capacity()}, {self.calc_prio_percentage()}, {self.visited_nodes}]"

        return s
