""" Factory for segment routing algorithms"""
import random

from algorithm.generic_sr import GenericSR
from algorithm.segment_routing.demand_first_waypoints import DemandsFirstWaypoints
from algorithm.segment_routing.heur_ospf_weights import HeurOSPFWeights
from algorithm.segment_routing.inverse_capacity import InverseCapacity
from algorithm.segment_routing.priority_based import PriorityBased
from algorithm.segment_routing.segment_ilp import SegmentILP
from algorithm.segment_routing.sequential_combination import SequentialCombination
from algorithm.segment_routing.uniform_weights import UniformWeights
from algorithm.segment_routing.waypoints import Waypoints
from algorithm.segment_routing.priority_vp import PriorityVp
from algorithm.segment_routing.joint_waypoints import JointWaypoints
from algorithm.segment_routing.waypoint_congestion_control import WaypointCongestionControl

def get_algorithm(algorithm_name: str, nodes: list, links: list, demands: list, weights=None, waypoints=None,
                  seed: float = 42, ilp_method: str = None, time_out: int = None, sf: int = 100) -> GenericSR:
    priorities = []
    for i, d in enumerate(demands):
        random.seed(seed + i)
        priorities.append(random.randint(0, 19) == 0)

    algorithm_name = algorithm_name.lower()
    if algorithm_name == "demand_first_waypoints":
        algorithm = DemandsFirstWaypoints(nodes, links, demands, weights, waypoints)
    elif algorithm_name == "waypoints":
        algorithm = Waypoints(nodes, links, demands, weights, waypoints)
    elif algorithm_name == "heur_ospf_weights":
        algorithm = HeurOSPFWeights(nodes, links, demands, weights, waypoints, seed=seed, time_out=time_out)
    elif algorithm_name == "inverse_capacity":
        algorithm = InverseCapacity(nodes, links, demands, weights, waypoints, seed=seed, demand_priorities=priorities)
    elif algorithm_name == "segment_ilp":
        algorithm = SegmentILP(nodes, links, demands, weights, waypoints, waypoint_count=1, method=ilp_method,
                               splitting_factor=sf, time_out=time_out)
    elif algorithm_name == "sequential_combination":
        algorithm = SequentialCombination(nodes, links, demands, weights, waypoints, seed=seed, time_out=time_out,
                                          first_algorithm="heur_ospf_weights", second_algorithm="demand_first_waypoints")
    elif algorithm_name == "joint_waypoints":
        algorithm = JointWaypoints(nodes, links, demands, weights, waypoints, seed=seed, time_out=time_out,
                                          #first_algorithm="waypoints", second_algorithm="inverse_capacity")
                                          first_algorithm="inverse_capacity", second_algorithm="waypoints")
    elif algorithm_name == "uniform_weights":
        algorithm = UniformWeights(nodes, links, demands, weights, waypoints, seed=seed, demand_priorities=priorities)
    elif algorithm_name == "priority_based":
        """
        algorithm = PriorityBased(
            [0, 1, 2, 3],
            [(0, 1, 1000), (1, 3, 1000), (0, 2, 501), (2, 3, 501)],
            [(0, 3, 500), (0, 3, 500), (0, 3, 500), (0, 3, 500)],
            seed=seed,
            priorities=priorities,
        )
        """
        algorithm = PriorityBased(nodes, links, demands, weights, waypoints, seed=seed, demand_priorities=priorities)
    elif algorithm_name == "waypoint_congestion_control":
        algorithm = WaypointCongestionControl(nodes, links, demands, weights, waypoints, seed=seed)
    elif algorithm_name == "priority_vp":
        algorithm = PriorityVp(nodes, links, demands, weights, waypoints, demand_priorities = priorities, seed=seed)
    else:
        err_msg = f"algorithm not found: {algorithm_name}"
        raise Exception(err_msg)
    return algorithm