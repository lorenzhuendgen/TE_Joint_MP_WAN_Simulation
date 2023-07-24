"""factory for demand providers"""

from demand.generic_demand_provider import GenericDemandProvider
from demand.max_flow_lp.maximal_multi_commodity_flow_dp import McfDP
from demand.snd_lib.sndlib_demand_provider import SndLibTM
from demand.nanonet.nanonet_demand_provider import NanoNetDP


def get_demand_provider(
        provider: str, number_samples: int = 1, active_pairs_fraction=0.5, n: int = 0, seed=0, fixed_total: float = 100,
        flows_per_pair: int = 1, topology_name: str = None, links: list = None, unscaled_demands_sets: list = None,
        mcf_method: str = "MAXIMAL") -> GenericDemandProvider:
    provider = provider.lower()
    if provider == "snd_lib":
        return SndLibTM(topology_name=topology_name, number_samples=number_samples, fixed_total=fixed_total,
                        flows_per_pair=flows_per_pair)
    elif provider == "mcf":
        return McfDP(n=n, seed=seed, links=links, unscaled_dm_sets=unscaled_demands_sets, method=mcf_method,
                     number_samples=number_samples, flows_per_pair=flows_per_pair,
                     active_pairs_fraction=active_pairs_fraction)
    elif provider == "nanonet":
        return NanoNetDP(topology_name=topology_name)
    else:
        raise Exception(f"demand provider not found: {provider}")
