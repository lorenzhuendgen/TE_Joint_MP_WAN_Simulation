""" Factory for topology providers"""

from topology.generic_topology_provider import GenericTopologyProvider
from topology.snd_lib.sndlib_top import SndLibTop
from topology.topology_zoo.topology_zoo_top import TopologyZoo
from topology.nanonet.nanonet_top import NanoNetTop


def get_topology_factory(provider: str) -> GenericTopologyProvider:
    provider = provider.lower()
    if provider == "topology_zoo":
        return TopologyZoo()
    if provider == "snd_lib":
        return SndLibTop()
    if provider == "nanonet":
        return NanoNetTop()

    msg = f"Topology Provider not found with: {provider}"
    raise Exception(msg)
