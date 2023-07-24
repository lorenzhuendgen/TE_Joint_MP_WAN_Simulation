import os
from xml.dom import minidom

from topology.generic_topology_provider import GenericTopologyProvider
from topology.nanonet.file_mapping import file_map
from utility import utility
import json

class NanoNetTop(GenericTopologyProvider):
    @staticmethod
    def get_topology_names():
        """ returns names of all supported topologies """
        return list(file_map.keys())

    @staticmethod
    def __read_network_json(topology_file_name) -> (list, int):
        full_file_name = os.path.abspath(topology_file_name)

        with open(full_file_name, 'r') as file:
            data = json.load(file)

        nodes = set([link['i'] for link in data['links']] + [link['j'] for link in data['links']])
        links = [(link['i'],link['j'],link['capacity']) for link in data['links']]

        n = len(nodes)

        return links, n

    def get_topology(self, topology_name: str, default_capacity: float = 1, **kwargs) -> (list, int):
        topology_name = utility.get_base_name(topology_name).lower()
        assert topology_name in file_map, "topology not supported. \nchoose from:\n\t" + ', '.join(
            list(file_map.keys()))
        topology_file_name = os.path.join(utility.BASE_PATH_NANONET_DATA, file_map[topology_name])
        links, n = self.__read_network_json(topology_file_name)
        return links, n
