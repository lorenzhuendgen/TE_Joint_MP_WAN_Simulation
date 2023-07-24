import os

from demand.generic_demand_provider import GenericDemandProvider
from demand.nanonet.file_mapping import file_map
from utility import utility
import json


class NanoNetDP(GenericDemandProvider):
    def __init__(self, topology_name: str):
        topology_name = utility.get_base_name(topology_name).lower()
        assert topology_name in file_map, f"topology not supported. \nchoose from:\n\t" + '\n\t'.join(
            list(file_map.keys()))
        self.__topology_name = topology_name
        self.__file = os.path.abspath(os.path.join(utility.BASE_PATH_NANONET_DATA, file_map[topology_name]))
        with open(self.__file, 'r') as file:
            data = json.load(file)

        self.__demands = [(link['src'], link['dst'], link['demand_size']) for link in data['demands']]
        return

    def demand_matrix(self, sample: int) -> dict:
        raise NotImplementedError()

    def demand_sequence(self, sample: int) -> list:
        raise NotImplementedError()

    def demand_matrices(self) -> list:
        raise NotImplementedError()

    def demand_sequences(self) -> list:
        """ Generator object to get all sample demand sequences """
        yield self.__demands

    def __len__(self):
        """ len is defined by the number of samples """
        return len(self.__demands)

    def __str__(self):
        self.get_name()

    def get_name(self) -> str:
        return f"NanoNet_{self.__topology_name}"
