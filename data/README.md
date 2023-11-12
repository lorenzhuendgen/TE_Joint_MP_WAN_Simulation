# REAL WORLD DATA
For the experiments, real-world data from two sources was used:

## SNDLib data
Traffic and topology data from [SNDLib](http://sndlib.zib.de/home.action), which we redistribute under the license: 
### DEMANDS
|Network   | Granularity | Horizon | #Matrices | XML TGZ  |
|----------|-------------|---------|-----------|----------|
|abilene   | 5 min       | 6 month | 48096     | [Download](http://sndlib.zib.de/download/directed-abilene-zhang-5min-over-6months-ALL.tgz) 
|geant	   | 15 min      | 4 month | 11460     | [Download](http://sndlib.zib.de/download/directed-geant-uhlig-15min-over-4months-ALL.tgz) 
|germany50 | 5 min       | 1 day   | 288       | [Download](http://sndlib.zib.de/download/directed-germany50-DFN-aggregated-5min-over-1day.tgz) 

### TOPOLOGIES
Used topologies are provided from SNDLib [Download][http://sndlib.zib.de/download/sndlib-networks-xml.zip](Download)

## TopologyZoo data
Topology data available from [TopologyZoo](https://www.topology-zoo.org/dataset.html) was used for some experiments.
To reproduce the results using TopologyZoo data:
1. Download the whole dataset: [Download](https://www.topology-zoo.org/files/archive.zip)
2. Unzip the data
3. Save the *.graphml files in the directory [data/topologies/topology_zoo](data/topologies/topology_zoo/)
