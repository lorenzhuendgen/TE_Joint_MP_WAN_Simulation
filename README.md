# Exploring Traffic Engineering - Multipath Routing with Link Weight and Segment Optimization

## Overview 
The code has been the basis for computational evaluations within the bachelor thesis '**Exploring Traffic Engineering - Multipath Routing with Link Weight and Segment Optimization**'.
This repository contains all implemented algorithms, traffic and topology generators. Additionally, the raw results (JSON), and the plotting script used for the result figures from the thesis are provided.
The basis for this repository is the repository [TE_SR_WAN_simulation](https://github.com/tfenz/TE_SR_WAN_simulation), following the publication '**Traffic Engineering with Joint Link Weight and Segment Optimization**'.

## Dependencies and Requirements
The algorithms are implemented in [Python (3.7.10)](https://www.python.org/downloads/release/python-3710/) leveraging the library [NetworkX (2.5.1)](https://networkx.github.io/documentation/networkx-2.4/) and [NetworKit (8.1)](https://github.com/networkit/networkit). 
To solve the ILP we used [Gurobi (9.1.2)](https://www.gurobi.com/downloads/gurobi-software/).  
The package manager [conda (4.8.2)](https://anaconda.org/anaconda/beautifulsoup4/files?version=4.8.2) can be used to easily set up necessary packages. See the conda [environment.yml](environment.yml) for this.

The python library *NetworKit* does not support Microsoft Windows in version 8.1, but the code should run on Ubuntu and MacOS.
The host (virtual) machine for this thesis was running Ubuntu 18.04.5 LTS.

## Structure

| Directory                             | Description                                                                    |
|---------------------------------------|--------------------------------------------------------------------------------|
| **[data/](data)**                     | Target directory for real-world traffic/topologies from SNDLib and TopologyZoo |
| **[results_thesis/](results_thesis)** | Raw result data (json) used in the evaluations shown in the thesis             |
| **[out/](out)**                       | Stores generated json results and plots                                        |
| **[src/](src)**                       | Source root containing *main.py* and plot_results.py                           |
| **[src/algorithm/](src/algorithm)**   | WAN Routing algorithms (link weight, segment and/or multipath optimizations)   |
| **[src/topology/](src/topology)**     | Topology provider (reads/prepares available real-world topology data)          |
| **[src/demand/](src/demand)**         | Reader for real world traffic data and synthetic traffic generator             |
| **[src/utility/](src/utility)**       | Globally shared statics/consts and helper classes (e.g. JSON reader/writer)    |

## Prerequisites
### Conda
Conda is used as package manager with this [environment.yml](environment.yml) defining the conda environment used in the evaluations.
For details go to: [install conda](https://docs.conda.io/projects/conda/en/latest/user-guide/install/)

### Gurobi
Gurobi is used to solve linear problems for the ILP algorithms. To reproduce the results a licence is required (academic licences are freely available here: 
[info](https://www.gurobi.com/academia/academic-program-and-licenses/)). 
Download and install the Gurobi Optimizer (9.1.2) from [download](https://www.gurobi.com/downloads/).

## Real-World Data
For some variety in the experiments, real as well as synthetic topologies and demands are used from [SNDLib](http://sndlib.zib.de/home.action) and [TopologyZoo](http://www.topology-zoo.org/dataset.html).

Overview of real-world data usage
* Figure 6.1 and 6.2 (many topologies): Topology data from SNDLib and TopologyZoo.
* Figure 6.3 (real demands): Topology and traffic data from SNDLib.
* Figure 6.4 (all algorithms): Topology data from SNDLib.

### SNDLib Data
Traffic and topology data from SNDLib was used, which we redistribute under the [ZIB ACADEMIC LICENSE](data/LICENSE_SNDLib).
The data is stored in the directory **[data/](data)**.

### TopologyZoo Data
Additionally, topology data available from [TopologyZoo](http://www.topology-zoo.org/dataset.html) was used.

**Note:** The data from topology zoo is **NOT** included in the repository and must be manually added:
1. Download the whole dataset: [Download](http://www.topology-zoo.org/files/archive.zip)
2. Unzip the data
3. Save the *.graphml files in the directory [data/topologies/topology_zoo](data/topologies/topology_zoo/))

## Install Python & Dependencies
Create a conda environment and install all python dependencies using the provided environment.yml file:
```bash
conda env create -f environment.yml
```
The created environment is named 'wan_sr', activate with:
```bash
conda activate wan_sr
```

## Run Tests
Navigate to source code root:
```bash
cd ./src
```

### Start 
Run evaluation with:
```bash
python3.7 main.py
```

### Output
The results are stored in a JSON file located in **[out/](src)** after running the main.py script.
*Note: The directory **[results_paper/](results_thesis)** contains the raw results obtained during the evaluations of the thesis.*

## Plot Results
Create Plots from provided raw result data 
```bash
python3 plot_results.py [optional <data-dir> containing json result data]
```
*Note: By default, the script plots the raw result data used in Fig. 6.1-6.4 in the thesis. To plot the data created by running the main.py script, you can pass the directory containing the json files as parameter to the plotting script. E.g.:* 
```bash
python3 plot_results.py "../out/"
```


*This project is licensed under the [MIT License](LICENSE)*.

