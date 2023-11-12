""" Plot script for Fig. 3, 4 and 5 in paper: 'Traffic Engineering with Joint Link Weight and Segment Optimization' """

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from utility import utility
from utility.json_result_handler import JsonResultReader
from utility.utility import HIGHLIGHT, CEND

# DEFAULT_DIR_DATA = utility.create_dirs(f"../results_paper")
DEFAULT_DIR_DATA = utility.create_dirs(f"../out")
DIR_PLOT = utility.create_dirs(f"../out/plots")

# plot settings
SMALL_SIZE = 14
LARGE_SIZE = 15
TITLE_SIZE = 17
plt.style.use('ggplot')
plt.rc('font', weight='bold', family='serif')
plt.rc('xtick', labelsize=SMALL_SIZE)
plt.rc('ytick', labelsize=SMALL_SIZE)
plt.rc('figure', titlesize=TITLE_SIZE)

# map to assign each algorithm a color later
algo_c_map = {}

# maps display name to internal name of topologies
top_n_map = {
    # sndlib
    "abilene": "Abilene",
    "geant": "Géant",
    "germany50": "Germany50",
    "polska": "Polska",
    "nobel-us": "Nobel-US",
    "atlanta": "Atlanta",
    "nobel-germany": "Nobel-Germany",
    "pdh": "Pdh",
    "nobel-eu": "Nobel-EU",
    "di": "Di",
    "janos-us": "Janos-US",
    "dfn-bwin": "Dfn-Bwin",
    "france": "France",
    "dfn-gwin": "Dfn-Gwin",
    "newyork": "Newyork",
    "norway": "Norway",
    "sun": "Sun",
    "ta1": "Ta1",
    "cost266": "Cost266",
    "janos-us-ca": "Janos-US-CA",
    "india35": "India35",
    "zib54": "Zib54",
    "giul39": "Giul39",
    "pioro40": "Pioro40",
    "ta2": "Ta2",
    "brain": "Brain",

    # topology zoo
    "basnet": "BasNet",
    "cesnet1999": "CesNet1999",
    "kreonet": "KreoNet",
    "eenet": "EeNet",
    "uran": "Uran",
    "amres": "Amres",
    "janetlense": "JanetLense",
    "renater2010": "Renater2010",
    "renater2001": "Renater2001",
    "karen": "Karen",
    "rediris": "RedIris",
    "rnp": "Rnp",
    "kentmanjan2011": "KentmanJan2011",
    "myren": "Myren",
    "belnet2006": "BelNet2006",
    "carnet": "CarNet",
    "niif": "Niif",
    "sanet": "SaNet",
    "geant2009": "Géant2009",
    "switchl3": "SwitchL3",
    "savvis": "Savvis",
    "atmnet": "Atmnet"
}


def add_vertical_algorithm_labels(ax, val_list=None):
    """ Computes the position of the vertical algorithm labels and adds them to the plot """
    ymin, ymax = ax.get_ylim()
    lines = ax.get_lines()
    boxes = [c for c in ax.get_children() if type(c).__name__ == 'PathPatch']
    lines_per_box = int(len(lines) / len(boxes))

    if val_list is None:
        val_list = list(algo_c_map.keys())
    off_set = (ymax - ymin) * 0.025
    for i, median in enumerate(lines[3:len(lines):lines_per_box]):
        x, y = (data.mean() for data in median.get_data())

        value = val_list[i % len(val_list)]
        label = value
        # if y position of label is above a certain level, label will be abbreviated
        if y > ymax - (ymax - ymin) * 0.25:
            label = f"{value[0:3]}."
        ax.text(x, y + off_set, label, ha='center', va='bottom', fontsize=SMALL_SIZE, rotation=90,
                color=algo_c_map[value],
                fontweight='bold')


def create_box_plot(df_plot, x, y, hue, file_name, x_label="", y_label="", fig_size=None,
                    title=None, y_lim_top=None):
    """ Setup and perform matplotlib boxplot"""
    fig, ax = plt.subplots(figsize=fig_size)

    flier_props = dict(markersize=1, linestyle='none')
    box_plot = sns.boxplot(x=x, y=y, hue=hue, data=df_plot, ax=ax, linewidth=0.5, flierprops=flier_props,
                           palette=algo_c_map)

    plt.ylabel(y_label, weight='bold', fontsize=LARGE_SIZE)
    plt.xlabel(f'{x_label}', weight='bold', fontsize=LARGE_SIZE)
    if title:
        plt.title(title, weight='bold', fontsize=TITLE_SIZE, color="dimgrey")
    plt.tight_layout()
    ax.set_facecolor('white')
    ax.grid(linestyle=':', color='grey', linewidth=0.5)
    ax.get_legend().remove()
    x_grid_lines = ax.get_xgridlines()
    if y_lim_top:
        plt.ylim(0.8, y_lim_top)
    for y_line in x_grid_lines:
        y_line.set_color('white')

    # Doing whatever it takes to get the list of unique algorithm names
    # plotted_algs = df_plot.groupby(hue).filter(lambda x: len(x) > 2).drop_duplicates(subset=hue)[hue].tolist()
    # add_vertical_algorithm_labels(box_plot.axes, val_list=plotted_algs)

    add_vertical_algorithm_labels(box_plot.axes)
    plt.xticks(rotation=0)
    plt.savefig(file_name.replace(" ", ""), bbox_inches="tight", format='pdf')
    plt.close()
    print(file_name)
    return


def get_incomplete_sample_nrs(df):
    """ Returns sample nrs + topologies if at least 1 algorithm result is missing """
    topology_incomplete_sample_nr_map = dict()
    n_samples = df.loc[df['sample_idx'].idxmax()]['sample_idx'] + 1
    for ilp_method in np.unique(df['algorithm_complete']):
        dfx = df[df['algorithm_complete'] == ilp_method]
        dfg_tops = dfx.groupby(by='topology_name')
        for key, group in dfg_tops:
            if n_samples > group.shape[0]:
                if key not in topology_incomplete_sample_nr_map:
                    topology_incomplete_sample_nr_map[key] = set()
                for s_nr in range(n_samples):
                    if s_nr not in list(group['sample_idx']):
                        topology_incomplete_sample_nr_map[key].add(s_nr)

    return topology_incomplete_sample_nr_map


def filter_trees(df):
    """ All results from tree topologies are removed """
    df = df[df["topology_name"] != "amres"]
    df = df[df["topology_name"] != "atmnet"]
    df = df[df["topology_name"] != "basnet"]
    df = df[df["topology_name"] != "brain"]
    df = df[df["topology_name"] != "carnet"]
    df = df[df["topology_name"] != "cesnet1999"]
    df = df[df["topology_name"] != "eenet"]
    df = df[df["topology_name"] != "kentmanjan2011"]
    df = df[df["topology_name"] != "sanet"]
    df = df[df["topology_name"] != "savvis"]
    return df


def filter_biggest_12_topologies(df):
    """ Filters 12 biggest* non-tree topologies** with complete link capacity information.
        * 'big' regarding nodes
        ** from SNDLib and TopologyZoo """
    biggest_tops = ["Ta2", "Germany50", "Zib54", "Pioro40", "Giul39", "Janos-US-CA", "Renater2010", "SwitchL3", "Myren",
                    "Cost266", "Niif", "Géant2009"]

    df = df[df.topology_name.isin(biggest_tops)]
    return df


def prepare_data_and_plot(df, title, plot_type):

    """ Prepares data (filter topologies, beautify naming, sorting) and starts plotting """
    # create plot sub directory
    out_path = utility.create_dirs(os.path.join(DIR_PLOT, plot_type))

    # filter out tree topologies
    df = filter_trees(df)

    # for the 'all algorithm plot' (including ilps) show only abilene
    if plot_type.startswith("all_algorithms"):
        df = df[(df["topology_name"] == "abilene")]

    # fill map with algorithm colors for the used algorithms
    if plot_type.startswith("real_demands"):
        algo_c_map.update({
            'UnitWeights': "black",
            'InverseCapacity': "brown",
            'HeurOSPF': "lime",
            'UnitWeightsGreedyWP': "grey",
            'UnitWeightsGreedy8SplitMP': "darkgrey",
            'UnitWeightsLocalOptMP': "silver",
            'UnitWeights2PartOptMP': "darkgrey",
            'UnitWeightsGlobalOptMP': "silver",
            'InverseCapacityGreedyWP': "maroon",
            'InverseCapacityGreedy8SplitMP': "red",
            'InverseCapacityLocalOptMP': "orange",
            'InverseCapacity2PartOptMP': "red",
            'InverseCapacityGlobalOptMP': "orange",
            'HeurOSPFGreedyWP': "darkgreen",
            'HeurOSPFGreedy8SplitMP': "green",
            'HeurOSPFLocalOptMP': "lightgreen",
            'HeurOSPF2PartOptMP': "green",
            'HeurOSPFGlobalOptMP': "lightgreen"
        })
    if plot_type.startswith("many_topologies_synthetic_demands"):
        algo_c_map.update({
            'UnitWeights': "black",
            'InverseCapacity': "brown",
            'HeurOSPF': "lime",
            'UnitWeightsGreedyWP': "grey",
            'UnitWeightsGreedy8SplitMP': "darkgrey",
            'UnitWeightsLocalOptMP': "silver",
            'UnitWeights2PartOptMP': "darkgrey",
            'UnitWeightsGlobalOptMP': "silver",
            'InverseCapacityGreedyWP': "maroon",
            'InverseCapacityGreedy8SplitMP': "red",
            'InverseCapacityLocalOptMP': "orange",
            'InverseCapacity2PartOptMP': "red",
            'InverseCapacityGlobalOptMP': "orange",
            'HeurOSPFGreedyWP': "darkgreen",
            'HeurOSPFGreedy8SplitMP': "green",
            'HeurOSPFLocalOptMP': "lightgreen",
            'HeurOSPF2PartOptMP': "green",
            'HeurOSPFGlobalOptMP': "lightgreen"
        })
    if plot_type.startswith("abilene_all_algorithms"):
        algo_c_map.update({
            'UnitWeights': "black",
            'InverseCapacity': "brown",
            'HeurOSPF': "lime",
            'UnitWeightsGreedyWP': "grey",
            'UnitWeightsGreedy8SplitMP': "darkgrey",
            'UnitWeightsLocalOptMP': "silver",
            'UnitWeights2PartOptMP': "darkgrey",
            'UnitWeightsGlobalOptMP': "silver",
            'InverseCapacityGreedyWP': "maroon",
            'InverseCapacityGreedy8SplitMP': "red",
            'InverseCapacityLocalOptMP': "orange",
            'InverseCapacity2PartOptMP': "red",
            'InverseCapacityGlobalOptMP': "orange",
            'HeurOSPFGreedyWP': "darkgreen",
            'HeurOSPFGreedy8SplitMP': "green",
            'HeurOSPFLocalOptMP': "lightgreen",
            'HeurOSPF2PartOptMP': "green",
            'HeurOSPFGlobalOptMP': "lightgreen",
            'ILP Weights': "magenta",
            'ILP Waypoints': "mediumvioletred",
            'ILP Joint': "purple"
        })

    # beautify algorithm names
    df["algorithm"] = df["algorithm"].str.replace("_", " ").str.title().str.replace(" ", "")
    df["ilp_method"] = df["ilp_method"].str.replace("_", " ").str.title().str.replace(" ", "")
    df["algorithm_complete"] = df[['algorithm', 'ilp_method']].agg(' '.join, axis=1).str.replace('  ', ' ').str.strip()
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("OspfWeights", "OSPF")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("Uniform", "Unit")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("SegmentIlp", "ILP")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("DemandFirstWaypoints", "GreedyWP")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("SequentialCombination", "JointHeur")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("JointWaypoints", "JointWaypoints")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("WaypointMultipathGreedy", "Greedy8SplitMP")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("WaypointMultipathOptimized", "LocalOptMP")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("WaypointMultipathTwoPhaseOptimization", "2PartOptMP")
    df["algorithm_complete"] = df["algorithm_complete"].str.replace("WaypointMultipathGlobalOptimization", "GlobalOptMP")

    # beautify topology names
    df["topology_name"] = df["topology_name"].apply(lambda x: top_n_map[x])

    # sort df by topology + algorithm name
    df['algorithm_complete'] = pd.Categorical(df['algorithm_complete'], list(algo_c_map.keys()))
    df = df.sort_values(by=["topology_name", "algorithm_complete"], ignore_index=True)

    # filter incomplete samples:
    incomplete = get_incomplete_sample_nrs(df)
    if incomplete:
        print(f"Remove incomplete samples from topologies (Topology, SampleNr): {incomplete}")
        for top in get_incomplete_sample_nrs(df):
            df.drop(df[(df["topology_name"] == top) & (df["sample_idx"].isin(incomplete[top]))].index, inplace=True)
        print()

    # print mean values to console
    print("Mean objective over all topologies:")
    for algo in df['algorithm_complete'].unique():
        df_x = df[df["algorithm_complete"] == algo]
        mean = np.mean(df_x["objective"].values.mean())
        print(f'{algo:>20}: {mean}')
    print()

    # plot files
    print("Plot files:")
    if plot_type.startswith("real_demands"):
        # plot figures for real demands multipath
        y_lim_top = None
        width = 6 + 1.5 * df['topology_name'].nunique()
        plot_file = os.path.join(out_path, f"real_demands.pdf")
        create_box_plot(df, "topology_name", "objective", "algorithm_complete", plot_file,
                        x_label="",
                        y_label="Max. Normalized Link Utilization", fig_size=(width, 8), title=title,
                        y_lim_top=y_lim_top)
    elif plot_type.startswith("abilene_all_algorithms"):
        # plot figures for real demands multipath
        y_lim_top = None
        width = 6 + 1.5 * df['topology_name'].nunique()
        plot_file = os.path.join(out_path, f"abilene_all_algorithms.pdf")
        create_box_plot(df, "topology_name", "objective", "algorithm_complete", plot_file,
                        x_label="",
                        y_label="Max. Normalized Link Utilization", fig_size=(width, 8), title=title,
                        y_lim_top=y_lim_top)
    elif plot_type.startswith("many_topologies_synthetic_demands1"):
        # plot figures for real demands multipath
        y_lim_top = None
        width = 6 + 1.5 * df['topology_name'].nunique()
        plot_file = os.path.join(out_path, f"many_topologies_synthetic_demands1.pdf")
        create_box_plot(df, "topology_name", "objective", "algorithm_complete", plot_file,
                        x_label="",
                        y_label="Max. Normalized Link Utilization", fig_size=(width, 8), title=title,
                        y_lim_top=y_lim_top)
    elif plot_type.startswith("many_topologies_synthetic_demands2"):
        # plot figures for real demands multipath
        y_lim_top = None
        width = 6 + 1.5 * df['topology_name'].nunique()
        plot_file = os.path.join(out_path, f"many_topologies_synthetic_demands2.pdf")
        create_box_plot(df, "topology_name", "objective", "algorithm_complete", plot_file,
                        x_label="",
                        y_label="Max. Normalized Link Utilization", fig_size=(width, 8), title=title,
                        y_lim_top=y_lim_top)
    else:
        print(f"Can't interpret plot type {plot_type}")
    return


if __name__ == "__main__":
    # parse args
    if len(sys.argv) == 1:
        dir_data = DEFAULT_DIR_DATA
    elif len(sys.argv) == 2:
        dir_data = os.path.abspath(sys.argv[1])
        if not os.path.exists(dir_data):
            raise NotADirectoryError(f"Directory {dir_data} doesn't exist")
    else:
        raise SyntaxError("Max. one argument allowed: <data-dir> containing json result data. ")

    # map data to plot titles and plot type
    raw_dfs_title = list()

    # fetch results from file and create dataframe
    # figure real_demands
    data_real_demands = os.path.join(dir_data, "results_real_demands.json")
    if os.path.exists(data_real_demands):
        df_real_demands = pd.DataFrame(
            JsonResultReader(data_real_demands).fetch_results())
        raw_dfs_title.append((df_real_demands, "Scaled Real Demands", "real_demands"))
    else:
        print(f"{utility.FAIL}results_real_demands.json not existing in {dir_data}{utility.CEND}")

    # figure abilene_all_algorithms
    data_abilene_all_algorithms = os.path.join(dir_data, "results_abilene_all_algorithms.json")
    if os.path.exists(data_abilene_all_algorithms):
        df_abilene_all_algorithms = pd.DataFrame(
            JsonResultReader(data_abilene_all_algorithms).fetch_results())
        raw_dfs_title.append((df_abilene_all_algorithms, "MCF Synthetic Demands", "abilene_all_algorithms"))
    else:
        print(f"{utility.FAIL}results_abilene_all_algorithms.json not existing in {dir_data}{utility.CEND}")

    # figure many_topologies_synthetic demands1
    data_many_topologies_synthetic_demands1 = os.path.join(dir_data, "results_many_topologies_synthetic_demands1.json")
    if os.path.exists(data_many_topologies_synthetic_demands1):
        df_many_topologies_synthetic_demands1 = pd.DataFrame(
            JsonResultReader(data_many_topologies_synthetic_demands1).fetch_results())
        raw_dfs_title.append((df_many_topologies_synthetic_demands1, "MCF Synthetic Demands", "many_topologies_synthetic_demands1"))
    else:
        print(f"{utility.FAIL}results_many_topologies_synthetic_demands1.json not existing in {dir_data}{utility.CEND}")

    # figure many_topologies_synthetic demands2
    data_many_topologies_synthetic_demands2 = os.path.join(dir_data, "results_many_topologies_synthetic_demands2.json")
    if os.path.exists(data_many_topologies_synthetic_demands2):
        df_many_topologies_synthetic_demands2 = pd.DataFrame(
            JsonResultReader(data_many_topologies_synthetic_demands2).fetch_results())
        raw_dfs_title.append((df_many_topologies_synthetic_demands2, "MCF Synthetic Demands", "many_topologies_synthetic_demands2"))
    else:
        print(f"{utility.FAIL}results_many_topologies_synthetic_demands2.json not existing in {dir_data}{utility.CEND}")

    # start plot process for each dataframe
    for df_i, title_i, plot_type_i in raw_dfs_title:
        print(f"{HIGHLIGHT}{title_i} - {plot_type_i}{CEND}")
        prepare_data_and_plot(df_i, title_i, plot_type_i)
        print()
