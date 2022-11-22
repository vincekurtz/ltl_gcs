from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton
from ltlgcs.cdd import vpoly_to_hpoly

import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Voronoi
from copy import copy

from pydrake.geometry.optimization import HPolyhedron, VPolytope

##
#
# A large-scale example with randomly generated non-rectangular regions
#
##

def generate_transition_system(num_partitions, label_dict, seed=0, xmax=15,
        ymax=10):
    """
    Create a transition system with randomly generated polygonal partitions.

    Args:
        num_partitions: number of partitions to create
        label_dict: dictionary of label probabilities. {"a":0.1} gives a 10%
                    chance of assigning label "a" to each partition.
        of "a" to each partition
        seed: seed for pseudorandom number generator
        xmax: size of the workspace in the horizontal direction
        ymax: size of the workspace in the vertical direction

    Returns:
        a labeled transition system
    """
    ts = TransitionSystem(2)

    # Define seed points for a voronoi diagram
    np.random.seed(seed)
    points = np.random.uniform(low=[0, 0], 
            high=[xmax, ymax], size=(num_partitions, 2))

    # Per https://stackoverflow.com/questions/28665491/, we get a bounded voronoi
    # diagram by reflecting the original points up, down, left, and right
    left_reflection = copy(points)
    bottom_reflection = copy(points)
    right_reflection = copy(points)
    top_reflection = copy(points)
    left_reflection[:,0] *= -1
    bottom_reflection[:,1] *= -1
    right_reflection[:,0] *= -1
    right_reflection[:,0] += 2*xmax
    top_reflection[:,1] *= -1
    top_reflection[:,1] += 2*ymax
    all_points = np.vstack([points, left_reflection, bottom_reflection,
        right_reflection, top_reflection])

    # Extract convex partitions from the voronoi diagram
    vor = Voronoi(all_points)
    for p in range(num_partitions):
        point = points[p]
        region_index = vor.point_region[p]
        vertex_indices = vor.regions[region_index]
        vertices = vor.vertices[vertex_indices]

        # convert to halfspace representation
        vpoly = VPolytope(vertices.T)
        hpoly = vpoly_to_hpoly(vpoly)

        # randomly generate labels
        labels = []
        for label, probability in label_dict.items():
            if np.random.uniform() < probability:
                labels.append(label)

        if "obs" not in labels:
            ts.AddPartition(hpoly, labels)

    ts.AddEdgesFromIntersections()

    return ts

# Create the scenario
print("Constructing Transition System")
label_probabilities = {"a":0.1, "b":0.1, "c":0.1, "d":0.1, "obs":0.3}
ts = generate_transition_system(50, label_probabilities)

ts.visualize()
plt.show()

# Convert the specification to a DFA
print("Converting to DFA")
spec = "(F a) & (F b) & (F c) & (F d) & (G ~obs)"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
print("Constructing GCS")
start_point = [0.5, 8.5]
order = 3
continuity = 2
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
print("Solving Shortest Path")
bgcs.AddLengthCost(norm="L1")
bgcs.AddDerivativeCost(norm="L1", degree=1, weight=0.5)
solve_start_time = time.time()
res = bgcs.SolveShortestPath(
        convex_relaxation=True,
        preprocessing=True,
        verbose=True,
        max_rounded_paths=10,
        solver="mosek")
solve_time = time.time() - solve_start_time

if res.is_success():
    # Plot the resulting trajectory
    ts.visualize()
    bgcs.PlotSolution(res, plot_control_points=True, plot_path=True)
    
    # Print timing infos
    print("\n")
    print("Solve Times:")
    print("    LTL --> DFA    : ", dfa_time)
    print("    TS x DFA = GCS : ", product_time)
    print("    GCS solve      : ", solve_time)
    print("    Total          : ", dfa_time + product_time + solve_time)
    print("")

    print("GCS vertices: ", bgcs.nv())
    print("GCS edges: ", bgcs.ne())

    plt.show()
else:
    print("Optimization failed!")

