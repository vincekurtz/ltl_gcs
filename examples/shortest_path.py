from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton
from ltlgcs.cdd import vpoly_to_hpoly

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron, VPolytope

##
#
# A simple example that demonstrates the ability to find the shortest path in
# configuration space, even when that path moves through more partitions.
#
##

# Construct a labeled transition system
ts = TransitionSystem(2)

ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[0,0],[1,0],[0,0.5],[1,0.5]]).T
        )), [])
ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[0,0.5],[0,1],[1,0.5]]).T
        )), [])
ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[0,1],[1,1],[1,0.5]]).T
        )), [])
ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[0,1],[0,1.5],[1,1]]).T
        )), [])
ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[0,1.5],[1,1.5],[1,1]]).T
        )), [])
ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[0,1.5],[0,2],[5,2],[5,1.5]]).T
        )), [])
ts.AddPartition(
        vpoly_to_hpoly(VPolytope(
            np.array([[1.5,0],[1.5,1],[4.5,1.5],[5,1.5],[5,0]]).T
        )), [])
ts.AddPartition(
        HPolyhedron.MakeBox([0.5,0.1],[0.8,0.4]),
        ["b"])
ts.AddPartition(
        HPolyhedron.MakeBox([1.7,0.1],[2.0,0.4]),
        ["a"])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
spec = "F (a | b)"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
start_point = [1.5, 1.75]
order = 4
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
bgcs.AddLengthCost(norm="L2")
bgcs.AddDerivativeCost(degree=1, norm="L2", weight=0.1)
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
    color_dict = {
            "#2077B4" : [["a"]],
            "#80BF80" : [["b"]]}
    ts.visualize(color_dict)
    bgcs.PlotSolution(res, plot_control_points=True, plot_path=True)
    plt.gca().xaxis.set_visible(False)
    plt.gca().yaxis.set_visible(False)
    
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

