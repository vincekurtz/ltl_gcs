from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron

##
#
# A robot must pick up a key before passing through a door to reach a target
#
##

# Construct the labeled transition system
ts = TransitionSystem(2)

ts.AddPartition(
        HPolyhedron.MakeBox([-2,0],[5,10]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([5,4],[7,6]), ["door1"])
ts.AddPartition(
        HPolyhedron.MakeBox([7,2.1],[9,10]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([7,0],[9,2.1]), ["door2"])
ts.AddPartition(
        HPolyhedron.MakeBox([9,0],[11,2]), ["goal"])
ts.AddPartition(
        HPolyhedron.MakeBox([-2,0],[0,2]), ["key1"])
ts.AddPartition(
        HPolyhedron.MakeBox([-2,8],[0,10]), ["key2"])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
spec = "(~door1 U key1) & (~door2 U key2) & (F goal)"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
start_point = [4.0, 9.0]
order = 3
continuity = 2
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
bgcs.AddLengthCost(norm="L2")
bgcs.AddDerivativeCost(degree=1, weight=1.0)
bgcs.AddDerivativeCost(degree=2, weight=1.0)
solve_start_time = time.time()
res = bgcs.SolveShortestPath(
        convex_relaxation=True,
        preprocessing=False,
        verbose=True,
        max_rounded_paths=10,
        solver="mosek")
solve_time = time.time() - solve_start_time

if res.is_success():
    # Plot the resulting trajectory
    color_dict = {
            "white": [[]],
            "#2077B4": [["goal"]],
            "#F14732": [["door1"],["door2"]],
            "#80BF80": [["key1"],["key2"]]
            }
    ts.visualize(color_dict, background='black', alpha=1.0)
    bgcs.PlotSolution(res, plot_control_points=False, plot_path=True)
    
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

    # Make an animation of the trajectory
    bgcs.AnimateSolution(res, save=False, filename='media/key_door.gif')

    plt.show()
else:
    print("Optimization failed!")

