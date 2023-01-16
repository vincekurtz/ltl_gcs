from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron

##
#
# A reach-avoid scenario based on an example in Shoukry et al., "SMC:
# Satisfiability Modulo Convex Programming"
#
##

# Construct a labeled transition system based on a simple grid
ts = TransitionSystem(2)

ts.AddPartition(
        HPolyhedron.MakeBox([0,0],[2,8]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([2,6],[3,8]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([3,0],[6,9]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([0,9],[4,13]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([5,9],[6,13]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([7,7],[12,13]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([7,0],[12,6]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([6,0],[7,1]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([7,6],[8,7]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([6,12],[7,13]), 
        [])
ts.AddPartition(
        HPolyhedron.MakeBox([11,5],[11.1,5.1]), 
        ["g"])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
spec = "F g"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
start_point = [1.5, 2.0]
order = 5
continuity = 2
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
bgcs.AddLengthCost(norm="L2")
bgcs.AddDerivativeCost(norm="L2", degree=1, weight=1.0)
bgcs.AddDerivativeCost(norm="L2", degree=2, weight=1.0)
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
    
    bgcs.AnimateSolution(res, save=False, filename='media/maze.gif')

    plt.show()
else:
    print("Optimization failed!")

