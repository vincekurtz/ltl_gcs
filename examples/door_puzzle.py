from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron

##
#
# A more complicated key-door scenario with 5 doors and 5 keys, each in
# different rooms. This scenario is used in in
#
# Sun, Dawei, et al. "Multi-agent motion planning from signal temporal logic
# specifications.", RA-L 2022.
#
# Vega-Brown, William, and Nicholas Roy. "Admissible abstractions for
# near-optimal task and motion planning.", IJCAI 2018.
#
##

# Construct the labeled transition system
ts = TransitionSystem(2)

ts.AddPartition(
        HPolyhedron.MakeBox([0,0],[2,5]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([0,8],[10,10]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([0,5.5],[4.5,7.5]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([8,0],[10,2]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([8,2.5],[10,7.5]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([2.5,0],[7.5,5]), [])
ts.AddPartition(
        HPolyhedron.MakeBox([5,5],[7.5,7.5]), [])

ts.AddPartition(
        HPolyhedron.MakeBox([2,0.5],[2.5,1.5]), ["d1"])
ts.AddPartition(
        HPolyhedron.MakeBox([0.5,5],[1.5,5.5]), ["d4"])
ts.AddPartition(
        HPolyhedron.MakeBox([7.5,6],[8,7]), ["d2"])
ts.AddPartition(
        HPolyhedron.MakeBox([8.5,2],[9.5,2.5]), ["d3"])
ts.AddPartition(
        HPolyhedron.MakeBox([8.5,7.5],[9.5,8]), ["d5"])

ts.AddPartition(
        HPolyhedron.MakeBox([0.5,0.5],[1.5,1.5]), ["k3"])
ts.AddPartition(
        HPolyhedron.MakeBox([6,0.5],[7,1.5]), ["k1"])
ts.AddPartition(
        HPolyhedron.MakeBox([8.5,0.5],[9.5,1.5]), ["k4"])
ts.AddPartition(
        HPolyhedron.MakeBox([3,3.5],[4,4.5]), ["k2"])
ts.AddPartition(
        HPolyhedron.MakeBox([3,6],[4,7]), ["k5"])

ts.AddPartition(
        HPolyhedron.MakeBox([0.5,8.5],[1.5,9.6]), ["goal"])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
print("Converting to DFA")
spec = "(~d1 U k1) & (~d2 U k2) & (~d3 U k3) & (~d4 U k4) & (~d5 U k5) & (F goal)"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
print("Computing product GCS")
start_point = [5.0, 2.5]
order = 3
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
print("Solving Shortest Path")
bgcs.AddLengthCost(norm="L2")
bgcs.AddDerivativeCost(degree=1, weight=0.5)
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

