from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron

##
#
# A robot must visit several targets of various types
#
##

np.random.seed(0)
groups = ["a","b","c","d","e"]
targets_per_group = 2

# Construct the labeled transition system
print("Constructing Transition System")
ts = TransitionSystem(2)

ts.AddPartition(
        HPolyhedron.MakeBox([0,0],[10,10]), [])

for g in groups:
    for j in range(targets_per_group):
        x = np.random.uniform(0,9)
        y = np.random.uniform(0,9)

        ts.AddPartition(
                HPolyhedron.MakeBox([x,y],[x+1,y+1]), [g])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
print("Converting Specification to DFA")
spec = f"True"    # (F a) & (F b) & ...
for g in groups:
    spec += f" & (F {g})"

dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
print("Constructing GCS problem")
start_point = [5.0, 2.0]
order = 3
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
print("Solving GCS problem")
bgcs.AddLengthCost(norm="L2")
bgcs.AddDerivativeCost(norm="L2", degree=1, weight=0.5)
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

