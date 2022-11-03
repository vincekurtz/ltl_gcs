from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron

# Construct a labeled transition system based on a simple grid
nx, ny = (4, 4)
ts = TransitionSystem(2)

for x in range(nx):
    for y in range(ny):
        partition = HPolyhedron.MakeBox([x,y],[x+1,y+1])

        # Associate partitions with labels
        labels=[]
        if (x,y) == (2,3):
            labels.append("goal")
        elif 1<=x and x<=2 and 1<=y and y<=2:
            labels.append("obs")

        ts.AddPartition(partition, labels)
ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
spec = "~obs U goal"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
start_point = [0.5, 0.2]
order = 2
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
bgcs.AddLengthCost(norm="L2")
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

    plt.show()
else:
    print("Optimization failed!")

