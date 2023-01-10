from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton
from ltlgcs.cdd import vpoly_to_hpoly

import time
import numpy as np
import matplotlib.pyplot as plt

from pydrake.geometry.optimization import HPolyhedron, VPolytope

##
#
# An example for comparison with with the state-of-the-art sampling based method
# described in 
#
# Luo et al, "An Abstraction-Free Method for Multirobot Temporal Logic Optimal
# Control Synthesis", TRO 2021.
#
##

# Create the scenario
ts = TransitionSystem(2)

ts.AddPartition(HPolyhedron.MakeBox([0,0],[0.3,0.2]),[])
ts.AddPartition(HPolyhedron.MakeBox([0,0.2],[1.0,0.7]),[])
ts.AddPartition(HPolyhedron.MakeBox([0.7,0],[1.0,0.2]),[])
ts.AddPartition(HPolyhedron.MakeBox([0,0.7],[0.4,1.0]),[])
ts.AddPartition(HPolyhedron.MakeBox([0.6,0.7],[1.0,1.0]),[])

def triangle_at(x, y):
    return vpoly_to_hpoly(VPolytope(
        np.array([[x,y],[x+0.15,y],[x,y+0.15]]).T))

ts.AddPartition(triangle_at(0.1,0.7), ["l1"])
ts.AddPartition(triangle_at(0.7,0.7), ["l2"])
ts.AddPartition(triangle_at(0.7,0.3), ["l3"])
ts.AddPartition(triangle_at(0.3,0.3), ["l4"])
ts.AddPartition(triangle_at(0.0,0.1), ["l5"])
ts.AddPartition(triangle_at(0.0,0.4), ["l6"])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
print("Converting to DFA")
spec = "F (l1 & (F l3)) & (~l1 U l2) & F (l5 & F (l6 & F l4)) & (~l4 U l5)"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system to produce a graph of
# convex sets
print("Constructing GCS")
start_point = [0.8, 0.1]
order = 1
continuity = 0
product_start_time = time.time()
bgcs = ts.Product(dfa, start_point, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
print("Solving Shortest Path")
bgcs.AddLengthCost(norm="L2")
solve_start_time = time.time()
res = bgcs.SolveShortestPath(
        convex_relaxation=True,
        preprocessing=False,
        verbose=True,
        max_rounded_paths=1,
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

    plt.show()
else:
    print("Optimization failed!")

