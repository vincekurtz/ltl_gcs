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

def generate_transition_system(num_partitions, seed=0, xmax=15, ymax=10):
    """
    Create a transition system with randomly generated polygonal partitions.

    Args:
        num_partitions: number of partitions to create
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

        vpoly = VPolytope(vertices.T)
        hpoly = vpoly_to_hpoly(vpoly)

        ts.AddPartition(hpoly, [])

    ts.AddEdgesFromIntersections()

    return ts

ts = generate_transition_system(10)
ts.visualize() 
plt.show()
