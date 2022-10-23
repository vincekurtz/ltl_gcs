from ltlgcs.graph import DirectedGraph

from pydrake.all import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

class TransitionSystem(DirectedGraph):
    """
    A finite state transition system that represents the robot's workspace.
    Each state corresponds to a convex partition of the state space. Each
    partition is labeled with a set of predicates that hold everywhere in the
    partition.

    More formally, this transition system is defined by 

        - States (vertices)
        - Transitions between states (edges)
        - Labels for each state
        - A convex set for each state
    """
    def __init__(self, n):
        """
        Construct an (empty) transition system.

        Args:
            n: the ambient dimension of each convex set
        """
        assert n > 0
        self.n = n

        self.vertices = []  # represented as integer indices [1,2,...]
        self.edges = []     # represented as tuples of indices [(1,3),...]

        self.partitions = {}  # {vertex_index : ConvexSet}
        self.labels = {}      # {vertex_index : ["a", "b"]}

        # Running counter so that we use unique vertex indices
        self.v_idx = 0

    def AddPartition(self, convex_set, labels):
        """
        Add a new state with the given convex set and labels to the transition
        system. Note that by default, this state will be disconnected from all
        other states, use AddEdge to add transitions between adjacent or
        overlapping states. 

        Args:
            convex_set: a Drake ConvexSet corresponding to this partition
            labels: a list of strings representing the predicates that hold in
                    this partition

        Returns:
            vertex_idx: an integer index representing this vertex
        """
        assert isinstance(convex_set, ConvexSet)
        assert isinstance(labels, list)
        assert convex_set.ambient_dimension() == self.n
        
        vertex_index = self.v_idx;
        
        self.vertices.append(vertex_index)
        self.partitions[vertex_index] = convex_set
        self.labels[vertex_index] = labels

        self.v_idx += 1
        return vertex_index

    def AddEdge(self, source_vertex, target_vertex):
        """
        Add a transition between two partitions (aka states aka vertices).

        Args:
            source_vertex: index of the starting vertex
            target_vertex: index of the ending vertex
        """
        assert source_vertex in self.vertices
        assert target_vertex in self.vertices
        # TODO: consider additional check that the partitions for these two
        # states are not disjoint
        self.edges.append((source_vertex, target_vertex))

    def visualize(self):
        """
        Make a pyplot visualization of the regions on the current pyplot axes. 
        Only supports 2D polytopes for now. 
        """
        for vertex in self.vertices:
            region = self.partitions[vertex]
            label = self.labels[vertex]
            assert region.ambient_dimension() == 2, "only 2D sets allowed"

            # Compute vertices of the polygon in known order
            v = VPolytope(region).vertices().T
            hull = ConvexHull(v)
            v_sorted = np.vstack([v[hull.vertices,0],v[hull.vertices,1]]).T

            # Make a polygonal patch
            poly = Polygon(v_sorted, alpha=0.5, edgecolor="k", linewidth=3)
            plt.gca().add_patch(poly)

            # Add a text label showing the predicates that hold in this partion
            center_point = region.ChebyshevCenter()
            plt.text(center_point[0], center_point[1], 
                    ", ".join(['%s']*len(label)) % tuple(label),
                    horizontalalignment='center',
                    verticalalignment='center',
                    fontsize=12, color='black')

            # Use equal axes so square things look square
        plt.axis('equal')

