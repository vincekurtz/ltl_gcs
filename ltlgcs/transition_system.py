from ltlgcs.graph import DirectedGraph
from ltlgcs.dfa import DeterministicFiniteAutomaton
from ltlgcs.bspline_gcs import BSplineGraphOfConvexSets

from pydrake.all import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from sympy.parsing.sympy_parser import parse_expr


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

    def Product(self, dfa, start_point, order, continuity):
        """
        Compute the product of this transition system and a Deterministic Finite
        Automaton (DFA), which is a B-spline graph of convex sets. 

        Args:
            dfa: Deterministic Finite Automaton corresponding to some LTL
                 specification.
            start_point: starting point, i.e., initial system state
            order: degree of the bezier curve that will be planned through the
                   graph of convex sets
            continuity: continuity of the bezier curve that will be planned
                   through the graph of convex sets

        Returns:
            gcs: BSplineGraphOfConvexSets such that any path through the graph
                 corresponds to a path through this transition system that satisfies
                 the LTL specification defined by the given DFA. 
        """
        assert isinstance(dfa, DeterministicFiniteAutomaton)

        # Check that the starting point is in one of the partitions
        # TODO: handle the case where the starting point might be contained in
        # multiple partitions
        s0 = None
        for s in self.vertices:
            if self.partitions[s].PointInSet(start_point):
                s0 = s
        assert s0 is not None, \
                "the given start point is not contained in any partition"

        # Construct vertices: one for each pair of vertices in the DFA and this
        # transition system
        states = {}
        state_idx = 0
        vertices = []
        for s in self.vertices:
            for q in dfa.vertices:
                states[state_idx] = (s,q)
                vertices.append(state_idx)
                state_idx += 1

        # Define regions (convex sets) for each vertex in the graph of convex sets
        regions = {}
        for v in vertices:
            s, q = states[v]
            regions[v] = self.partitions[s]

        # Define the starting vertex
        start_vertex = None
        for v in vertices:
            s, q = states[v]
            if (s == s0) and (q == dfa.initial_vertex):
                start_vertex = v
        assert start_vertex is not None, "could not find a valid start vertex"
        # Define edges in the graph of convex sets. An edge between (s,q) and
        # (s',q') exists if the following conditions hold:
        #
        #   1. s-->s' in this transition system
        #   2. q-->q' in the DFA
        #   3. The label of s in the transition system satisfies the label of
        #      (q-->q') in the DFA.
        edges = []
        for v in vertices:
            for v_prime in vertices:
                if (v != v_prime):  # There are no self-loops
                    s, q = states[v]
                    s_prime, q_prime = states[v_prime]

                    if ((s, s_prime) in self.edges) and \
                            ((q, q_prime) in dfa.edges):
                        ts_label = self.labels[s_prime]
                        dfa_label = dfa.labels[(q,q_prime)]

                        # Only add edges where the label of transition system
                        # matches the label of the DFA
                        if satisfies(ts_label, dfa_label):
                            edges.append((v,v_prime))
        
        # Define the ending vertex. There are edges from (s,q) to the end
        # vertex whenever q is in the accepting set of the DFA. 
        end_vertex = state_idx
        for v in vertices:
            s, q = states[v]
            if q in dfa.accepting_vertices:
                edges.append((v, end_vertex))
        vertices.append(end_vertex)

        # TODO: eliminate need for this dummy region for the target state
        regions[end_vertex] = HPolyhedron.MakeBox([0,0],[0,0])

        # Construct and return the graph of convex sets
        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions, start_vertex,
                end_vertex, start_point, order, continuity)

        return bgcs

def satisfies(ts_label, dfa_label):
    """
    Check if a transition system label (e.g., ["a","b"]) satisfies a given DFA
    label (e.g. "a & ~b", "a | b", etc). 

    Args:
        ts_label: a list of predicates that hold for a particular partition in a
                  transition system. Any predicates that are not listed are
                  assumed to not hold. 
        dfa_label: a simple boolean formula denoting which predicates should
                  hold for a given DFA transition to be valid. Generated by
                  ltl2dfa.

    Returns:
        True if ts_label satisfies dfa_label and False otherwise
    """
    # Construct a dictionary mapping predicates to their truth values depending
    # as defined by ts_label.
    ts_dict = {}
    expression = parse_expr(dfa_label)
    for symbol in expression.free_symbols:
        if str(symbol) in ts_label:
            ts_dict[str(symbol)] = 1
        else:
            ts_dict[str(symbol)] = 0

    # Use sympy to check whether the formula in ts_dict holds
    res = expression.subs(ts_dict)
    return res

