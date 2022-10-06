from ltlgcs.graph import DirectedGraph

from pydrake.geometry.optimization import ConvexSet, VPolytope
from pydrake.solvers import GurobiSolver

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

class BSplineGraphOfConvexSets(DirectedGraph):
    """
    Problem setup and solver for planning a B-Spline trajectory through a graph
    of convex sets. The graph setup is as follows:

        - Each vertex is associated with a convex set
        - Each convex set contains a Bezier curve
        - The optimal path is a B-spline made up of Bezier curves. These curves
          must line up with each other. 
        - The goal is to find a (minimum cost) trajectory from a given starting
          point to the target vertex
        - The target vertex is not associated with any constraints on the
          B-splines: it just indicates that the task is complete.
    """
    def __init__(self, vertices, edges, regions):
        """
        Construct a graph of convex sets

        Args:
            vertices: list of integers representing each vertex in the graph
            edges: list of pairs of integers (vertices) for each edge
            regions: dictionary mapping each vertex to a Drake ConvexSet
        """
        # General graph constructor
        super().__init__(vertices, edges)

        # Check that the regions correspond to valid vertices, and valid convex
        # sets
        for vertex, region in regions.items():
            assert vertex in self.vertices, "invalid vertex index"
            assert isinstance(region, ConvexSet), "regions must be convex sets"

        self.regions = regions

    def AddLengthCost(self):
        pass

    def AddDerivativeCost(self):
        pass

    def SolveShortestPath(self):
        pass
    
    def PlotScenario(self):
        """
        Add a plot of each region to the current matplotlib axes. 
        Only supports 2D polytopes for now.
        """
        for region in self.regions.values():
            assert region.ambient_dimension() == 2, "only 2D sets allowed"
            # Compute vertices of the polygon in known order
            v = VPolytope(region).vertices().T
            hull = ConvexHull(v)
            v_sorted = np.vstack([v[hull.vertices,0],v[hull.vertices,1]]).T

            # Make a polygonal patch
            poly = Polygon(v_sorted, alpha=0.5, edgecolor="k", linewidth=3)
            plt.gca().add_patch(poly)
       
        # Use equal axes so square things look square
        plt.axis('equal')

    def PlotSolution(self):
        pass


