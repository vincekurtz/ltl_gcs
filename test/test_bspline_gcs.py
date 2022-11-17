import unittest

from ltlgcs.bspline_gcs import BSplineGraphOfConvexSets

from pydrake.geometry.optimization import HPolyhedron, VPolytope

import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt

class TestBsplineGcs(unittest.TestCase):
    def test_scenario_plot(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]

        regions = {1 : HPolyhedron.MakeBox([0,0],[2,2]),
                   2 : HPolyhedron.MakeBox([0,2],[2,4]),
                   3 : HPolyhedron.MakeBox([2,2],[4,4])}

        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions, 1, 3, [1,1],
                order=2, continuity=0)
        bgcs.PlotScenario()
        plt.show(block=True)  # use block=True to see the plot

    def test_solve_shortest_path(self):
        vertices = [0,1,2,3,4,5,6]
        edges = [(0,1), (1,0), 
                 (1,2), (2,1),
                 (2,3), (3,2),
                 (3,4), (4,3), 
                 (4,6),
                 (0,5), (5,0),
                 (5,6)]
        regions = {0 : HPolyhedron.MakeBox([1,-1],[6,0]),
                   1 : HPolyhedron.MakeBox([0,0],[2,2]),
                   2 : HPolyhedron.MakeBox([0,2],[2,4]),
                   3 : HPolyhedron.MakeBox([2,2],[4,4]),
                   4 : HPolyhedron.MakeBox([4,3],[6,5]),
                   5 : HPolyhedron.MakeBox([5,0],[7,1]),
                   6 : HPolyhedron.MakeBox([0,0],[0,0])}  # target is irrelevant
        
        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions, 1, 6,
                [1.0,1.2], order=3, continuity=2)
        bgcs.AddLengthCost(norm="L2")
        bgcs.AddDerivativeCost(1, weight=0.1, norm="L2")
        res = bgcs.SolveShortestPath(verbose=True)
        self.assertTrue(res.is_success())

        bgcs.PlotScenario()
        bgcs.PlotSolution(res, plot_control_points=True, plot_path=True)
        plt.show()

    def test_kl_loop(self):
        # Define the regions in question
        p0 = HPolyhedron.MakeBox([0,0],[1,2])
        p1 = HPolyhedron.MakeBox([1,0],[2,0.8])
        p2 = HPolyhedron.MakeBox([1,0.8],[2,1.2])
        p3 = HPolyhedron.MakeBox([1,1.2],[2,2])
        regions = {0 : p0,
                   1 : p1,
                   2 : p1,
                   3 : p2,
                   4 : p3,
                   5 : p0,
                   6 : p1,
                   10 : HPolyhedron.MakeBox([0,0], [0,0])}  # target
        vertices = [0,1,2,3,4,5,6,10]

        # First GCS takes us to an accepting state
        edges = [(0,1), (1,10)]
        bgcs1 = BSplineGraphOfConvexSets(vertices, edges, regions, 0, 10,
                [0.5,1.0], order=4, continuity=2)
        bgcs1.AddLengthCost()
        bgcs1.AddDerivativeCost(1)
        res1 = bgcs1.SolveShortestPath(verbose=True, convex_relaxation=False)
        self.assertTrue(res1.is_success())

        # Second GCS finds a loop
        x_loop = res1.GetSolution(bgcs1.gcs_verts[1].x())
        edges = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,6),(6,10)]
        bgcs2 = BSplineGraphOfConvexSets(vertices, edges, regions, 1, 10,
                [x_loop[0],x_loop[1]], order=4, continuity=2)

        loop_start = bgcs2.gcs_verts[1]
        loop_end = bgcs2.gcs_verts[6]

        for i in range(len(x_loop)):
            loop_end.AddConstraint(loop_end.x()[i] == x_loop[i])
            loop_start.AddConstraint(loop_start.x()[i] == x_loop[i])

        bgcs2.AddLengthCost()
        bgcs2.AddDerivativeCost(1)
        bgcs2.AddDerivativeCost(2)

        res2 = bgcs2.SolveShortestPath(verbose=True, convex_relaxation=False)
        self.assertTrue(res2.is_success())

        # Plot the scenario
        for region in (p0, p1, p2, p3):
            v = VPolytope(region).vertices().T
            hull = ConvexHull(v)
            v_sorted = np.vstack([v[hull.vertices,0], v[hull.vertices,1]]).T

            poly = Polygon(v_sorted, alpha=0.5, edgecolor='k', linewidth=3)
            plt.gca().add_patch(poly)

            if region == p1:
                label = "a"
            elif region == p3:
                label = "b"
            else:
                label = ""

            center_point = region.ChebyshevCenter()
            plt.text(center_point[0], center_point[1],
                    label,
                    horizontalalignment='center',
                    verticalalignment='center',
                    fontsize=12, color='black')


        plt.axis('equal')

        # Plot the solution
        bgcs1.PlotSolution(res1, plot_control_points=True, plot_path=True)
        bgcs2.PlotSolution(res2, plot_control_points=True, plot_path=True)
        plt.show()



