import unittest

from ltlgcs.bezier_gcs import BezierGraphOfConvexSets

from pydrake.geometry.optimization import HPolyhedron, VPolytope

import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt

class TestBezierGcs(unittest.TestCase):
    def test_scenario_plot(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]

        regions = {1 : HPolyhedron.MakeBox([0,0],[2,2]),
                   2 : HPolyhedron.MakeBox([0,2],[2,4]),
                   3 : HPolyhedron.MakeBox([2,2],[4,4])}

        bgcs = BezierGraphOfConvexSets(vertices, edges, regions, 1, 3, [1,1],
                order=2, continuity=0)
        bgcs.PlotScenario()
        plt.show(block=False)  # use block=True to see the plot

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
        
        bgcs = BezierGraphOfConvexSets(vertices, edges, regions, 1, 6,
                [1.0,1.2], order=3, continuity=2)
        bgcs.AddLengthCost(norm="L2")
        bgcs.AddDerivativeCost(1, weight=0.1, norm="L2")
        res = bgcs.SolveShortestPath(verbose=False)
        self.assertTrue(res.is_success())
        
        curves = bgcs.ExtractSolution(res)
        self.assertTrue(len(curves) == 4)

        bgcs.PlotScenario()
        bgcs.PlotSolution(res, plot_control_points=True, plot_path=True)
        bgcs.AnimateSolution(res, show=False)
        plt.show(block=False)  # use block=True to see the plot

    def test_kl_loop(self):
        # Define the regions in question
        p0 = HPolyhedron.MakeBox([0,0],[1,2])
        p1 = HPolyhedron.MakeBox([1,0],[2,0.8])
        p2 = HPolyhedron.MakeBox([1,0.8],[2,1.2])
        p3 = HPolyhedron.MakeBox([1,1.2],[2,2])
        regions = {0 : p0,
                   1 : p1,
                   2 : p2,
                   3 : p3,
                   4 : p3,
                   5 : p0,
                   6 : p1,
                   7 : p2,
                   8 : p3,
                   10 : HPolyhedron.MakeBox([0,0], [0,0])}  # target
        vertices = [0,1,2,3,4,5,6,7,8,10]

        # First GCS takes us to an accepting state
        edges = [(0,1), (1,2), (2,3), (3,10)]
        bgcs1 = BezierGraphOfConvexSets(vertices, edges, regions, 0, 10,
                [0.5,1.0], order=4, continuity=2)
        bgcs1.AddLengthCost()
        bgcs1.AddDerivativeCost(1)
        bgcs1.AddDerivativeCost(2, weight=3)
        res1 = bgcs1.SolveShortestPath(verbose=False, convex_relaxation=False)
        self.assertTrue(res1.is_success())

        # Second GCS finds a loop
        x_loop = res1.GetSolution(bgcs1.gcs_verts[3].x())
        edges = [(0,1),(1,2),(2,3),(3,4),(4,5),(5,6),(6,7),(7,8),(8,10)]
        bgcs2 = BezierGraphOfConvexSets(vertices, edges, regions, 3, 10,
                [x_loop[0],x_loop[1]], order=4, continuity=2)

        loop_start = bgcs2.gcs_verts[3]
        loop_end = bgcs2.gcs_verts[8]

        for i in range(len(x_loop)):
            loop_end.AddConstraint(loop_end.x()[i] == x_loop[i])
            loop_start.AddConstraint(loop_start.x()[i] == x_loop[i])

        bgcs2.AddLengthCost()
        bgcs2.AddDerivativeCost(1)
        bgcs2.AddDerivativeCost(2)

        res2 = bgcs2.SolveShortestPath(verbose=False, convex_relaxation=False)
        self.assertTrue(res2.is_success())

        # Plot the scenario
        plt.figure(figsize=(3,3))
        for region in (p0, p1, p2, p3):
            v = VPolytope(region).vertices().T
            hull = ConvexHull(v)
            v_sorted = np.vstack([v[hull.vertices,0], v[hull.vertices,1]]).T

            poly = Polygon(v_sorted, facecolor='white', edgecolor='k', linewidth=1)
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

        plt.gca().xaxis.set_visible(False)
        plt.gca().yaxis.set_visible(False)
        plt.gca().set_facecolor('black')
        plt.axis('equal')

        # Plot the solution
        bgcs1.PlotSolution(res1, plot_control_points=False, plot_path=True)
        bgcs2.PlotSolution(res2, plot_control_points=False, plot_path=True)
        plt.show(block=False)  # use block=True to see the plot

