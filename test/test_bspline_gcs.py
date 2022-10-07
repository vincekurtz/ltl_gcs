import unittest

from ltlgcs.bspline_gcs import BSplineGraphOfConvexSets

from pydrake.geometry.optimization import HPolyhedron
import matplotlib.pyplot as plt

class TestBsplineGcs(unittest.TestCase):
    def test_scenario_plot(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]

        regions = {1 : HPolyhedron.MakeBox([0,0],[2,2]),
                   2 : HPolyhedron.MakeBox([0,2],[2,4]),
                   3 : HPolyhedron.MakeBox([2,2],[4,4])}

        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions, 1, 3, [1,1],
                order=2)
        bgcs.PlotScenario()
        plt.show(block=False)  # use block=True to see the plot

    def test_solve_shortest_path(self):
        vertices = [0,1,2,3,4,5]
        edges = [(0,1),(1,2), (2,3), (3,4), (4,5)]
        regions = {0 : HPolyhedron.MakeBox([1,-1],[2,0]),
                   1 : HPolyhedron.MakeBox([0,0],[2,2]),
                   2 : HPolyhedron.MakeBox([0,2],[2,4]),
                   3 : HPolyhedron.MakeBox([2,2],[4,4]),
                   4 : HPolyhedron.MakeBox([4,3],[6,5]),
                   5 : HPolyhedron.MakeBox([0,0],[0,0])}  # target is irrelevant
        
        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions, 1, 5,
                [1.0,1.0], order=2, continuity=1)
        bgcs.AddLengthCost(norm="L2_squared")
        bgcs.AddDerivativeCost()
        #bgcs.AddLengthCost(weight=1e1, norm="L2_squared")
        res = bgcs.SolveShortestPath(verbose=True)
        self.assertTrue(res.is_success())

        # DEBUG: make some nice plots
        bgcs.PlotScenario()
        bgcs.PlotSolution(res, plot_control_points=True, plot_path=True)
        plt.show()



