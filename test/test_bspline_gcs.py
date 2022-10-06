import unittest

from ltlgcs.bspline_gcs import BSplineGraphOfConvexSets

from pydrake.geometry.optimization import HPolyhedron
import matplotlib.pyplot as plt

class TestBsplineGcs(unittest.TestCase):
    def test_constructor(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]

        r1 = HPolyhedron.MakeBox([0,0],[2,2])
        r2 = HPolyhedron.MakeBox([0,2],[2,4])
        r3 = HPolyhedron.MakeBox([2,2],[4,4])
        regions = {1:r1, 2:r2, 3:r3}

        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions)

    def test_scenario_plot(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]

        regions = {1 : HPolyhedron.MakeBox([0,0],[2,2]),
                   2 : HPolyhedron.MakeBox([0,2],[2,4]),
                   3 : HPolyhedron.MakeBox([2,2],[4,4])}

        bgcs = BSplineGraphOfConvexSets(vertices, edges, regions)
        bgcs.PlotScenario()
        plt.show()


