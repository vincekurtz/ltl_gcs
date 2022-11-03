import unittest
import warnings

from ltlgcs.transition_system import TransitionSystem, satisfies
from ltlgcs.dfa import DeterministicFiniteAutomaton

from pydrake.geometry.optimization import HPolyhedron
import matplotlib.pyplot as plt

class TestTransitionSystem(unittest.TestCase):
    def setUp(self):
        # The ltl2dfa library has some issues with not closing files, which
        # we'll just ignore for now.
        warnings.simplefilter('ignore', category=ResourceWarning)

    def test_construction(self):
        ts = TransitionSystem(2)

        partition_one = HPolyhedron.MakeBox([0,0],[2,2]) 
        label_one = ["a"]
        vertex_one = ts.AddPartition(partition_one, label_one)

        partition_two = HPolyhedron.MakeBox([0,2],[2,4])
        label_two = ["a","b"]
        vertex_two = ts.AddPartition(partition_two, label_two)

        ts.AddEdge(vertex_one, vertex_two)

        self.assertEqual(ts.vertices, [0,1])
        self.assertEqual(ts.edges, [(0,1)])
        self.assertEqual(ts.labels, {0:["a"], 1:["a", "b"]})
        self.assertEqual(ts.partitions, {0:partition_one, 1:partition_two})

    def test_plot(self):
        ts = TransitionSystem(2)
        partition_one = HPolyhedron.MakeBox([0,0],[2,2]) 
        label_one = ["a"]
        vertex_one = ts.AddPartition(partition_one, label_one)
        partition_two = HPolyhedron.MakeBox([1,2],[3,4])
        label_two = ["a","b"]
        vertex_two = ts.AddPartition(partition_two, label_two)
        ts.AddEdge(vertex_one, vertex_two)

        ts.visualize()

        plt.show()

    def test_satisfies(self):
        self.assertTrue(satisfies(["a"], "a | b"))
        self.assertTrue(satisfies(["a", "b"], "a | b"))
        self.assertTrue(satisfies(["a", "b"], "a & b"))
        self.assertTrue(satisfies([], "a | b | ~c"))
        self.assertTrue(satisfies(["a"], "a"))
        self.assertFalse(satisfies(["a"], "b"))
        self.assertTrue(satisfies(["a"], '~b'))
        self.assertFalse(satisfies(["a", "b"], "~b"))

    def test_product(self):
        # Make a toy transition system
        ts = TransitionSystem(2)
        partition_one = HPolyhedron.MakeBox([0,0],[1,4]) 
        label_one = ["a"]
        vertex_one = ts.AddPartition(partition_one, label_one)
        
        partition_two = HPolyhedron.MakeBox([1,2],[3,4])
        label_two = ["a","b"]
        vertex_two = ts.AddPartition(partition_two, label_two)

        partition_three = HPolyhedron.MakeBox([3,2],[5,4])
        label_three = ["c"]
        vertex_three = ts.AddPartition(partition_three, label_three)
        
        partition_four = HPolyhedron.MakeBox([1,0],[5,2])
        label_four = []
        vertex_four = ts.AddPartition(partition_four, label_four)

        ts.AddEdgesFromIntersections()

        # Set up a toy specification
        string = "(~b U c) & (F b)"
        #string = "a U c"
        dfa = DeterministicFiniteAutomaton(string)

        # Create a B-spline Graph of Convex sets as the product of the
        # transition system and the specification.
        start_point = [0.5, 0.2]
        order = 3
        continuity = 2
        bgcs = ts.Product(dfa, start_point, order, continuity)

        # Solve a path planning problem on this graph
        bgcs.AddLengthCost(norm="L2")
        #bgcs.AddDerivativeCost(degree=1, weight=0.1)
        res = bgcs.SolveShortestPath(
                convex_relaxation=True,
                preprocessing=True,
                verbose=True,
                max_rounded_paths=10)
        #self.assertTrue(res.is_success())

        ts.visualize()
        bgcs.PlotSolution(res, plot_control_points=True, plot_path=True)
        bgcs.visualize()
        plt.show()


