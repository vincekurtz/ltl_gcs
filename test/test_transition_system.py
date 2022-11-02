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

    def test_product(self):
        # Make a toy transition system
        ts = TransitionSystem(2)
        partition_one = HPolyhedron.MakeBox([0,0],[2,2]) 
        label_one = ["a"]
        vertex_one = ts.AddPartition(partition_one, label_one)
        partition_two = HPolyhedron.MakeBox([1,2],[3,4])
        label_two = ["a","b"]
        vertex_two = ts.AddPartition(partition_two, label_two)
        ts.AddEdge(vertex_one, vertex_two)

        # Set up a toy specification
        string = "(F b)"
        dfa = DeterministicFiniteAutomaton(string)

        # Create a B-spline Graph of Convex sets as the product of the
        # transition system and the specification.
        start_point = [0.5, 0.2]
        order = 2
        continuity = 1
        
        bspline_gcs = ts.Product(dfa, start_point, order, continuity)
        
        #ts.visualize()
        #dfa.visualize()
        #plt.show()


