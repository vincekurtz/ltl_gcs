import unittest

from ltlgcs.transition_system import TransitionSystem

from pydrake.geometry.optimization import HPolyhedron
import matplotlib.pyplot as plt

class TestTransitionSystem(unittest.TestCase):
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

