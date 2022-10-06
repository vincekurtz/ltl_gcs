import unittest
from ltlgcs.graph import DirectedGraph

class TestDirectedGraph(unittest.TestCase):
    def test_constructor(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]
        g = DirectedGraph(vertices, edges)

    def test_adding(self):
        vertices = [1,2,3]
        edges = [(1,2), (2,3)]
        g = DirectedGraph(vertices, edges)

        new_v = 4
        new_e = (2,4)
        g.add_vertex(new_v)
        g.add_edge(new_e)

        self.assertEqual(g.nv(), 4)
        self.assertEqual(g.ne(), 3)

if __name__ == '__main__':
    unittest.main()

