import unittest

import numpy as np
from ltlgcs.cdd import vpoly_to_hpoly
from pydrake.geometry.optimization import VPolytope, HPolyhedron

class TestConeDoubleDescription(unittest.TestCase):
    def test_cdd(self):
        vertices = np.array([[0,0],
                             [0,2],
                             [2,2]])
        vpoly = VPolytope(vertices.T)
        hpoly = vpoly_to_hpoly(vpoly)

        self.assertTrue(isinstance(hpoly, HPolyhedron))

        vpoly2 = VPolytope(hpoly)
        self.assertTrue(np.all(vpoly2.vertices() == vertices.T))

if __name__ == '__main__':
    unittest.main()

