import cdd
import numpy as np
from pydrake.geometry.optimization import VPolytope, HPolyhedron

def vpoly_to_hpoly(vpoly):
    """
    Convert a polytope in vertex representation to a polytope in halfspace
    representation using cone double description.

    Args:
        vpoly: a Drake VPolytope that we wish to convert

    Returns:
        a Drake HPolyhedron representation of vpoly
    """
    assert isinstance(vpoly, VPolytope)

    # Create cdd vertex representation
    vertices = vpoly.vertices().T
    t = np.ones((vertices.shape[0], 1))  # t=1 indicates points, not rays
    tV = np.hstack([t, vertices])
    V = cdd.Matrix(tV, number_type='float')
    V.rep_type = cdd.RepType.GENERATOR

    # Convert to cdd halfspace representation
    H = cdd.Polyhedron(V)
    bA = np.array(H.get_inequalities())
    b = np.array(bA[:,0])
    A = np.array(bA[:,1:])

    # Construct Drake halfspace representation
    hpoly = HPolyhedron(-A, b)   # cdd uses A x + b >= 0

    return hpoly

