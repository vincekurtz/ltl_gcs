from ltlgcs.graph import DirectedGraph

from pydrake.all import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

class BSplineGraphOfConvexSets(DirectedGraph):
    """
    Problem setup and solver for planning a B-Spline trajectory through a graph
    of convex sets. The graph setup is as follows:

        - Each vertex is associated with a convex set
        - Each convex set contains a Bezier curve
        - The optimal path is a B-spline made up of Bezier curves. These curves
          must line up with each other. 
        - The goal is to find a (minimum cost) trajectory from a given starting
          point to the target vertex
        - The target vertex is not associated with any constraints on the
          B-splines: it just indicates that the task is complete.
    """
    def __init__(self, vertices, edges, regions, start_vertex, end_vertex,
                 start_point, order=2, continuity=1):
        """
        Construct a graph of convex sets

        Args:
            vertices:      list of integers representing each vertex in the graph
            edges:         list of pairs of integers (vertices) for each edge
            regions:       dictionary mapping each vertex to a Drake ConvexSet
            start_vertex:  index of the starting vertex
            end_vertex:    index of the end/target vertex
            start_point:   initial point of the path
            order:         order of bezier curve under consideration
            continuity:    number of continuous derivatives of the curve
        """
        # General graph constructor
        super().__init__(vertices, edges)

        # Dimensionality of the problem is defined by the starting point
        assert regions[start_vertex].PointInSet(start_point)
        self.start_point = start_point
        self.dim = len(start_point)

        # Check that the regions correspond to valid vertices and valid convex
        # sets
        for vertex, region in regions.items():
            assert vertex in self.vertices, "invalid vertex index"
            assert isinstance(region, ConvexSet), "regions must be convex sets"
            assert region.ambient_dimension() == self.dim
        self.regions = regions

        # Validate the start and target vertices
        assert start_vertex in vertices
        assert end_vertex in vertices
        self.start_vertex = start_vertex
        self.end_vertex = end_vertex
       
        # B-splines can guarantee continuity of n-1 derivatives
        assert continuity < order
        self.order = order
        self.continuity = continuity

        # Create "dummy" symbolic bsplines for an arbitrary edge. This allows us
        # to derive expressions for various things, such as derivatives of the
        # spline, in terms of the original control points (decision variables)
        self.dummy_xu = MakeMatrixContinuousVariable(self.order+1, self.dim, "xu")
        self.dummy_xv = MakeMatrixContinuousVariable(self.order+1, self.dim, "xv")
        self.dummy_edge_vars = np.concatenate((
            self.dummy_xu.flatten(), self.dummy_xv.flatten()))

        self.dummy_path_u = BsplineTrajectory_[Expression](
                BsplineBasis_[Expression](self.order+1, self.order+1, 
                    KnotVectorType.kClampedUniform, 0, 1),
                self.dummy_xu.T)
        self.dummy_path_v = BsplineTrajectory_[Expression](
                BsplineBasis_[Expression](self.order+1, self.order+1, 
                    KnotVectorType.kClampedUniform, 0, 1),
                self.dummy_xv.T)

        # Create the GCS problem
        self.gcs = GraphOfConvexSets()
        self.source, self.target = self.SetupShortestPathProblem()

    def AddLengthCost(self, weight=1.0, norm="L2"):
        """
        Add a penalty on the distance between control points, which is an
        overapproximation of total path length.

        Args:
            weight: Weight for this cost, scalar.
            norm: Norm to use to when evaluating distance between control
                  points. See AddDerivativeCost for details.
        """
        self.AddDerivativeCost(0, weight=weight, norm=norm)

    def AddDerivativeCost(self, degree, weight=1.0, norm="L2"):
        """
        Add a penalty on the derivative of the path. We do this by applying an
        approximate length cost to the derivative of the path, e.g., on the distance 
        between control points of the derivative.

        There are several norms we can use to approximate these lenths:
            L1 - most computationally efficient, and introduces only linear
            costs and constraints to the GCS problem.

            L2 - closest approximation to the actual curve length, introduces
            cone constraints to the GCS problem.

            L2_squared - incentivises evenly space control points, which can
            produce some nice smooth-looking curves. Introduces cone constraints
            to the GCS problem. 
        
        Args:
            degree: The derivative to penalize. degree=0 is the original
                    trajectory, degree=1 is the first derivative, etc. 
            weight: Weight for this cost, scalar
            norm:   Norm to use to when evaluating distance between control
                    points (see above)
        """
        assert norm in ["L1", "L2", "L2_squared"], "invalid length norm"
        assert degree >= 0
        assert degree < self.order

        # Get a symbolic version of the i^th derivative of a segment
        path_deriv = self.dummy_path_u.MakeDerivative(degree)

        # Get a symbolic expression for the difference between subsequent
        # control points in the i^th derivative, in terms of the original
        # decision variables
        deriv_control_points = path_deriv.control_points()

        A = []
        for i in range(self.order - degree):
            diff = deriv_control_points[i] - deriv_control_points[i+1]
            A.append(DecomposeLinearExpressions(diff.flatten(),
                        self.dummy_xu.flatten()))

        # Apply a cost to the starting segment of each edge. 
        for edge in self.gcs.Edges():
            x = edge.xu()
            for i in range(self.order - degree):
                if norm == "L1":
                    cost = L1NormCost(weight*A[i], np.zeros(self.dim))
                elif norm == "L2":
                    cost = L2NormCost(weight*A[i], np.zeros(self.dim))
                else:  # L2 squared
                    cost = QuadraticCost(
                            Q=weight*A[i].T@A[i], b=np.zeros(len(x)), c=0.0)
                edge.AddCost(Binding[Cost](cost, x))

    def SetupShortestPathProblem(self):
        """
        Formulate a shortest path through convex sets problem where the path is
        composed of bezier curves that must be contained in each convex set.

        Returns:
            source: Drake Gcs Vertex corresponding to the initial convex set
            target: Drake Gcs Vertex corresponding to the final convex set
        """
        # Define vertices. The convex sets for each vertex are such that each
        # control point must be contained in the corresponding region
        gcs_verts = {}  # map our vertices to GCS vertices
        for v in self.vertices:
            gcs_verts[v] = self.gcs.AddVertex(
                    self.regions[v].CartesianPower(self.order + 1))
        
        # Define edges
        for e in self.edges:
            # Get vertex IDs of source and target for this edge
            u_id = gcs_verts[e[0]].id()
            v_id = gcs_verts[e[1]].id()

            self.gcs.AddEdge(u_id, v_id)
       
        # Define source and target vertices
        source = gcs_verts[self.start_vertex]
        target = gcs_verts[self.end_vertex]

        # Add continuity constraints. This includes both continuity (first and
        # last control points line up) and smoothness (first and last control
        # points of derivatives of the path line up).
        for i in range(self.continuity + 1):
            # N.B. i=0 corresponds to the path itsefl
            dummy_path_u_deriv = self.dummy_path_u.MakeDerivative(i)
            dummy_path_v_deriv = self.dummy_path_v.MakeDerivative(i)

            continuity_err = dummy_path_v_deriv.control_points()[0] - \
                             dummy_path_u_deriv.control_points()[-1]

            continuity_constraint = LinearEqualityConstraint(
                    DecomposeLinearExpressions(
                        continuity_err, 
                        self.dummy_edge_vars),
                    np.zeros(self.dim))

            # Apply the continuity constraints to each edge in the graph
            for edge in self.gcs.Edges():
                if edge.v() != target:
                    edge.AddConstraint(Binding[Constraint](
                        continuity_constraint,
                        np.concatenate((edge.xu(), edge.xv()))))
        
        # Add initial condition constraint
        for i in range(self.dim):
            source.AddConstraint(source.x()[i] == self.start_point[i])

        # Allow access to GCS vertices later
        self.gcs_verts = gcs_verts

        return (source, target)

    def SolveShortestPath(self, verbose=True, convex_relaxation=False,
            preprocessing=True, max_rounded_paths=0, solver="mosek"):
        """
        Solve the shortest path problem (self.gcs).

        Args:
            verbose: whether to print solver details to the screen
            convex_relaxation: whether to solve the original MICP or the convex
                               relaxation (+rounding)
            preprocessing: preprocessing step to reduce the size of the graph
            max_rounded_paths: number of distinct paths to compare during
                               rounding for the convex relaxation
            solver: underling solver for the CP/MICP. Must be "mosek" or
                    "gurobi"

        Returns:
            result: a MathematicalProgramResult encoding the solution.
        """
        # Set solver options
        options = GraphOfConvexSetsOptions()
        options.convex_relaxation = convex_relaxation
        options.preprocessing = preprocessing
        options.max_rounded_paths = max_rounded_paths
        if solver == "mosek":
            options.solver = MosekSolver()
        elif solver == "gurobi":
            options.solver = GurobiSolver()
        else:
            raise ValueError(f"Unknown solver {solver}")
        solver_opts = SolverOptions()
        solver_opts.SetOption(CommonSolverOption.kPrintToConsole, verbose)
        options.solver_options = solver_opts
        
        # Solve the problem
        result = self.gcs.SolveShortestPath(self.source, self.target, options)

        return result
    
    def PlotScenario(self):
        """
        Add a plot of each region to the current matplotlib axes. 
        Only supports 2D polytopes for now.
        """
        for vertex, region in self.regions.items():
            assert region.ambient_dimension() == 2, "only 2D sets allowed"

            if vertex != self.end_vertex:
                # The target vertex is trivial and therefore not plotted

                # Compute vertices of the polygon in known order
                v = VPolytope(region).vertices().T
                hull = ConvexHull(v)
                v_sorted = np.vstack([v[hull.vertices,0],v[hull.vertices,1]]).T

                # Make a polygonal patch
                poly = Polygon(v_sorted, alpha=0.5, edgecolor="k", linewidth=3)
                plt.gca().add_patch(poly)
       
        # Use equal axes so square things look square
        plt.axis('equal')

    def PlotSolution(self, result, plot_control_points=True, plot_path=True):
        """
        Add a plot of the solution to the current matplotlib axes. Only
        supported for 2D. 

        Args:
            result: MathematicalProgramResult from calling SolveShortestPath
            plot_control_points: flag for plotting the control points
            plot_path: flag for plotting the actual path
        """
        for edge in self.gcs.Edges():
            # Note that focusing on xu for each edge ignores the target
            # state, since that's just an indicator of task completion
            phi = result.GetSolution(edge.phi())
            xu = result.GetSolution(edge.xu())

            if phi > 0.0:
                # Construct a bezier curve from the control points
                control_points = xu.reshape(self.order+1, -1)
                basis = BsplineBasis(self.order+1, self.order+1, 
                                     KnotVectorType.kClampedUniform, 0, 1)
                path = BsplineTrajectory(basis, control_points.T)
                
                if plot_control_points:
                    plt.plot(control_points[:,0], control_points[:,1], 'o--',
                            color='red', alpha=phi)

                if plot_path:
                    curve = path.vector_values(np.linspace(0,1))
                    plt.plot(curve[0,:], curve[1,:], color='blue', linewidth=3,
                            alpha=phi)

