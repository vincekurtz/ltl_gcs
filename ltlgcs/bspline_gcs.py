from ltlgcs.graph import DirectedGraph

from pydrake.geometry.optimization import (ConvexSet, VPolytope, 
                                           GraphOfConvexSets,
                                           GraphOfConvexSetsOptions)
from pydrake.solvers import (GurobiSolver, MosekSolver, CommonSolverOption,
                             SolverOptions)
from pydrake.all import eq, le, ge
from pydrake.math import BsplineBasis, KnotVectorType
from pydrake.trajectories import BsplineTrajectory

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
                 start_point, order):
        """
        Construct a graph of convex sets

        Args:
            vertices:      list of integers representing each vertex in the graph
            edges:         list of pairs of integers (vertices) for each edge
            regions:       dictionary mapping each vertex to a Drake ConvexSet
            start_vertex:  index of the starting vertex
            end_vertex:   index of the end/target vertex
            start_point:   initial point of the path
            order:         order of bezier curve under consideration
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

        # Create the GCS problem
        self.order = order
        self.gcs = GraphOfConvexSets()
        self.source, self.target = self.SetupShortestPathProblem()

    def AddLengthCost(self):
        pass

    def AddDerivativeCost(self):
        pass

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
        
        # Add edge constraints
        for edge in self.gcs.Edges():
            if edge.v() != target:
                # We don't add any constraints on transitions to the target
                # region, since the target vertex is a trivial extra node that
                # indicates task completion.
                u_control_points = edge.xu().reshape(self.order+1,-1)
                v_control_points = edge.xv().reshape(self.order+1,-1)

                # Last control point of the previous region must line up with the
                # first control point of the next region for continuity
                constraints = eq(u_control_points[-1,:], v_control_points[0,:])
                [edge.AddConstraint(c) for c in constraints]

                # TODO: add smoothness constraints
        
        # Add initial condition constraint
        for i in range(self.dim):
            source.AddConstraint(source.x()[i] == self.start_point[i])

        return (source, target)

    def SolveShortestPath(self, verbose=True):
        """
        Solve the shortest path problem (self.gcs).

        Args:
            verbose: whether to print solver details to the screen

        Returns:
            result: a MathematicalProgramResult encoding the solution.
        """
        # Set solver options
        options = GraphOfConvexSetsOptions()
        options.convex_relaxation = True
        options.preprocessing = False
        options.solver = GurobiSolver()
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

            if phi > 0.999:
                control_points = xu.reshape(self.order+1, -1)
                
                if plot_control_points:
                    plt.plot(control_points[:,0], control_points[:,1], 'o--', color='red')

                if plot_path:
                    # Construct a bezier curve from the control points
                    basis = BsplineBasis(self.order+1, self.order+1, 
                                         KnotVectorType.kClampedUniform, 0, 1)
                    path = BsplineTrajectory(basis, control_points)
                    curve = path.vector_values(np.linspace(0,1))
                    plt.plot(curve[0,:], curve[1,:], color='blue', linewidth=3)

