from ltlgcs.graph import DirectedGraph

from pydrake.geometry.optimization import (ConvexSet, VPolytope, 
                                           GraphOfConvexSets,
                                           GraphOfConvexSetsOptions)
from pydrake.solvers import (GurobiSolver, MosekSolver, CommonSolverOption,
                             SolverOptions)

from pydrake.all import eq, le, ge

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
    def __init__(self, vertices, edges, regions):
        """
        Construct a graph of convex sets

        Args:
            vertices: list of integers representing each vertex in the graph
            edges: list of pairs of integers (vertices) for each edge
            regions: dictionary mapping each vertex to a Drake ConvexSet
        """
        # General graph constructor
        super().__init__(vertices, edges)

        # Check that the regions correspond to valid vertices, and valid convex
        # sets
        for vertex, region in regions.items():
            assert vertex in self.vertices, "invalid vertex index"
            assert isinstance(region, ConvexSet), "regions must be convex sets"

        self.regions = regions

    def AddLengthCost(self):
        pass

    def AddDerivativeCost(self):
        pass

    def SolveShortestPath(self):
        order = 2
        dim = self.regions[self.vertices[0]].ambient_dimension()

        # Set options
        options = GraphOfConvexSetsOptions()
        options.convex_relaxation = True
        options.preprocessing = False
        options.solver = GurobiSolver()
        options.solver_options = SolverOptions()
        options.solver_options.SetOption(CommonSolverOption.kPrintToConsole, 1)
       
        # Create the graph
        gcs = GraphOfConvexSets()

        gcs_verts = {}  # map our vertices to GCS vertices
        for v in self.vertices:
            gcs_verts[v] = gcs.AddVertex(
                    self.regions[v].CartesianPower(order + 1))

        for e in self.edges:
            # Get vertex IDs of source and target for this edge
            u_id = gcs_verts[e[0]].id()
            v_id = gcs_verts[e[1]].id()

            gcs.AddEdge(u_id, v_id)
        
        # TODO: set source and target properly
        source = gcs_verts[1].id()
        target = gcs_verts[3].id()

        # Add edge costs
        for e in gcs.Edges():
            source_final_control_point = e.xu()[-dim:]
            target_first_control_point = e.xv()[:dim]
            #edge_cost = (e.xu() - e.xv()).dot(e.xu() - e.xv())
            edge_cost = (source_final_control_point - target_first_control_point).dot(source_final_control_point - target_first_control_point)
            e.AddCost(edge_cost)

        # Add edge constraints

        # Add initial condition constraint
        # TODO: consider defining starting point as a new vertex in the graph
        start_vertex = gcs_verts[1]
        start_state = np.array([1.0, 1.0])
        start_vertex.AddConstraint(start_vertex.x()[0] == start_state[0])
        start_vertex.AddConstraint(start_vertex.x()[1] == start_state[1])
        
        

        # Solve the problem
        res = gcs.SolveShortestPath(source, target, options)
        assert res.is_success(), "Optimization failed!"

        # Extract the solution
        for edge in gcs.Edges():
            phi = res.GetSolution(edge.phi())
            xu = res.GetSolution(edge.xu())
            xv = res.GetSolution(edge.xv())
            print(f"phi : {phi}, xu : {xu}, xv : {xv}")

            if phi > 0.999:
                # DEBUG: plot control points
                u_control_points = xu.reshape(order+1, -1)
                v_control_points = xv.reshape(order+1, -1)

                print(u_control_points)

                plt.plot(u_control_points[:,0], u_control_points[:,1], 'o-',
                        color='red')
                plt.plot(v_control_points[:,0], v_control_points[:,1], 'o-',
                        color='red')

        # DEBUG: plot the scenario
        self.PlotScenario()
        plt.show()


    
    def PlotScenario(self):
        """
        Add a plot of each region to the current matplotlib axes. 
        Only supports 2D polytopes for now.
        """
        for region in self.regions.values():
            assert region.ambient_dimension() == 2, "only 2D sets allowed"
            # Compute vertices of the polygon in known order
            v = VPolytope(region).vertices().T
            hull = ConvexHull(v)
            v_sorted = np.vstack([v[hull.vertices,0],v[hull.vertices,1]]).T

            # Make a polygonal patch
            poly = Polygon(v_sorted, alpha=0.5, edgecolor="k", linewidth=3)
            plt.gca().add_patch(poly)
       
        # Use equal axes so square things look square
        plt.axis('equal')

    def PlotSolution(self):
        pass


