class DirectedGraph:
    """
    A simple directed graph.
    """
    def __init__(self, vertices, edges):
        """
        Create a directed graph from the given vertices and edges.

        Args:
            vertices: a list of unique integers specifying each vertex
            edges: a list of pairs of integers specifying connections between
                   vertices. 
        """
        # verify that each edge connects valid vertices
        for edge in edges:
            assert edge[0] in vertices 
            assert edge[1] in vertices 

        self.vertices = vertices
        self.edges = edges

    def add_vertex(self, v):
        """
        Add a new vertex to the graph

        Args:
            v: integer index of the new vertex
        """
        assert v not in self.vertices, "vertex already exists"
        self.vertices.append(v)

    def add_edge(self, e):
        """
        Add a new edge to the graph

        Args:
            e: tuple (source vertex, target vertex) of the edge to add
        """
        assert e[0] in self.vertices, "source vertex doesn't exist"
        assert e[1] in self.vertices, "target vertex doesn't exist"

        self.edges.append(e)

    def nv(self):
        """
        Return the number of vertices in this graph
        """
        return len(self.vertices)

    def ne(self):
        """
        Return the number of edges in this graph
        """
        return len(self.edges)

    def __str__(self):
        return f"directed graph with {self.nv()} vertices and {self.ne()} edges."

