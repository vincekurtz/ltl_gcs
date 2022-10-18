from ltlgcs.graph import DirectedGraph

from ltlf2dfa.parser.ltlf import LTLfParser
import graphviz
import pydot
import re

class DeterministicFiniteAutomaton(DirectedGraph):
    """
    Representation of a Deterministic Finite Automaton, equivalent to a co-safe
    linear temporal logic specification.

    A DFA is composed of:

        - States Q (a.k.a. vertices)
        - Transitions between states (a.k.a. edges)
        - Labels for each transition
        - An initial state
        - A set of accepting states
    """
    def __init__(self, ltl_string):
        """
        Construct a DFA from the given (co-safe) LTL specification.

        Args:
            ltl_string: string representing the LTL specification, following
                        ltlf2dfa syntax. 
        """
        # Compute a corresponding LTL formula and predicates
        parser = LTLfParser()
        self.ltl_formula = parser(ltl_string)
        self.predicates = self.ltl_formula.find_labels()

        # Convert to pydot format
        self.dot_string = self.ltl_formula.to_dfa()

        pydot_graphs = pydot.graph_from_dot_data(self.dot_string)
        assert len(pydot_graphs) == 1, "graphviz string resulted in > 1 graph"
        pydot_graph = pydot_graphs[0]

        # Extract vertices, edges, initial vertex, labels, and accepting states
        self.vertices = []  # list of vertices (represented as integer indeces)
        self.edges = []     # list of pairs of vertex indeces, e.g., (1,3)

        self.predicates = []  # List of strings for each predicate
        self.labels = {}      # Maps edges to predicates, e.g., (1,3)->"a"

        self.initial_vertex = None
        self.accepting_vertices = []

        for edge in pydot_graph.get_edges():
            source_name = edge.get_source()
            destination_name = edge.get_destination()

            # Check if this edge denotes the initial state
            if source_name == "init":
                # Add the destination to our list of vertices, if it's not
                # alread included
                if int(destination_name) not in self.vertices:
                    self.vertices.append(int(destination_name))

                # Set the destination as the initial state
                self.initial_vertex = int(destination_name)
            else:
                # Check if source and destination are in our list of vertices yet.
                # If not, add them. 
                if int(source_name) not in self.vertices:
                    self.vertices.append(int(source_name))
                if int(destination_name) not in self.vertices:
                    self.vertices.append(int(destination_name))
                        
                # Add this edge to our list of edges
                edge_tuple = (int(source_name), int(destination_name))
                self.edges.append(edge_tuple)

                # Add a label for this edge
                self.labels[edge_tuple] = edge.get_attributes()["label"]

        # Find the accepting states
        accepting_states_line = self.dot_string.split('\n')[6]
        m = re.findall(r"; ([0-9]*)", accepting_states_line)
        self.accepting_vertices = [int(vertex) for vertex in m]

    def visualize(self):
        # We'll over-ride the visualization using the dot string that
        # was provided by the LTLf2DFA utility. 
        s = graphviz.Source(self.dot_string)
        s.render('/tmp/my_dfa.gv', format='jpg', view=True)

