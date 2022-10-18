import unittest
from ltlgcs.dfa import DeterministicFiniteAutomaton
import warnings

class TestDFA(unittest.TestCase):
    def setUp(self):
        # The ltl2dfa library has some issues with not closing files, which
        # we'll just ignore for now.
        warnings.simplefilter('ignore', category=ResourceWarning)

    def test_constructor(self):
        # Some basic sanity checks with a simple reach-avoid type specification
        string = "~b U a"
        dfa = DeterministicFiniteAutomaton(string)

        self.assertEqual(dfa.nv(), 3)
        self.assertEqual(dfa.ne(), 5)
        self.assertEqual(len(dfa.labels), dfa.ne())
        self.assertEqual(dfa.initial_vertex, 1)
        self.assertEqual(dfa.accepting_vertices, [2])

if __name__ == '__main__':
    unittest.main()

