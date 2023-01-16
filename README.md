# Linear Temporal Logic Motion Planning via Graphs of Convex Sets

This repository contains code to accompany the paper *Temporal Logic Motion
Planning with Convex Optimization via Graphs of Convex Sets* by Vince Kurtz and
Hai Lin. 

## Installation

## Dependencies
- pydrake
- mona
- ltlf2dfa
- treelib
- unittest
- matplotlib
- scipy
- sympy
- numpy
- graphviz
- pydot
- pickle

## Examples

The following examples and several other can be found in the `examples`
directory.

### Key-door puzzle

Don't pass through a door until picking up a corresponding key, and eventually
reach a goal.

LTL specification:

$$
\varphi = (\lnot door\_1 \mathsf{U} key\_1) \land (\lnot door\_2 \mathsf{U} key\_2) \land \lozenge goal
$$

Solution:

![](media/key_door.gif)

File: `examples/key_door.py`

### Large key-door puzzle

A similar task, but with more keys and doors. 

LTL specification:

$$
\varphi = \bigwedge\_{i=1}^5 (\lnot d\_i \mathsf{U} k\_i) \land \lozenge goal
$$

Solution:

![](media/door_puzzle.gif)

File: `examples/door_puzzle.py`

### Randomly-generated multi-target scenario

Eventually visit $a$, $c$, and $d$, and always avoid $b$:

LTL specification:

$$
\varphi = \lozenge a \land \square \lnot b \land \lozenge c \land \lozenge d
$$

Solution:

![](media/random_polytopes.gif)

File: `examples/random_polytopes.py`


