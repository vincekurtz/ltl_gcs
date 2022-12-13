from pydrake.all import *
import numpy as np
import time
import pickle

from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

##
#
# A temporal logic motion planning example for a high-DoF robot arm. The robot
# arm must press a button before reaching through a doorway. 
#
##

# Several important configurations, could be defined via inverse dynamics
q0 = np.array([-0.3, 0.5, -0.5, -2.1, 0., 2.3, 0.8])     # initial
q1 = np.array([-0.4, 0.8, -0.7, -1.6, 0, 2.3, 0.8])      # door
q2 = np.array([-0.9, 0.4, -0.43, -2.6, 0.45, 2.2, 0.8])
q3 = np.array([-1.5, 0.3, -0.35, -2.5, 0.9, 2.1, 0.8])
q4 = np.array([-2.6, 0.9, 0, -1.3, 1.8, 1.9, 0.8])       # button

# Create a MultibodyPlant model of the system
builder = DiagramBuilder()

plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.package_map().Add("ltl_gcs", "./")
directives = LoadModelDirectives("examples/models/robot_arm.yaml")
models = ProcessModelDirectives(directives, plant, parser)
plant.Finalize()

ctrl = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
builder.Connect(ctrl.get_output_port(0), plant.get_actuation_input_port())

params = DrakeVisualizerParams(role=Role.kIllustration)
DrakeVisualizer().AddToBuilder(builder=builder, scene_graph=scene_graph,
        params=params)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

# Construct convex sets using IRIS. This can be quite slow, so we do it offline
# and save the results. 
perform_iris = False
if perform_iris:
    seeds = {"q0":q0, "q1":q1, "q2":q2, "q3":q3, "q4":q4}
    for name, configuration in seeds.items():
        plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
        plant.SetPositions(plant_context, configuration)

        # N.B. we use pretty conservative IRIS options here, resulting in a
        # significant underapproximation of the convex sets. This requires more
        # sample points, but limits compute time. 
        iris_options = IrisOptions()
        iris_options.require_sample_point_is_contained = True
        iris_options.iteration_limit = 1
        iris_options.termination_threshold = 2e-2
        iris_options.relative_termination_threshold = 2e-2
        iris_options.num_collision_infeasible_samples = 1

        start_time = time.time()
        hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)

        print(f"Generated a collision-free polytope around {name} with {len(hpoly.b())} faces in {time.time()-start_time} seconds")

        with open(f"examples/iris_region_{name}.pkl", "wb") as f:
            pickle.dump(hpoly,f)

# Load saved convex decomposition of free space
with open("examples/iris_region_q0.pkl", "rb") as f:
    q0_region = pickle.load(f)
with open("examples/iris_region_q1.pkl", "rb") as f:
    q1_region = pickle.load(f)
with open("examples/iris_region_q2.pkl", "rb") as f:
    q2_region = pickle.load(f)
with open("examples/iris_region_q3.pkl", "rb") as f:
    q3_region = pickle.load(f)
with open("examples/iris_region_q4.pkl", "rb") as f:
    q4_region = pickle.load(f)

# Construct a labeled transition system
ts = TransitionSystem(7)
ts.AddPartition(q0_region, [])
ts.AddPartition(q1_region, [])
ts.AddPartition(q2_region, [])
ts.AddPartition(q3_region, [])
ts.AddPartition(q4_region, [])
ts.AddPartition(
        HPolyhedron.MakeBox(q1,q1), ["door"])
ts.AddPartition(
        HPolyhedron.MakeBox(q4,q4), ["button"])
ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
spec = "(~door U button) & (F door)"
#spec = "(F door)"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system
order = 3
continuity = 1
product_start_time = time.time()
bgcs = ts.Product(dfa, q0, order, continuity)
product_time = time.time() - product_start_time

# Solve the planning problem
bgcs.AddLengthCost(norm="L2_squared")
solve_start_time = time.time()
res = bgcs.SolveShortestPath(
        convex_relaxation=True,
        preprocessing=True,
        verbose=True,
        max_rounded_paths=10,
        solver="mosek")
solve_time = time.time() - solve_start_time

if res.is_success():
    # Print timing infos
    print("\n")
    print("Solve Times:")
    print("    LTL --> DFA    : ", dfa_time)
    print("    TS x DFA = GCS : ", product_time)
    print("    GCS solve      : ", solve_time)
    print("    Total          : ", dfa_time + product_time + solve_time)
    print("")

    print("GCS vertices: ", bgcs.nv())
    print("GCS edges: ", bgcs.ne())

    # Play back the trajectory
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
    dt = 0.02
    t = 0
    path = bgcs.ExtractSolution(res)
    for segment in path:
        for s in np.linspace(0,1,50):
            q = segment.value(s)
            plant.SetPositions(plant_context, q)
            diagram_context.SetTime(t)
            diagram.ForcedPublish(diagram_context)
            time.sleep(dt)
            t += dt
    

