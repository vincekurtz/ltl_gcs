from pydrake.all import *
import numpy as np
import time
import pickle

from ltlgcs.transition_system import TransitionSystem
from ltlgcs.dfa import DeterministicFiniteAutomaton

# Create a MultibodyPlant model of the system
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-4)
parser = Parser(plant)
parser.AddModels(FindResourceOrThrow("drake/examples/atlas/urdf/atlas_minimal_contact.urdf"))
plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("pelvis"),
        RigidTransform([0,0,0.95]))
plant.Finalize()

ctrl = builder.AddSystem(ConstantVectorSource(np.zeros(plant.num_actuators())))
builder.Connect(ctrl.get_output_port(0), plant.get_actuation_input_port())

params = DrakeVisualizerParams(role=Role.kIllustration)
DrakeVisualizer().AddToBuilder(builder=builder, scene_graph=scene_graph,
        params=params)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

# Construct a labeled transition system
ts = TransitionSystem(30)

# Construct convex sets using IRIS
iris_options = IrisOptions()

p1 = np.array([0, 1, 0.8])        # left hand target
p2 = np.array([0.5, -0.8, 1.7])   # right hand target
p3 = np.array([-0.5, -0.4, 0.5])  # right foot target
eps = 0.03  # target size
targets = {"l_hand":p1, "r_hand":p2, "r_foot":p3}

print("Generating convex set from initial position")
q0 = plant.GetPositions(plant_context)
hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)
ts.AddPartition(hpoly, [])

for name, p in targets.items():
    print(f"Generating convex set for {name} target")
    
    # Get inverse kinematics constraints for reaching the given target
    ik = InverseKinematics(plant)
    ik.AddPositionConstraint(plant.GetFrameByName(name), [0,0,0],
            plant.world_frame(), p-eps, p+eps)
    ik.AddPositionCost(plant.GetFrameByName(name), [0,0,0], 
            plant.world_frame(), p, np.eye(3))

    res = Solve(ik.prog())
    assert res.is_success()
    q = res.GetSolution(ik.q())

    # Run IRIS with the given constraints
    iris_options.prog_with_additional_constraints = ik.prog()
    plant.SetPositions(plant_context, q)
    hpoly = IrisInConfigurationSpace(plant, plant_context, iris_options)

    # Add a corresponding partition to the transition system
    ts.AddPartition(hpoly, [name])

ts.AddEdgesFromIntersections()

# Convert the specification to a DFA
spec = "F (l_hand & F (r_hand & F r_foot))"
dfa_start_time = time.time()
dfa = DeterministicFiniteAutomaton(spec)
dfa_time = time.time() - dfa_start_time

# Take the product of the DFA and the transition system
order = 5
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
