from pydrake.all import *
import numpy as np
import time
import pickle

##
#
# A temporal logic motion planning example for a high-DoF robot arm. The robot
# arm must press a button before reaching through a doorway. 
#
##

# Several important configurations, could be defined via inverse dynamics
q0 = np.array([0.0, -0.3, 0, -2.6, 0, 2.3, 0.8])
q_button = np.array([-2.6, 0.9, 0, -1.3, 1.8, 1.9, 0.8])
q_door = np.array([-0.6, 1.3, -1.0, -1.0, 0, 2.3, 0.8])

q1 = np.array([-0.4, 0.8, -0.7, -1.6, 0, 2.3, 0.8])
#q1 = 0.67*q_door + 0.33*q0
#print(q1)

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
perform_iris = True
if perform_iris:
    #seeds = {"q0":q0, "q_button":q_button, "q_door":q_door}
    seeds = {"q1":q1}
    for name, configuration in seeds.items():
        plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
        plant.SetPositions(plant_context, configuration)

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

with open("examples/iris_region_q0.pkl", "rb") as f:
    q0_region = pickle.load(f)
with open("examples/iris_region_q1.pkl", "rb") as f:
    q1_region = pickle.load(f)
with open("examples/iris_region_q_button.pkl", "rb") as f:
    q_button_region = pickle.load(f)
with open("examples/iris_region_q_door.pkl", "rb") as f:
    q_door_region = pickle.load(f)

print(q0_region.Intersection(q1_region).IsEmpty())
print(q1_region.Intersection(q_door_region).IsEmpty())



# Simulate the system
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
plant.SetPositions(plant_context, q1)

simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(1e-4)

