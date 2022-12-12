from pydrake.all import *
import numpy as np

##
#
# A temporal logic motion planning example for a high-DoF robot arm
#
##

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

# Simulate the system
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
q0 = [0.0, -0.3, 0, -2.6, 0, 2.3, 0.8, 0.05, 0.05]
plant.SetPositions(plant_context, q0)

simulator = Simulator(diagram, diagram_context)
simulator.set_target_realtime_rate(1.0)
simulator.AdvanceTo(1e-4)

