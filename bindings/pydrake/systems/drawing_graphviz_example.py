"""Shows a quick demo of pydrake.systems.drawing.

To see this interactively:
  bazel run //bindings/pydrake/systems:drawing_graphviz_example
"""

# We must render the figures somehow.
# The Agg backend (creating a PNG image) is suitably cross-platform.
# Users should feel free to use a different back-end in their own code.
import os
os.environ['MPLBACKEND'] = 'Agg'  # noqa

# Now that the environment is set up, it's safe to import matplotlib, etc.
import matplotlib.pyplot as plt
import webbrowser

from pydrake.common import FindResourceOrThrow
from pydrake.geometry import DrakeVisualizer, SceneGraph
from pydrake.lcm import DrakeLcm
from pydrake.multibody import MultibodyPlant, Parser
from pydrake.systems.drawing import plot_graphviz, plot_system_graphviz
from pydrake.systems.framework import DiagramBuilder

# If running under `bazel run`, output to cwd to the user can find it.
# If running under `bazel test` avoid polluting the test's cwd.
for env_name in ['BUILD_WORKING_DIRECTORY', 'TEST_TMPDIR']:
    if env_name in os.environ:
        os.chdir(os.environ[env_name])

file_name = FindResourceOrThrow(
    "drake/examples/multibody/cart_pole/cart_pole.sdf")
builder = DiagramBuilder()
scene_graph = builder.AddSystem(SceneGraph())
cart_pole = builder.AddSystem(MultibodyPlant(0.0))
cart_pole.RegisterAsSourceForSceneGraph(scene_graph)
Parser(plant=cart_pole).AddModelFromFile(file_name)

plt.figure(figsize=(11, 8.5), dpi=300)
plot_graphviz(cart_pole.GetTopologyGraphvizString())
plt.savefig('cart_pole_topology.png')
assert os.path.exists('cart_pole_topology.png')

cart_pole.Finalize()
assert cart_pole.geometry_source_is_registered()

builder.Connect(
    scene_graph.get_query_output_port(),
    cart_pole.get_geometry_query_input_port())
builder.Connect(
    cart_pole.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(cart_pole.get_source_id()))
builder.ExportInput(cart_pole.get_actuation_input_port())

DrakeVisualizer.AddToBuilder(builder=builder, scene_graph=scene_graph)

diagram = builder.Build()
diagram.set_name("graphviz_example")

plt.figure(figsize=(11, 8.5), dpi=300)
plot_system_graphviz(diagram, max_depth=2)
plt.savefig('drawing_graphviz_example.png')
assert os.path.exists('drawing_graphviz_example.png')

# Show the figures (but not when testing).
if 'TEST_TMPDIR' not in os.environ:
    webbrowser.open_new_tab(url='cart_pole_topology.png')
    webbrowser.open_new_tab(url='drawing_graphviz_example.png')
