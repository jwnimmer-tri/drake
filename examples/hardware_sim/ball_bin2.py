# SPDX-License-Identifier: MIT-0

"""
Demonstrates combining Drake with the Blender render server to create a
simulation video (or still image).
"""

import argparse
import dataclasses as dc
import os
from pathlib import Path
import time
import typing
import subprocess

from pydrake.common.value import (
    AbstractValue,
)
from pydrake.common.yaml import yaml_load_typed
from pydrake.geometry import (
    QueryObject,
)
from pydrake.geometry import (
    RenderEngineGlParams,
    MakeRenderEngineGl,
    RenderEngineGltfClientParams,
    MakeRenderEngineGltfClient,
    RenderEngineVtkParams,
    MakeRenderEngineVtk,
)
from pydrake.multibody.parsing import (
    ModelDirective,
    ModelDirectives,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    LeafSystem,
)
from pydrake.systems.sensors import (
    CameraConfig,
    RgbdSensor,
    RgbdSensorAsync,
)
from pydrake.visualization import (
    VideoWriter,
)


@dc.dataclass
class Scenario:
    """Defines the YAML format for a scenario to be simulated."""

    # The maximum simulation time (in seconds).
    simulation_duration: float = 1.0

    # Simulator configuration (integrator and publisher parameters).
    simulator_config: SimulatorConfig = SimulatorConfig()

    # All of the fully deterministic elements of the simulation.
    directives: typing.List[ModelDirective] = dc.field(default_factory=list)

    # Cameras to add to the scene.
    cameras: typing.Mapping[str, CameraConfig] = dc.field(default_factory=dict)


def _run(args):
    """Runs the demo."""
    scenario = yaml_load_typed(
        schema=Scenario,
        filename=args.scenario_file,
        defaults=Scenario())

    # Create the scene.
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlant(
        config=MultibodyPlantConfig(),
        builder=builder)
    added_models = ProcessModelDirectives(
        directives=ModelDirectives(directives=scenario.directives),
        plant=plant)
    plant.Finalize()

    # Add rendering capability to scene graph.
    gl_engine = MakeRenderEngineGl(RenderEngineGlParams())
    scene_graph.AddRenderer("gl", gl_engine)
    gltf_engine1 = MakeRenderEngineGltfClient(RenderEngineGltfClientParams(
        base_url="http://127.0.0.1:8001"))
    scene_graph.AddRenderer("gltf1", gltf_engine1)
    gltf_engine2 = MakeRenderEngineGltfClient(RenderEngineGltfClientParams(
        base_url="http://127.0.0.1:8002"))
    scene_graph.AddRenderer("gltf2", gltf_engine2)
    vtk_engine = MakeRenderEngineVtk(RenderEngineVtkParams())
    scene_graph.AddRenderer("vtk", vtk_engine)

    # Add the camera(s).
    video_writers = []
    capture_offset = 0
    output_delay = 0.024
    for _, camera in scenario.cameras.items():
        name = camera.name
        X_WB = camera.X_PB.GetDeterministicValue()
        frame_id = plant.GetBodyFrameIdIfExists(plant.world_body().index())
        color, depth = camera.MakeCameras()
        if capture_offset == 0.0 and output_delay == 0.0:
            sensor = builder.AddSystem(RgbdSensor(
                frame_id, X_WB, color, depth))
        else:
            sensor = builder.AddSystem(RgbdSensorAsync(
                scene_graph=scene_graph, parent_id=frame_id, X_PB=X_WB,
                color_camera=color, depth_camera=None, fps=camera.fps,
                capture_offset=capture_offset, output_delay=output_delay))
        builder.Connect(scene_graph.get_query_output_port(),
                        sensor.GetInputPort("geometry_query"))
        writer = builder.AddSystem(
            VideoWriter(filename=f"{name}.mp4", fps=camera.fps, backend="cv2"))
        writer.ConnectRgbdSensor(builder=builder, sensor=sensor)
        video_writers.append(writer)
        if output_delay > 0:
            capture_offset += 0.025

    # Create the simulator.
    simulator = Simulator(builder.Build())
    ApplySimulatorConfig(scenario.simulator_config, simulator)

    # Simulate.
    start_time = time.time()
    print("RUNNING")
    simulator.AdvanceTo(scenario.simulation_duration)
    end_time = time.time()
    print(f"FINISHED after {(end_time-start_time):.1f} seconds")
    for writer in video_writers:
        writer.Save()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    args = parser.parse_args()

    scenario_file = "examples/hardware_sim/ball_bin.yaml"
    setattr(args, "scenario_file", scenario_file)
    _run(args)


if __name__ == "__main__":
    # Create output files in $PWD, not runfiles.
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])
    main()
