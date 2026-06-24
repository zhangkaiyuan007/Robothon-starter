#!/usr/bin/env python3
"""
Main simulation runner for the nuclear glovebox teleoperation task.

Orchestrates the full simulation loop with:
- MuJoCo physics engine
- TeleoperationController for joint commands
- SensorReader for touch/force feedback
- TaskExecutor for state machine
- Visualization hooks for overlays and rendering
"""

import os
import sys
import mujoco
import numpy as np

# Add the repository root to the Python path
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from submissions.nuclear_glovebox_teleoperation.src.teleoperation_controller import TeleoperationController
from submissions.nuclear_glovebox_teleoperation.src.sensor_utils import SensorReader
from submissions.nuclear_glovebox_teleoperation.src.task_executor import TaskExecutor
from submissions.nuclear_glovebox_teleoperation.src.visualization import (
    render_contact_points,
    render_force_feedback,
    get_camera_config
)


def run_simulation(duration: float = 30.0, render: bool = True) -> None:
    """
    Run the nuclear glovebox teleoperation simulation.

    Executes the full simulation loop for the specified duration, updating
    the controller, sensors, task state, and rendering at appropriate intervals.

    Args:
        duration: Simulation duration in seconds (default 30.0).
        render: Whether to render the scene (default True).
    """
    # Get the path to the MJCF file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mjcf_path = os.path.join(script_dir, "mjcf", "glovebox_scene.xml")

    if not os.path.exists(mjcf_path):
        print(f"Error: MJCF file not found at {mjcf_path}")
        sys.exit(1)

    # Load the model
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    data = mujoco.MjData(model)

    # Initialize controller, sensor reader, and task executor
    controller = TeleoperationController(model)
    sensor_reader = SensorReader(model)
    executor = TaskExecutor(model, data, controller, sensor_reader)

    # Initialize renderer (even if render=False, we need it for the update_scene call)
    renderer = mujoco.Renderer(model, height=480, width=640)

    # Simulation parameters
    dt = model.opt.timestep  # Timestep from MJCF (typically 0.002)
    max_steps = int(duration / dt)

    # Main simulation loop
    step_count = 0
    last_print_time = 0.0

    while not executor.is_task_complete() and step_count < max_steps:
        # Get current simulation time
        sim_time = step_count * dt

        # Print status every 500 steps (~1s of sim time at 500 Hz)
        if step_count % 500 == 0:
            elapsed_time = sim_time
            state_name = executor.get_state()
            time_in_state = executor.state_timer

            print(f"[{elapsed_time:6.2f}s] State: {state_name:8s} Time in state: {time_in_state:6.2f}s")

        # Execute one control cycle
        executor.step(dt)

        # Get control from executor and apply to simulation
        control_vec = controller.get_current_control()
        data.ctrl[:] = control_vec

        # Step the physics simulation
        mujoco.mj_step(model, data)

        # Update renderer every 10 steps (for 30fps video from 500Hz sim)
        if render and step_count % 10 == 0:
            renderer.update_scene(data)
            renderer.render()

        step_count += 1

    # Print final summary
    final_state = executor.get_state()
    final_time = step_count * dt
    is_complete = executor.is_task_complete()

    print("\n✓ Simulation completed")
    print(f"  Total time: {final_time:.1f}s")
    print(f"  Final state: {final_state}")
    print(f"  Task complete: {is_complete}")


if __name__ == "__main__":
    run_simulation(duration=30.0, render=False)
