"""
Test for TaskExecutor initialization.
"""

import mujoco
import numpy as np
import os
from task_executor import TaskExecutor
from teleoperation_controller import TeleoperationController
from sensor_utils import SensorReader


def test_task_executor_init():
    """Test that TaskExecutor initializes correctly."""
    # Load the model
    base_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(base_dir, "../mjcf/glovebox_scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Create controller and sensor reader
    controller = TeleoperationController(model)
    sensor_reader = SensorReader(model)

    # Create executor
    executor = TaskExecutor(model, data, controller, sensor_reader)

    # Verify initial state
    assert executor.get_state() == "extract", f"Expected 'extract', got '{executor.get_state()}'"
    assert executor.is_task_complete() == False, "Task should not be complete on init"

    print("✓ TaskExecutor initializes correctly")


if __name__ == "__main__":
    test_task_executor_init()
