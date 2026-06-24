"""
Unit tests for the TeleoperationController class.
"""

import os
import sys
import numpy as np

# Add the repository root to the Python path
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import mujoco
from teleoperation_controller import TeleoperationController


def test_controller_init():
    """
    Test that TeleoperationController initializes correctly with a 32-dimensional
    control vector of all zeros.
    """
    # Load the model from the MJCF file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "..", "mjcf", "glovebox_scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)

    # Create the controller
    controller = TeleoperationController(model)

    # Verify control vector shape and values
    control = controller.get_current_control()
    assert control.shape == (32,), f"Expected shape (32,), got {control.shape}"
    assert np.allclose(control, 0.0), f"Expected all zeros, got {control}"

    print("✓ TeleoperationController initialized correctly")


def test_controller_input():
    """
    Test that TeleoperationController correctly processes input by setting
    individual joint controls and verifying the control vector.
    """
    # Load the model from the MJCF file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, "..", "mjcf", "glovebox_scene.xml")
    model = mujoco.MjModel.from_xml_path(model_path)

    # Create the controller
    controller = TeleoperationController(model)

    # Set left hand joint 0 to 0.5
    controller.set_control("left", 0, 0.5)
    control = controller.get_current_control()
    assert control[0] == 0.5, f"Expected control[0] = 0.5, got {control[0]}"

    # Set right hand joint 0 to -0.3
    controller.set_control("right", 0, -0.3)
    control = controller.get_current_control()
    assert control[16] == -0.3, f"Expected control[16] = -0.3, got {control[16]}"

    print("✓ TeleoperationController processes input correctly")


if __name__ == "__main__":
    test_controller_init()
    test_controller_input()
