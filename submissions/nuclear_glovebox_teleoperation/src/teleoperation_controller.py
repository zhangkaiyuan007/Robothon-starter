"""
Teleoperation controller for mapping keyboard/input to joint commands.
Converts high-level input (left/right hand, joint index, value) to a 32-DOF control vector.
"""

import numpy as np
import mujoco
from typing import Dict


class TeleoperationController:
    """
    Maps input commands to joint-level control for dual-hand manipulation.

    Maintains a 32-dimensional control vector:
    - Indices 0-15: left hand (16 DOF)
    - Indices 16-31: right hand (16 DOF)

    All control values are clamped to [-1, 1].
    """

    def __init__(self, model: mujoco.MjModel):
        """
        Initialize the TeleoperationController.

        Args:
            model: The MuJoCo model object containing the environment.
        """
        self.model = model
        self.control_vec = np.zeros(32, dtype=np.float32)

    def set_control(self, hand: str, joint_idx: int, value: float) -> None:
        """
        Set control for a specific joint on the specified hand.

        Args:
            hand: Either "left" or "right".
            joint_idx: Joint index within the hand (0-15).
            value: Control value (clamped to [-1, 1]).

        Raises:
            ValueError: If hand is not "left" or "right", or joint_idx is out of range.
        """
        if hand not in ["left", "right"]:
            raise ValueError(f"Invalid hand: {hand}. Must be 'left' or 'right'.")
        if joint_idx < 0 or joint_idx > 15:
            raise ValueError(f"Invalid joint_idx: {joint_idx}. Must be in [0, 15].")

        # Clamp value to [-1, 1]
        clamped_value = np.clip(value, -1.0, 1.0)

        # Calculate control vector index
        if hand == "left":
            ctrl_idx = joint_idx
        else:  # hand == "right"
            ctrl_idx = 16 + joint_idx

        self.control_vec[ctrl_idx] = clamped_value

    def get_current_control(self) -> np.ndarray:
        """
        Get a copy of the current control vector.

        Returns:
            A copy of the 32-dimensional control vector.
        """
        return self.control_vec.copy()

    def reset_control(self) -> None:
        """
        Reset all control values to zero.
        """
        self.control_vec[:] = 0.0

    def apply_damping(self, data: mujoco.MjData, damping: float = 0.01) -> None:
        """
        Apply damping to the control vector (optional, not used yet).

        This method provides a placeholder for future damping logic.
        Damping can be used to smooth control signals or reduce oscillations.

        Args:
            data: The MuJoCo data object (simulation state).
            damping: Damping coefficient (default 0.01).
        """
        # Placeholder for damping logic
        pass
