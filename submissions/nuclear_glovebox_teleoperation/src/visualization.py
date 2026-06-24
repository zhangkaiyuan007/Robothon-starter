"""
Visualization utilities for rendering overlay information and camera configuration.
Provides hooks for overlays (contact points, force feedback) and camera presets.
"""

import mujoco
import numpy as np
from typing import Dict, Optional


def render_contact_points(model: mujoco.MjModel, data: mujoco.MjData, renderer: mujoco.Renderer) -> None:
    """
    Render contact points as visual markers on the scene.

    Placeholder for adding contact point visualization overlays.
    In a full implementation, this would mark contact sites with spheres or arrows.

    Args:
        model: The MuJoCo model object.
        data: The MuJoCo data object (simulation state).
        renderer: The MuJoCo renderer instance.
    """
    # Placeholder for contact point rendering
    # Future: Add visual markers for each contact point
    pass


def render_force_feedback(data: mujoco.MjData, sensor_reader, hand: str = "left") -> str:
    """
    Generate a text overlay showing force feedback for the specified hand.

    Returns a formatted string describing contact forces and positions.

    Args:
        data: The MuJoCo data object (simulation state).
        sensor_reader: The SensorReader instance.
        hand: Either "left" or "right" (default "left").

    Returns:
        String describing force feedback state.
    """
    # Read forces for the specified hand
    forces = sensor_reader.read_forces(data)

    # Count active contacts for this hand
    hand_prefix = f"{hand}_"
    active_contacts = sum(
        1 for site in forces
        if site.startswith(hand_prefix) and forces[site]
    )

    # Get contact positions
    positions = sensor_reader.get_contact_positions(data)

    # Format output string
    output = f"[{hand.upper()} HAND] Contacts: {active_contacts}"

    return output


def get_camera_config(camera_id: int = 0) -> Dict:
    """
    Get camera configuration preset for multi-angle recording.

    Returns a dictionary with camera parameters for the specified preset.

    Args:
        camera_id: Camera preset ID (0=front, 1=top, 2=side, etc.)

    Returns:
        Dictionary with camera configuration (lookat, distance, elevation, etc.)
    """
    presets = {
        0: {
            "name": "front",
            "lookat": [0.0, 0.0, 0.5],
            "distance": 1.5,
            "elevation": 0.0,
            "azimuth": 0.0
        },
        1: {
            "name": "top",
            "lookat": [0.0, 0.0, 0.5],
            "distance": 1.5,
            "elevation": 90.0,
            "azimuth": 0.0
        },
        2: {
            "name": "side",
            "lookat": [0.0, 0.0, 0.5],
            "distance": 1.5,
            "elevation": 0.0,
            "azimuth": 90.0
        },
        3: {
            "name": "isometric",
            "lookat": [0.0, 0.0, 0.5],
            "distance": 1.5,
            "elevation": 30.0,
            "azimuth": 45.0
        }
    }

    return presets.get(camera_id, presets[0])
