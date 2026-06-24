"""
Sensor reading utilities for the nuclear glovebox teleoperation environment.
Provides the SensorReader class to interpret touch/force data from hand sensors.
"""

import numpy as np
import mujoco
from typing import Dict, List, Optional


class SensorReader:
    """
    Reads touch and force data from hand sensors in the MJCF environment.

    Maintains lists of touch sensor sites for left and right hands,
    and provides methods to query contact forces and positions.
    """

    def __init__(self, model: mujoco.MjModel):
        """
        Initialize the SensorReader with the MuJoCo model.

        Args:
            model: The MuJoCo model object containing the environment.
        """
        self.model = model

        # Hardcoded touch site names for left hand
        self.left_touch_sites = [
            "left_thumb_touch",
            "left_thumb_touch_1",
            "left_index_touch",
            "left_index_touch_1",
            "left_middle_touch",
            "left_middle_touch_1",
            "left_ring_touch",
            "left_ring_touch_1"
        ]

        # Hardcoded touch site names for right hand
        self.right_touch_sites = [
            "right_thumb_touch",
            "right_thumb_touch_1",
            "right_index_touch",
            "right_index_touch_1",
            "right_middle_touch",
            "right_middle_touch_1",
            "right_ring_touch",
            "right_ring_touch_1"
        ]

    def get_touch_sites(self, hand: str) -> List[str]:
        """
        Get the list of touch sensor site names for the specified hand.

        Args:
            hand: Either "left" or "right".

        Returns:
            List of touch site names (8 sites per hand).
        """
        if hand == "left":
            return self.left_touch_sites
        elif hand == "right":
            return self.right_touch_sites
        else:
            raise ValueError(f"Invalid hand: {hand}. Must be 'left' or 'right'.")

    def read_forces(self, data: mujoco.MjData, threshold: float = 0.01) -> Dict[str, bool]:
        """
        Check which touch sites have contact forces above the threshold.

        Args:
            data: The MuJoCo data object (simulation state).
            threshold: Force threshold in Newtons. Default 0.01 N.

        Returns:
            Dictionary mapping site name to boolean (True if contact force > threshold).
        """
        forces = {}

        # Get all touch sites
        all_sites = self.left_touch_sites + self.right_touch_sites

        # Initialize all sites as not in contact
        for site in all_sites:
            forces[site] = False

        # Check contacts
        for contact in data.contact:
            # Get geom names involved in contact
            geom1_id = contact.geom1
            geom2_id = contact.geom2

            geom1_name = self.model.geom(geom1_id).name
            geom2_name = self.model.geom(geom2_id).name

            # Check if either geom is a touch site
            for site in all_sites:
                if geom1_name == site or geom2_name == site:
                    # Calculate force magnitude
                    force_vec = data.contact_force(contact.id)
                    force_magnitude = np.linalg.norm(force_vec)

                    # Mark as in contact if force exceeds threshold
                    if force_magnitude > threshold:
                        forces[site] = True

        return forces

    def get_contact_positions(self, data: mujoco.MjData) -> Dict[str, np.ndarray]:
        """
        Get the 3D positions of all touch sites (from their site data).

        Args:
            data: The MuJoCo data object (simulation state).

        Returns:
            Dictionary mapping site name to 3D position array [x, y, z].
        """
        positions = {}

        # Get all touch sites
        all_sites = self.left_touch_sites + self.right_touch_sites

        for site in all_sites:
            try:
                site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site)
                if site_id >= 0:
                    # Get site position from data
                    pos = data.site_xpos[site_id].copy()
                    positions[site] = pos
            except:
                # Site not found, skip
                pass

        return positions
