#!/usr/bin/env python3
"""
Test suite for the nuclear glovebox MJCF environment.
Verifies that the MJCF loads correctly and contains required bodies and joints.
"""

import os
import sys
import xml.etree.ElementTree as ET

def test_glovebox_loads():
    """Test that the glovebox MJCF environment loads without error."""

    # Get the path to the MJCF file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mjcf_path = os.path.join(script_dir, "mjcf", "glovebox_scene.xml")

    if not os.path.exists(mjcf_path):
        print(f"Error: MJCF file not found at {mjcf_path}")
        sys.exit(1)

    # Try to load with MuJoCo if available, otherwise validate XML structure
    model = None
    try:
        import mujoco
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        print("✓ MJCF loads successfully")
    except ImportError:
        # MuJoCo not available, validate XML structure instead
        print("(MuJoCo not available, validating XML structure)")
        try:
            tree = ET.parse(mjcf_path)
            print("✓ MJCF loads successfully (XML valid)")
        except ET.ParseError as e:
            print(f"Error: MJCF XML is invalid: {e}")
            sys.exit(1)
    except Exception as e:
        print(f"Error loading MJCF: {e}")
        sys.exit(1)

    # Check that required bodies exist
    required_bodies = [
        "glovebox_left_hand",
        "glovebox_right_hand",
        "sample_vial",
        "shielded_container"
    ]

    # Use MuJoCo if available, otherwise parse XML
    if model is not None:
        body_names = [model.body(i).name for i in range(model.nbody)]
        found_bodies = [b for b in required_bodies if b in body_names]
        num_dof = model.nq
        joint_names = [model.joint(i).name for i in range(model.njnt)]
    else:
        # Parse XML to extract information
        tree = ET.parse(mjcf_path)
        root = tree.getroot()

        bodies = root.findall(".//body")
        body_names = [b.get("name") for b in bodies if b.get("name")]
        found_bodies = [b for b in required_bodies if b in body_names]

        joints = root.findall(".//joint")
        joint_names = [j.get("name") for j in joints if j.get("name")]
        num_dof = len(joint_names)

    print(f"Bodies: {len(body_names)} (found {len(found_bodies)}/{len(required_bodies)} required)")

    missing_bodies = [b for b in required_bodies if b not in body_names]
    if missing_bodies:
        print(f"Error: Missing required bodies: {missing_bodies}")
        sys.exit(1)

    # Check DOF count (should be >= 32 for 2 hands x 16 joints)
    print(f"DOFs: {num_dof}")

    if num_dof < 32:
        print(f"Error: Expected at least 32 DOFs, found {num_dof}")
        sys.exit(1)

    # Verify joint names
    expected_joint_prefixes = [
        "left_hand_joint_",
        "right_hand_joint_"
    ]

    for prefix in expected_joint_prefixes:
        joints_with_prefix = [j for j in joint_names if j and j.startswith(prefix)]
        if len(joints_with_prefix) < 16:
            print(f"Warning: Expected at least 16 joints with prefix '{prefix}', found {len(joints_with_prefix)}")

    print("\nAll tests passed!")
    return True


if __name__ == "__main__":
    test_glovebox_loads()
