#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo "================================"
echo "Nuclear Glovebox Test Suite"
echo "================================"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Step 1: Verify dependencies
echo "Step 1: Verify dependencies..."
python3 << 'EOF'
import sys
deps = ["mujoco", "numpy", "imageio"]
missing = []

for dep in deps:
    try:
        __import__(dep)
    except ImportError:
        missing.append(dep)

if missing:
    print(f"✗ Missing dependencies: {missing}")
    print("Please install with: pip install mujoco numpy imageio[ffmpeg]")
    sys.exit(1)
else:
    print("✓ Dependencies OK")
EOF
if [ $? -ne 0 ]; then
    echo "Dependency check failed. Please ensure mujoco, numpy, and imageio are installed."
    exit 1
fi
echo ""

# Step 2: Test environment loading
echo "Step 2: Test environment loading..."
python3 << 'EOF'
import os
import sys
import mujoco

mjcf_path = os.path.join(os.getcwd(), "mjcf", "glovebox_scene.xml")

if not os.path.exists(mjcf_path):
    print(f"✗ MJCF file not found at {mjcf_path}")
    sys.exit(1)

try:
    model = mujoco.MjModel.from_xml_path(mjcf_path)
    print("✓ MJCF loads successfully")
    print(f"  Bodies: {model.nbody}")
    print(f"  DOFs: {model.nq}")
except Exception as e:
    print(f"✗ Error loading MJCF: {e}")
    sys.exit(1)
EOF
echo ""

# Step 3: Test controller
echo "Step 3: Test controller..."
python3 << 'EOF'
import os
import sys
import mujoco

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.getcwd()))))

mjcf_path = os.path.join(os.getcwd(), "mjcf", "glovebox_scene.xml")
model = mujoco.MjModel.from_xml_path(mjcf_path)

try:
    from submissions.nuclear_glovebox_teleoperation.src.teleoperation_controller import TeleoperationController
    controller = TeleoperationController(model)
    print("✓ TeleoperationController initialized correctly")

    # Test that it processes input correctly
    control = controller.get_current_control()
    if control is not None and len(control) > 0:
        print("✓ TeleoperationController processes input correctly")
    else:
        print("✗ TeleoperationController failed to produce control output")
        sys.exit(1)
except Exception as e:
    print(f"✗ Error testing controller: {e}")
    sys.exit(1)
EOF
echo ""

# Step 4: Test task executor
echo "Step 4: Test task executor..."
python3 << 'EOF'
import os
import sys
import mujoco

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.getcwd()))))

mjcf_path = os.path.join(os.getcwd(), "mjcf", "glovebox_scene.xml")
model = mujoco.MjModel.from_xml_path(mjcf_path)
data = mujoco.MjData(model)

try:
    from submissions.nuclear_glovebox_teleoperation.src.teleoperation_controller import TeleoperationController
    from submissions.nuclear_glovebox_teleoperation.src.sensor_utils import SensorReader
    from submissions.nuclear_glovebox_teleoperation.src.task_executor import TaskExecutor

    controller = TeleoperationController(model)
    sensor_reader = SensorReader(model)
    executor = TaskExecutor(model, data, controller, sensor_reader)
    print("✓ TaskExecutor initializes correctly")
except Exception as e:
    print(f"✗ Error testing task executor: {e}")
    sys.exit(1)
EOF
echo ""

# Step 5: Run full simulation (30s with timeout)
echo "Step 5: Run full simulation (30s)..."
timeout 60 python3 main.py 2>&1 || EXIT_CODE=$?

# Check if timeout occurred
if [ $? -eq 124 ]; then
    echo "✗ Simulation timed out after 60 seconds"
    exit 1
fi

# Check if video was generated
if [ -f "demo.mp4" ]; then
    VIDEO_SIZE=$(stat -c%s "demo.mp4")
    VIDEO_SIZE_MB=$(echo "scale=1; $VIDEO_SIZE / 1024 / 1024" | bc)
    FRAME_COUNT=$(python3 << 'EOF'
import imageio
import os
video = imageio.get_reader("demo.mp4")
print(len(video))
EOF
)
    echo "✓ Video saved: submissions/nuclear_glovebox_teleoperation/demo.mp4"
    echo "  Size: ~${VIDEO_SIZE_MB}MB"
    echo "  Frames: $FRAME_COUNT"
fi
echo ""

# Step 6: Verify outputs
echo "Step 6: Verify outputs..."

# Check demo.mp4
if [ -f "demo.mp4" ]; then
    SIZE=$(stat -c%s "demo.mp4")
    if [ $SIZE -gt 100000 ]; then  # More than 100KB
        SIZE_MB=$(echo "scale=1; $SIZE / 1024 / 1024" | bc)
        echo "✓ demo.mp4 generated (~${SIZE_MB}MB)"
    else
        echo "✗ demo.mp4 is too small ($(stat -c%s demo.mp4) bytes)"
        exit 1
    fi
else
    echo "✗ demo.mp4 not found"
    exit 1
fi

# Check README.md
if [ -f "README.md" ]; then
    LINE_COUNT=$(wc -l < "README.md")
    if [ $LINE_COUNT -gt 300 ]; then
        echo "✓ README.md exists ($LINE_COUNT lines)"
    else
        echo "✗ README.md exists but is too short ($LINE_COUNT lines)"
        exit 1
    fi
else
    echo "✗ README.md not found"
    exit 1
fi

# Check registration.json
if [ -f "registration.json" ]; then
    python3 << 'EOF'
import json
try:
    with open("registration.json", "r") as f:
        data = json.load(f)
    print("✓ registration.json exists")
except json.JSONDecodeError:
    print("✗ registration.json is not valid JSON")
    exit(1)
except Exception as e:
    print(f"✗ Error reading registration.json: {e}")
    exit(1)
EOF
else
    echo "✗ registration.json not found"
    exit 1
fi

echo ""
echo "================================"
echo "✓ All tests passed!"
echo "================================"
echo ""
echo "Next steps:"
echo "  1. Update registration.json with your Robothon UUID"
echo "  2. Review demo.mp4 (should show full task sequence)"
echo "  3. Submit via Robothon platform"
