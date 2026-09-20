"""Keep standalone robot modules importable after tests were grouped here."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ROBOT_DIR = PROJECT_ROOT / "robot"
ROS_PACKAGE_TEST_DIR = (
    PROJECT_ROOT / "ros2_ws" / "src" / "ur3_magnetic_control" / "test"
)

for path in (ROBOT_DIR, ROS_PACKAGE_TEST_DIR):
    value = str(path)
    if value not in sys.path:
        sys.path.insert(0, value)
