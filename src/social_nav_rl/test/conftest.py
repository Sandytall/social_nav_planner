"""Make the sibling social_nav_tools package importable for the env's mock backend in bare
pytest (no colcon/ROS sourcing)."""
import os
import sys

_HERE = os.path.dirname(__file__)
_TOOLS = os.path.abspath(os.path.join(_HERE, "..", "..", "social_nav_tools"))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
