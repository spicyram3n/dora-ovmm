"""Next best view planning for the HSR-C.

The planner itself lives here as importable Python. The C++ side under src/ is
only the thin layer that talks to nav2 and the controllers, where the action
clients are less awkward than in rclpy.
"""
