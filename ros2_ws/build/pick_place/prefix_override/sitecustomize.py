import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/workspaces/pick-and-place-conveyor/ros2_ws/install/pick_place'
