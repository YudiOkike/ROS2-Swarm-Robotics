import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/parallels/ros2_ws/src/swarm_robots/install/swarm_robots'
