import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/enzo/droid/WORK-2026/work_ws/src/install/robot_odometry'
