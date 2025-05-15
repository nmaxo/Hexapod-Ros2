import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/max/hexapod_ws/install/joint_state_publisher_gui'
