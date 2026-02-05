import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/technomant/ros2_ws/install/fake_encoder_pkg'
