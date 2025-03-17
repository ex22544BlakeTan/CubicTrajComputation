import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ex22544/ros2_ws/src/ar_test/install/ar_test'
