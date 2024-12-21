import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/surafel/ros2_ws/src/lab_1_package/install/lab_1_package'
