import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/DELL/Desktop/ros/ros2/turtle_square/install/turtle_square'
