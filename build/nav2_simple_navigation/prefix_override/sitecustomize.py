import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/viet/hospital_robot_nav/install/nav2_simple_navigation'
