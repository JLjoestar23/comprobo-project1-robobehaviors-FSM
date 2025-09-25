import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ros/ros2_ws/src/comprobo-project1-robobehaviors-FSM/install/robobehaviors_fsm'
