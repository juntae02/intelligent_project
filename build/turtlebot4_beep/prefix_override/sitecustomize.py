import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/juntae02/rokey_ws/install/turtlebot4_beep'
