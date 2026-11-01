import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sebastian/jar_ws/src/jar_robot/install/jar_exploracion'
