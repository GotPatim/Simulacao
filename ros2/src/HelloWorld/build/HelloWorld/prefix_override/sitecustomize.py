import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nicolas/Downloads/PB_Ros2.ws/src/HelloWorld/install/HelloWorld'
