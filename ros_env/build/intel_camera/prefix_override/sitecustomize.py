import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/uli/Desktop/tec/personal/intel_sense/ros_env/install/intel_camera'
