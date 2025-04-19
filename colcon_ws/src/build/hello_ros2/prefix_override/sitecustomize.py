import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lee/kuLimo/colcon_ws/src/install/hello_ros2'
