import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zyx/ros2_learn/node/chapt3/topic_practice_ws/install/py_status_publisher'
