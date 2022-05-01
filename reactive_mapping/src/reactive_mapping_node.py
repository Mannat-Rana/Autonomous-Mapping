#!/usr/bin/env python
import sys
import rospy
sys.path.insert(1, '/home/sikhdragon/catkin_ws/src/reactive_mapping/include')
from reactive_mapping.reactive_mapping import ReactiveMapping

if __name__ == '__main__':
    node = ReactiveMapping()
    rospy.spin()