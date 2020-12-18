#!/usr/bin/env python
#
import sys
import numpy
import heapq
import rospy
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import *
from collections import deque

NAME = "MARTINEZ_FADUL"


print "PRUEBA - " + NAME
rospy.init_node("prEBA")

