#!usr/env/bin python
import numpy as np
import math
import matplotlib.pyplot as plt
import argparse
import sys

import rospy
import baxter_interface


# import messages
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

# import service
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,


# The two modules are local ignore the error
from point_2d_my import Point
from line_2d_my import Line
from polygon_my import Polygon
