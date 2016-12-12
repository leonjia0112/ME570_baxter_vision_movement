#!usr/env/bin python
import numpy as np
import math
import matplotlib.pyplot as plt
import argparse
import sys

'''
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
'''

# The two modules are local ignore the error
from point_2d_my import Point
from line_2d_my import Line
from polygon_my import Polygon

ATTRACTION_COEFFICIENT = 5
REPULSIVE_COEFFICIENT = 60
REPULSIVE_RANGE = 50
STEP_SIZE = 0.1
TOTAL_STEP = 10000
TARGET_RANGE = 0.5
INITIAL_DISTANCE_VALUE = 1000
REPULSIVE_POLY_MULTIPLIER = 3
ALPHA = 0.2


def closest_point_to_line(point_test, line_point_1, line_point_2):
    line_temp_list = line_point_1.find_line_point_to_point(line_point_2)
    line_temp = Line(line_temp_list[0], line_temp_list[1], line_temp_list[2])
    # line_temp.show_line_function()
    # line_temp.plot_line()
    orthogonal_line = line_temp.orthogonal_line_cross_point(point_test)
    orthogonal_line.plot_line()
    # orthogonal_line.show_line_function()
    # orthogonal_line.plot_line()
    # orthogonal_line = Line(orthogonal_line_list[0], orthogonal_line_list[1], orthogonal_line_list[2])
    projection_point_list = line_temp.interception_between_two_lines(orthogonal_line)
    projection_point = Point(projection_point_list[0][0], projection_point_list[1][0])
    # projection_point.plot_point()
    # projection_point.show_value()
    # obstacle_point = Point(0, 0)
    if line_temp.check_point_on_line(point_test):
        if (line_point_1.x <= point_test.x <= line_point_2.x or line_point_2.x <= point_test.x <= line_point_2.x) and\
                (line_point_1.y <= point_test.y <= line_point_2.y or line_point_2.y <= point_test.y <= line_point_2.y):
            obstacle_point = Point(point_test.x, point_test.y)
        else:
            distance_point_1 = point_test.find_distance_between_points(line_point_1)
            distance_point_2 = point_test.find_distance_between_points(line_point_2)

            if distance_point_1 < distance_point_2:
                obstacle_point = Point(line_point_1.x, line_point_1.y)
            else:
                obstacle_point = Point(line_point_2.x, line_point_2.y)
    elif (line_point_1.x - ALPHA <= projection_point.x <= line_point_2.x + ALPHA or
            line_point_1.x + ALPHA >= projection_point.x >= line_point_2.x - ALPHA) and\
            (line_point_1.y - ALPHA <= projection_point.y <= line_point_2.y + ALPHA or
             line_point_1.y + ALPHA >= projection_point.y >= line_point_2.y - ALPHA):
        obstacle_point = Point(projection_point.x, projection_point.y)
    else:
        distance_point_1 = point_test.find_distance_between_points(line_point_1)
        distance_point_2 = point_test.find_distance_between_points(line_point_2)

        if distance_point_1 < distance_point_2:
            obstacle_point = Point(line_point_1.x, line_point_1.y)
        else:
            obstacle_point = Point(line_point_2.x, line_point_2.y)

    return obstacle_point


def closest_point_to_obstacle(point_test, polygon):
    ref_point = []
    distance = INITIAL_DISTANCE_VALUE
    for n in range(polygon.get_length() + 1):
        ref_point.append(n % polygon.get_length())

    closest_point_on_obstacle = Point(0, 0)
    for n in range(polygon.get_length()):
        # line_temp = polygon[n].find_line_point_to_point(polygon[n + 1])
        # distance.append(point_test, line_temp)
        print(ref_point[n], ref_point[n+1])
        point_obstacle = closest_point_to_line(point_test, polygon.poly[ref_point[n]], polygon.poly[ref_point[n + 1]])
        if point_test.find_distance_between_points(point_obstacle) < distance:
            distance = point_test.find_distance_between_points(point_obstacle)
            closest_point_on_obstacle.x = point_obstacle.x
            closest_point_on_obstacle.y = point_obstacle.y
    return closest_point_on_obstacle


def attraction_force(point_to_test, point_goal):
    vector_magnitude = point_to_test.find_distance_between_points(point_goal)
    attraction_vector = Point(point_goal.x - point_to_test.x, point_goal.y - point_to_test.y)
    attraction_vector.scale_point(ATTRACTION_COEFFICIENT / vector_magnitude)
    return attraction_vector


def repulsive_force(point_to_test, point_obstacle, polygon_centroid):
    distance_to_obstacle = point_to_test.find_distance_between_points(point_obstacle)
    distance_to_center = point_to_test.find_distance_between_points(polygon_centroid)
    repulsive_vector = Point(0, 0)
    abs_distance = distance_to_center - distance_to_obstacle
    if distance_to_center <= REPULSIVE_RANGE:
        repulsive_magnitude = (0.5*REPULSIVE_COEFFICIENT)*(1/(distance_to_center-3)**2)
        # *(1/abs_distance**2)
        vector_magnitude = point_to_test.find_distance_between_points(polygon_centroid)
        repulsive_vector.set_x((point_to_test.x - polygon_centroid.x) * repulsive_magnitude / vector_magnitude)
        repulsive_vector.set_y((point_to_test.y - polygon_centroid.y) * repulsive_magnitude / vector_magnitude)

    return repulsive_vector


def total_potential_force(att_vector, rep_vector):
    total_vector = Point(att_vector.x + rep_vector.x, att_vector.y + att_vector.y)
    return total_vector


def potential_get_path(polygon, point_start, point_goal, polygon_centroid):
    path = [Point(point_start.x, point_start.y)]
    point_current = Point(point_start.x, point_start.y)
    step = 0
    while point_current.find_distance_between_points(point_goal) >= TARGET_RANGE and step <= TOTAL_STEP:
        att_vector = attraction_force(point_current, point_goal)
        point_obstacle = closest_point_to_obstacle(point_current, polygon)
        # point_obstacle.plot_point()
        print('current \npoint obstacle\nattraction\nrepulsive')
        point_current.show_value()
        point_obstacle.show_value()
        # print(point_current.find_distance_between_points(point_obstacle))

        if point_current.find_distance_between_points(point_obstacle) <= REPULSIVE_RANGE:
            rep_vector = repulsive_force(point_current, point_obstacle, polygon_centroid)
        else:
            rep_vector = Point(0, 0)

        att_vector.show_value()
        rep_vector.show_value()

        total_vector = total_potential_force(att_vector, rep_vector)
        # force_point = po
        # point_current.plot_line_between_two_point()
        # point_current.show_value()
        point_current.add_point(total_vector, STEP_SIZE)
        # point_current.show_value()
        path.append(Point(point_current.x, point_current.y))
        step += 1

    return path


# Parameter:
#   path contains a sequence of x y coordinate
# Return:
#   plot the sequence of dot on a figure
def plot_path(path):
    x = []
    y = []
    for p in path:
        # print(p.x, p.y)
        x.append(p.x)
        y.append(p.y)

    plt.plot(x, y, 'k-')

'''
def ik_test(limb):
    # initialized a node with name
    rospy.init_node("rsdk_ik_service_client")

    # string variable
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"

    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)

    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.657579481614,
                    y=0.851981417433,
                    z=0.0388352386502,
                ),
                orientation=Quaternion(
                    x=-0.366894936773,
                    y=0.885980397775,
                    z=0.108155782462,
                    w=0.262162481772,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.656982770038,
                    y=-0.852598021641,
                    z=0.0388609422173,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found:")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print(limb_joints)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return limb_joints
'''

def main():

    '''
    # Baxter Setup
    """RSDK Joint Position Waypoints Example

        Records joint positions each time the navigator 'OK/wheel'
        button is pressed.
        Upon pressing the navigator 'Rethink' button, the recorded joint positions
        will begin playing back in a loop.
        """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['left', 'right'],
        help='limb to record/playback waypoints'
    )
    parser.add_argument(
        '-s', '--speed', default=0.3, type=float,
        help='joint position motion speed ratio [0.0-1.0] (default:= 0.3)'
    )
    parser.add_argument(
        '-a', '--accuracy',
        default=baxter_interface.settings.JOINT_ANGLE_TOLERANCE, type=float,
        help='joint position accuracy (rad) at which waypoints must achieve'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")

    # initialized node
    rospy.init_node("rsdk_joint_position_waypoints_%s" % (args.limb,))
    '''

    p_1 = Point(2, 1)
    p_2 = Point(10, 1)
    p_3 = Point(10, 10)
    p_4 = Point(2, 10)
    p_5 = Point(2, 8)
    p_6 = Point(6, 6)
    p_7 = Point(6, 4)
    p_8 = Point(1, 2)

    polygon = Polygon()
    polygon.add_point(p_1)
    polygon.add_point(p_2)
    polygon.add_point(p_3)
    polygon.add_point(p_4)
    polygon.add_point(p_5)
    polygon.add_point(p_6)
    polygon.add_point(p_7)
    polygon.add_point(p_8)

    polygon.polygon_plot('r-')
    convex_polygon = polygon.concave_convex_conversion()
    convex_polygon.polygon_plot('g-')
    safe_range_polygon = convex_polygon.repulsive_poly(REPULSIVE_POLY_MULTIPLIER)
    safe_range_polygon.polygon_plot('b-')
    point_centroid = safe_range_polygon.get_polygon_centroid()
    # p = Point(-5, -5)
    point_centroid.plot_point()
    point_start = Point(-5, 5)
    p_obs = closest_point_to_obstacle(point_start, safe_range_polygon)
    point_start.plot_point()
    p_obs.plot_point()
    point_goal = Point(15, 0)
    point_goal.plot_point()
    # path = potential_get_path(safe_range_polygon, point_start, point_goal, point_centroid)
    # plot_path(path)
    plt.show()


main()
