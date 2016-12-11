import numpy as np
import math
import matplotlib.pyplot as plt

# The two modules are local ignore the error
from point_2d_my import Point
from line_2d_my import Line


ATTRACTION_COEFFICIENT = 1
REPULSIVE_RANGE = 2
REPULSIVE_COEFFICIENT = 50
STEP_SIZE = 0.1
TOTAL_STEP = 1000
TARGET_RANGE = 1
INITIAL_DISTANCE_VALUE = 1000
REPULSIVE_POLY_MULTIPLIER = 3


# Parameter:
#   two points
# Return:
#   the angle of the line segment of these points
def find_angle(point_1, point_2):
    delta_y = point_2.y - point_1.y
    delta_x = point_2.x - point_1.x
    length = math.sqrt(delta_x*delta_x + delta_y*delta_y)
    angle = math.asin(delta_y/length)

    if delta_x < 0 < delta_y:
        angle = math.pi - angle
    elif delta_x < 0 and delta_y < 0:
        angle = -math.pi - angle
    elif delta_x < 0 and delta_y == 0:
        angle = math.pi
    # print(angle)
    return angle


# Parameter:
#   angle to test inclusion, included angle by previous angle and next angle
# Return:
#   True if the angle is within the two angles
#   False if the angle is outside the two angle
def test_one_angle_inclusion(angle_pre, angle_next, angle_test):
    is_included = False
    if angle_pre > angle_next:
        if angle_next <= angle_test <= angle_pre:
            is_included = True
            is_included = True
    elif angle_next > angle_pre:
        if -math.pi <= angle_test <= angle_pre or angle_next <= angle_test <= math.pi:
            is_included = True
    return is_included


# parameter:
#   a polygon and a vertex_index
# return;
#   if the angle between the two connected edge contain all the other vertex within the range of angle
#   return True
#   else return False
def inclusion_finder(polygon, vertex_index):
    if vertex_index-1 < 0:
        vertex_order_minus = len(polygon) - 1
    else:
        vertex_order_minus = vertex_index - 1

    if vertex_index + 1 > len(polygon) - 1:
        vertex_order_plus = 0
    else:
        vertex_order_plus = vertex_index + 1

    point_test = polygon[vertex_index]
    point_test_minus = polygon[vertex_order_minus]
    point_test_plus = polygon[vertex_order_plus]

    angle_previous = find_angle(point_test, point_test_minus)
    angle_next = find_angle(point_test, point_test_plus)

    all_points_included = True

    for n in range(len(polygon)):
        if n != vertex_index and n != vertex_order_minus and n != vertex_order_plus:
            angle_temp = find_angle(point_test, polygon[n])
            if not test_one_angle_inclusion(angle_previous, angle_next, angle_temp):
                all_points_included = False
                break

    return all_points_included


def polygon_concave_convex_conversion(polygon):
    mark = []
    mark_double = []
    start = 0
    concave_start = 0
    delete = []
    for i in range(len(polygon)):
        mark.append(inclusion_finder(polygon, i))
    for i in range(len(mark) * 2):
        mark_double.append(mark[i % len(mark)])
    for i in range(1, len(mark_double)):
        initialized = False
        if mark_double[i] and mark_double[i - 1]:
            initialized = True
        if not mark_double[i] and initialized:
            start = i
            break
    case = False
    for i in range(start, len(mark_double)):
        if not mark_double[i] and not case:
            case = True
            concave_start = i

        if case and mark_double[i] and mark_double[i - 1]:
            for j in range(concave_start + 1, i - 2):
                delete.append(j % len(mark))
            case = False
    new_delete = list(set(delete))
    new_polygon = []

    # debug print
    # for a in new_delete:
    # print(new_delete)
    # print(list(set(delete)))

    for i in range(len(polygon)):
        temp = i
        if temp not in new_delete:
            new_polygon.append(polygon[i])
    return new_polygon


# Method added to polygon object
def polygon_plot(my_polygon, plot_type):
    x = []
    y = []
    for i in range(len(my_polygon)):
        x.append(my_polygon[i].x)
        y.append(my_polygon[i].y)
    x.append(my_polygon[0].x)
    y.append(my_polygon[0].y)

    plt.plot(x, y, plot_type)
    plt.axis([-10, 20, -10, 20])


# parameter
#   a list with integers
# return
#   a new list that is sorted and remove all the element that occurs more than once in the list
def delete_duplicate_number(my_list):
    list.sort(my_list)
    delete_list = my_list[0]
    for i in range(1, len(my_list)):
        if my_list[i] != my_list[i - 1]:
            delete_list.append(my_list[i])
    return delete_list


def repulsive_poly_point(point_center, point_a, point_b):
    angle_center_a = find_angle(point_center, point_a)
    angle_center_b = find_angle(point_center, point_b)

    # debug print
    # print(angle_center_a)
    # print(angle_center_b)

    angle_new = (angle_center_a + angle_center_b)/2
    if angle_center_a > angle_center_b:
        if angle_new == math.pi:
            angle_new = 0
        elif angle_new == 0:
            angle_new = math.pi
        elif angle_new > 0:
            angle_new -= math.pi
        elif angle_new < 0:
            angle_new += math.pi

    x = math.cos(angle_new) * REPULSIVE_POLY_MULTIPLIER + point_center.x
    y = math.sin(angle_new) * REPULSIVE_POLY_MULTIPLIER + point_center.y
    new_point = Point(x, y)

    # print(x, '; ', y)
    return new_point


def repulsive_poly(polygon):
    new_polygon = list()
    ref_point = list()
    for i in range(0, len(polygon)):
        ref_point.append(i)
    ref_point.append(0)
    ref_point.append(1)

    # for i in ref_point:
    #    print(i)
    # for n in ref_point:
    #    print(n)
    # for n in range(1, len(polygon) + 1):
    #    print(n)

    for n in range(1, len(polygon) + 1):

        # debug print
        # print(n)
        # print(ref_point[n], ref_point[n-1], ref_point[n+1], '\n')

        point_temp = polygon[ref_point[n]]
        point_previous = polygon[ref_point[n - 1]]
        point_next = polygon[ref_point[n + 1]]
        new_polygon.append(repulsive_poly_point(point_temp, point_previous, point_next))
    return new_polygon


def closest_point_to_line(point_test, line_point_1, line_point_2):
    line_temp_list = line_point_1.find_line_point_to_point(line_point_2)
    line_temp = Line(line_temp_list[0], line_temp_list[1], line_temp_list[2])
    # line_temp.show_line_function()
    # line_temp.plot_line()
    orthogonal_line = line_temp.orthogonal_line_cross_point(point_test)
    # orthogonal_line.show_line_function()
    # orthogonal_line.plot_line()
    # orthogonal_line = Line(orthogonal_line_list[0], orthogonal_line_list[1], orthogonal_line_list[2])
    projection_point_list = line_temp.interception_between_two_lines(orthogonal_line)
    projection_point = Point(projection_point_list[0][0], projection_point_list[1][0])
    # projection_point.show_value()
    # obstacle_point = Point(0, 0)
    if line_temp.check_point_on_line(point_test):
        if (line_point_1.x <= point_test.x <= line_point_2.x or line_point_2.x <= point_test.x <= line_point_2.x) and\
                (line_point_1.y <= point_test.y <= line_point_2.y or line_point_2.y <= point_test.y <= line_point_2.y):
            obstacle_point = point_test
        else:
            distance_point_1 = point_test.find_distance_between_points(line_point_1)
            distance_point_2 = point_test.find_distance_between_points(line_point_2)

            if distance_point_1 < distance_point_2:
                obstacle_point = line_point_1
            else:
                obstacle_point = line_point_2
    elif (line_point_1.x <= projection_point.x <= line_point_2.x or
            line_point_1.x >= projection_point.x >= line_point_2.x) and\
            (line_point_1.y <= projection_point.y <= line_point_2.y or
            line_point_1.y >= projection_point.y >= line_point_2.y):
        obstacle_point = projection_point
    else:
        distance_point_1 = point_test.find_distance_between_points(line_point_1)
        distance_point_2 = point_test.find_distance_between_points(line_point_2)

        if distance_point_1 < distance_point_2:
            obstacle_point = line_point_1
        else:
            obstacle_point = line_point_2
    return obstacle_point


def closest_point_to_obstacle(point_test, polygon):
    ref_point = []
    distance = INITIAL_DISTANCE_VALUE
    for n in range(len(polygon) + 1):
        ref_point.append(n % len(polygon))

    # distance = list
    closest_point_on_obstacle = Point(0, 0)
    for n in range(len(polygon)):
        # line_temp = polygon[n].find_line_point_to_point(polygon[n + 1])
        # distance.append(point_test, line_temp)
        point_obstacle = closest_point_to_line(point_test, polygon[ref_point[n]], polygon[ref_point[n + 1]])
        if point_test.find_distance_between_points(point_obstacle) < distance:
            distance = point_test.find_distance_between_points(point_obstacle)
            closest_point_on_obstacle.x = point_obstacle.x
            closest_point_on_obstacle.y = point_obstacle.y
    return closest_point_on_obstacle


def attraction_force(point_to_test, point_goal):
    attraction_vector = Point(point_goal.x - point_to_test.x, point_goal.y - point_to_test.y)
    attraction_vector.scale_point(ATTRACTION_COEFFICIENT)
    return attraction_vector


def repulsive_force(point_to_test, point_obstacle, polygon_obs):
    distance = point_to_test.find_distance_between_points(point_obstacle)
    repulsive_vector = Point(0, 0)
    if distance <= REPULSIVE_RANGE:
        repulsive_magnitude = (0.5*REPULSIVE_COEFFICIENT)*((1/REPULSIVE_RANGE) + (1/distance))*(1/distance**2)
        repulsive_vector.set_x((point_to_test.x - point_obstacle.x) * repulsive_magnitude)
        repulsive_vector.set_y((point_to_test.y - point_obstacle.y) * repulsive_magnitude)

    return repulsive_vector


def total_potential_force(att_vector, rep_vector):
    total_vector = Point(att_vector.x + rep_vector.x, att_vector.y + att_vector.y)
    return total_vector


def potential_get_path(polygon, point_start, point_goal):
    path = [Point(point_start.x, point_start.y)]
    point_current = point_start
    step = 0
    while point_current.find_distance_between_points(point_goal) >= TARGET_RANGE and step <= TOTAL_STEP:
        att_vector = attraction_force(point_current, point_goal)
        point_obstacle = closest_point_to_obstacle(point_current, polygon)
        print('current \npoint obstacle\nattraction\nrepulsive')
        point_current.show_value()
        point_obstacle.show_value()
        # print(point_current.find_distance_between_points(point_obstacle))
        if point_current.find_distance_between_points(point_obstacle) <= REPULSIVE_RANGE:
            rep_vector = repulsive_force(point_current, point_obstacle, polygon)
        else:
            rep_vector = Point(0, 0)
        att_vector.show_value()
        rep_vector.show_value()
        total_vector = total_potential_force(att_vector, rep_vector)
        # force_ponit = po
        # point_current.plot_line_between_two_point()
        # point_current.show_value()
        point_current.add_point(total_vector, STEP_SIZE)
        #point_current.show_value()
        path.append(Point(point_current.x, point_current.y))
        step += 1

    return path


def plot_path(path):
    x = []
    y = []
    for p in path:
        # print(p.x, p.y)
        x.append(p.x)
        y.append(p.y)

    plt.plot(x, y, 'r-')


# Method added to polygon object
def get_polygon_centroid(polygon):
    x = []
    y = []
    for p in polygon:
        x.append(p.x)
        y.append(p.y)
    sum_x = sum(x)
    sum_y = sum(y)
    c_x = sum_x / len(x)
    c_y = sum_y / len(y)
    return Point(c_x, c_y)


def main():
    p_1 = Point(2, 1)
    p_2 = Point(10, 1)
    p_3 = Point(10, 10)
    p_4 = Point(2, 10)
    p_5 = Point(2, 8)
    p_6 = Point(6, 6)
    p_7 = Point(6, 4)
    p_8 = Point(1, 2)

    polygon = [p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8]
    # polygon_plot(polygon, 'b-')
    # plt.show()
    convex_polygon = polygon_concave_convex_conversion(polygon)
    # polygon_plot(convex_polygon, 'r-')
    safe_range_polygon = repulsive_poly(convex_polygon)
    point_centroid = get_polygon_centroid(safe_range_polygon)
    # plt.show()
    p = Point(-5, -5)
    #p_obs = closest_point_to_obstacle(p, safe_range_polygon)

    #p_obs.show_value()
    #p.plot_line_between_two_points(p_obs)
    plt.axis([-20, 20, -20, 20])
    # projection_point = closest_point_to_line(Point(0, 3), Point(0, -1), Point(3, 2))
    # projection_point.show_value()
    point_start = Point(-5, -5)
    point_goal = Point(18, 0)
    path = potential_get_path(safe_range_polygon, point_start, point_goal)
    polygon_plot(safe_range_polygon, 'g-')
    plot_path(path)
    #for p in path:
    #    print(p.x, p.y)
    plt.show()


main()

'''
l_1 = Line(0, 1, -2)
p = Point(-3, -3)
l = l_1.orthogonal_line_cross_point(p)
l_1.show_line_function()
l.show_line_function()
'''
