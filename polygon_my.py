import numpy as np
import matplotlib.pyplot as plt
import math

from point_2d_my import Point

IMAGE_BOUNDARY = 20


class Polygon:
    def __init__(self):
        self.poly = []
        self.length = 0

    # Parameter
    #   A point object
    # Return
    #   Add a vertex to the polygon, vertex information is a point object
    def add_point(self, point):
        self.poly.append(point)
        self.length += 1

    # Return:
    #   The number of the vertices of the polygon
    def get_length(self):
        return self.length

    # Return:
    #   Get the centroid of the polygon and return as a point object
    def get_polygon_centroid(self):
        x = []
        y = []
        for p in self.poly:
            x.append(p.x)
            y.append(p.y)
        sum_x = sum(x)
        sum_y = sum(y)
        c_x = sum_x / len(x)
        c_y = sum_y / len(y)
        return Point(c_x, c_y)

    def get_polygon_vertices_list(self):
        return self.poly

    # Return:
    #   Plot the polygon
    def polygon_plot(self, plot_type):
        x = []
        y = []
        for i in range(len(self.poly)):
            x.append(self.poly[i].x)
            y.append(self.poly[i].y)
        x.append(self.poly[0].x)
        y.append(self.poly[0].y)

        plt.plot(x, y, plot_type)
        plt.axis([-IMAGE_BOUNDARY, IMAGE_BOUNDARY, -IMAGE_BOUNDARY, IMAGE_BOUNDARY])

    # Return:
    #   Change the current polygon to a polygon without concave part
    def concave_convex_conversion(self):
        mark = []
        mark_double = []
        start = 0
        concave_start = 0
        delete = []
        concave_flag = False

        for i in range(self.length):
            mark.append(self.inclusion_finder(i))
        for i in range(len(mark) * 2):
            mark_double.append(mark[i % len(mark)])
        for i in range(1, len(mark_double)):
            initialized = False

            # start with the first True after False
            if mark_double[i] and mark_double[i - 1]:
                initialized = True

            # find the False after initialized to start the process
            if not mark_double[i] and initialized:
                start = i
                break

        for i in range(start, len(mark_double)):
            if not mark_double[i] and not concave_flag:
                concave_flag = True
                concave_start = i

            if concave_flag and mark_double[i] and mark_double[i - 1]:
                for j in range(concave_start + 1, i - 2):
                    delete.append(j % len(mark))
                concave_flag = False
        # eliminate duplicate vertex indices
        new_delete = list(set(delete))
        new_polygon = Polygon()

        for i in range(len(self.poly)):
            # temp = i
            if i not in new_delete:
                new_polygon.add_point(self.poly[i])
        return new_polygon

    # parameter:
    #   a polygon and a vertex_index
    # return;
    #   if the angle between the two connected edge contain all the other vertex within the range of angle
    #   return True
    #   else return False
    def inclusion_finder(self, vertex_index):
        if vertex_index - 1 < 0:
            vertex_order_minus = len(self.poly) - 1
        else:
            vertex_order_minus = vertex_index - 1

        if vertex_index + 1 > len(self.poly) - 1:
            vertex_order_plus = 0
        else:
            vertex_order_plus = vertex_index + 1

        point_test = self.poly[vertex_index]
        point_test_minus = self.poly[vertex_order_minus]
        point_test_plus = self.poly[vertex_order_plus]

        angle_previous = point_test.find_angle(point_test_minus)
        angle_next = point_test.find_angle(point_test_plus)

        all_points_included = True

        for n in range(len(self.poly)):
            if n != vertex_index and n != vertex_order_minus and n != vertex_order_plus:
                angle_temp = point_test.find_angle(self.poly[n])
                if not self.test_one_angle_inclusion(angle_previous, angle_next, angle_temp):
                    all_points_included = False
                    break
        return all_points_included

    # Parameter:
    #   angle to test inclusion, included angle by previous angle and next angle
    # Return:
    #   True if the angle is within the two angles
    #   False if the angle is outside the two angle
    @staticmethod
    def test_one_angle_inclusion(angle_pre, angle_next, angle_test):
        is_included = False
        if angle_pre > angle_next:
            if angle_next <= angle_test <= angle_pre:
                is_included = True
        elif angle_next > angle_pre:
            if -math.pi <= angle_test <= angle_pre or angle_next <= angle_test <= math.pi:
                is_included = True
        return is_included

    # Parameter:
    #   The safty range number to create a saft polygon
    # Pre:
    #   This polygon must be all convex, concave edge can mess up the shape of the polygon
    # Return:
    #   Another polygon that has the same shape but with larger size then the original poly
    def repulsive_poly(self, multiplier):
        new_polygon = Polygon()
        ref_point = list()
        for i in range(self.length):
            ref_point.append(i)
        ref_point.append(0)
        ref_point.append(1)
        # from the second vertex of the poly to go around until reach the first vertex
        for n in range(1, self.length + 1):
            point_temp = self.poly[ref_point[n]]
            point_previous = self.poly[ref_point[n - 1]]
            point_next = self.poly[ref_point[n + 1]]
            new_polygon.add_point(point_temp.repulsive_poly_point(point_previous, point_next, multiplier))
        return new_polygon
