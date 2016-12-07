import math
from line_2d_my import Line


class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def point_equal_check(self, point_to_go):
        if self.x == point_to_go.x and self.y == point_to_go.y:
            return True
        else:
            return False

    def point_equal_check_in_range(self, point_to_go, target_range):
        if self.x - point_to_go.x <= target_range and \
                                self.y - point_to_go.y <= target_range:
            return True
        else:
            return False

    def set_x(self, x):
        self.x = x

    def set_y(self, y):
        self.y = y

    def scale_point(self, multiplier):
        self.x *= multiplier
        self.y *= multiplier

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def add_point(self, point, magnitude):
        self.x += point.x * magnitude
        self.y += point.y * magnitude

    def sum_two_points(self, point_to_go):
        point_to_go.x += self.x
        point_to_go.y += self.y
        return point_to_go

    def find_distance_between_points(self, point_to_go):
        return math.sqrt((self.x - point_to_go.x)**2 + (self.y - point_to_go.y)**2)

    def find_line_point_to_point(self, point_to_go):
        m = (point_to_go.y - self.y) / (point_to_go.x - self.x)
        h = - m * self.x + self.y
        a = 1
        b = - m
        c = - h
        return Line(a, b, c)
