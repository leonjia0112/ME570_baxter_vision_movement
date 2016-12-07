import math
from line_2d_my import Line

##########################################################
# This module is a 2d point obejct with the coordinate 
# of the x y cooridnate. This has several functions in 
# point operation. Find the distance between two points.
# Sum the value of two points. Check the equality to 
# two points. Scale the number of the coordinates with
# a multiplier value. Obtain a line function from this
# to another point.
##########################################################
class Point():
    # Field:
    #   store the x, y coordiante of the point
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Parameter:
    #   New x cooridnate value
    # Return:
    #   Change the x coordinate to the new x coordiante
    def set_x(self, x):
        self.x = x

    # Parameter:
    #   New x cooridnate value
    # Return:
    #   Change the x coordinate to the new x coordiante
    def set_y(self, y):
        self.y = y

    # Return the value of the x cooridnate
    def get_x(self):
        return self.x

    # Return the value of the y cooridnate
    def get_y(self):
        return self.y

    # Print out the value of the x,y coordinate on the console
    def show_value(self):
        print('x: ' + self.x + '; y: ' + self.y + '\n')

    
    def scale_point(self, multiplier):
        self.x *= multiplier
        self.y *= multiplier
    
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
