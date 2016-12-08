import math
import matplotlib.pyplot as plt
# from line_2d_my import Line


##########################################################
# This module is a 2d point obejct with the coordinate 
# of the x y cooridnate. This has several functions in 
# point operation. Find the distance between two points.
# Sum the value of two points. Check the equality to 
# two points. Scale the number of the coordinates with
# a multiplier value. Obtain a line function from this
# to another point.
##########################################################
class Point:
    # Field:
    #   store the x, y coordiante of the point
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    # Parameter:
    #   New x cooridnate value
    # Return:
    #   Change the x coordinate to the new x cooridnate
    def set_x(self, x):
        self.x = x

    # Parameter:
    #   New x cooridnate value
    # Return:
    #   Change the x coordinate to the new x cooridnate
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
        print('x: ' + repr(self.x) + '; y: ' + repr(self.y) + '\n')

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
        if self.x == point_to_go.x:
            a = 1
            b = 0
            c = self.x
        elif self.y ==  point_to_go.y:
            a = 0
            b = 1
            c = self.y
        else:
            m = (point_to_go.y - self.y) / (point_to_go.x - self.x)
            h = - m * self.x + self.y
            a = 1
            b = - m
            c = - h
        line_cross_two_points = [a, b, c]
        return line_cross_two_points

    def find_angle(self, point_2):
        delta_y = point_2.y - self.y
        delta_x = point_2.x - self.x
        length = math.sqrt(delta_x*delta_x + delta_y*delta_y)
        angle = math.asin(delta_y/length)

        if delta_x < 0 < delta_y:
            angle = math.pi - angle
        elif delta_x < 0 and delta_y < 0:
            angle = - math.pi - angle
        elif delta_x < 0 and delta_y == 0:
            angle = math.pi
        print(angle)
        return angle

    def plot_line_between_two_points(self, point):
        x = [self.x, point.x]
        y = [self.y, point.y]
        plt.plot(x, y, '-r')
