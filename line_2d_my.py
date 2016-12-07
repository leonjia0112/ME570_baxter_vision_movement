import numpy as np
from point_2d_my import Point

# This class is operating a line function in 2 dimension with the line function as ay + bx + c = 0
# This class manipulate several functionality. Check the point whether is on this line or not. Find
# a orthogonal line that cross a point in space. check the interception between two liens.
class Line():
    def __init__(self, a, b, c):
        # line function type: ay + bx + c = 0
        self.a = a
        self.b = b
        self.c = c

    # Parameter:
    #   A point object
    # Return:
    #   True if the point is on this line
    #   False if the point is not on this line
    def check_point_on_line(self, point_to_go):
        y = - self.b/self.a*point_to_go.x - self.c/self.a
        return y == point_to_go.y

    # Parameter:
    #   A point object
    # Return:
    #   A line object that is orthogonal to this line and cross the point
    def orthogonal_line_cross_point(self, point_to_go):
        slope = - self.b / self.a
        # intercept = - self.c / self.a
        orthogonal_slope =  - 1 / slope
        orthogonal_intercept = point_to_go.y + 1 / slope * point_to_go.x
        orthogonal_line = Line(1, - orthogonal_slope, - orthogonal_intercept)
        return orthogonal_line

    # Parameter:
    #   Another line object
    # Return:
    #   A point object that store the value of the interception between the other line and this line
    def interception_between_two_lines(self, other_line):
        a1 = other_line.a
        b1 = other_line.b
        c1 = other_line.c
        left = np.array([[self.b, self.a], [b1, a1]])
        right = np.array([[self.c], [c1]])
        solution = np.linalg.solve(left, right)
        interception_point = Point(solution[1], solution[2])
        return interception_point
