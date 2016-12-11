import numpy as np
# from point_2d_my import Point
import matplotlib.pyplot as plt


# This class is operating a line function in 2 dimension with the line function as ay + bx + c = 0
# This class manipulate several functionality. Check the point whether is on this line or not. Find
# a orthogonal line that cross a point in space. check the interception between two liens.
class Line():
    def __init__(self, a, b, c):
        # line function type: ay + bx + c = 0
        self.a = float(a)
        self.b = float(b)
        self.c = float(c)

    # Parameter:
    #   A point object
    # Return:
    #   True if the point is on this line
    #   False if the point is not on this line
    def check_point_on_line(self, point_to_go):
        if self.a == 0:
            if -self.c == point_to_go.x:
                return True
            else:
                return False
        elif self.b == 0:
            if -self.c == point_to_go.y:
                return True
            else:
                return False
        else:
            y = - self.b/self.a*point_to_go.x - self.c/self.a
            if y == point_to_go.y:
                return True
            else:
                return False

    # Parameter:
    #   A point object
    # Return:
    #   A line object that is orthogonal to this line and cross the point
    def orthogonal_line_cross_point(self, point_to_go):

        # intercept = - self.c / self.a
        if self.a == 0:
            orthogonal_a = 1
            orthogonal_b = 0
            orthogonal_c = - point_to_go.y
        elif self.b == 0:
            orthogonal_a = 0
            orthogonal_b = 1
            orthogonal_c = - point_to_go.x
        else:
            # slope = - self.b / self.a
            # orthogonal_slope = - 1 / slope
            orthogonal_a = 1
            orthogonal_b = self.a / self.b
            orthogonal_c = - point_to_go.y + (self.a / self.b) * point_to_go.x
        orthogonal_line = Line(orthogonal_a, orthogonal_b, orthogonal_c)
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
        right = np.array([[- self.c], [- c1]])
        solution = np.linalg.solve(left, right)
        interception_point = [solution[0], solution[1]]
        return interception_point

    def show_line_function(self):
        line_function = repr(self.a) + '*y + ' + repr(self.b) + '*x + ' + repr(self.c) + '= 0'
        print(line_function)

    def plot_line(self):

        x = []
        y = []

        if self.a == 0:
            x = [-self.c, -self.c]
            y = [-10, 10]
        elif self.b == 0:
            x = [-10, 10]
            y = [-self.c, -self.c]
        else:
            for n in list(range(-10, 10)):
                x.append(n)
                y.append(-self.b / self.a * n - self.c / self.a)
        plt.plot(x, y, 'r-')
