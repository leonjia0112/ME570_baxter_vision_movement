from point_2d_my import Point
import matplotlib.pyplot as plt
p_1 = Point(2, 1)
p_2 = Point(10, 1)
p_3 = Point(10, 10)
p_4 = Point(2, 10)
p_5 = Point(2, 8)
p_6 = Point(6, 6)
p_7 = Point(6, 4)
p_8 = Point(1, 2)

polygon = [p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8]
x = []
y = []

for p in polygon:
    x.append(p.x)
    y.append(p.y)

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

sum_x = sum(x)
sum_y = sum(y)
c_x = sum_x/len(x)
c_y = sum_y/len(y)
polygon_plot(polygon, 'r-')
plt.plot(c_x, c_y, 'b*')
print(c_x, c_y)
plt.show()

