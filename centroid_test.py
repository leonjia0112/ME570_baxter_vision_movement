from point_2d_my import Point
from polygon_my import Polygon

import matplotlib.pyplot as plt

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
poly_centroid = polygon.get_polygon_centroid()
poly_centroid.show_value()
plt.show()

