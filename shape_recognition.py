import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

from point_2d_my import Point

class ShapeRecognition:

    def __init__(self, img):
        self.img = img

    def process(self):
        my_lower = np.array([0, 0, 0], dtype=np.uint8)
        my_upper = np.array([100, 100, 100], dtype=np.uint8)
        mask = cv2.inRange(self.img, my_lower, my_upper)
        # cv2.imshow('mask', mask)

        (flags, contours, h) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # print(contours)
         return contours

img = cv2.imread('polygon_1.png')



shape_rec = ShapeRecognition(img)
contours = shape_rec.process()
all_points = []
print(contours[0].size)
print(len(contours[0]))
for i in range(len(contours[0])):
    all_points.append(Point(contours[0][i][0][0], -contours[0][i][0][1]))

for p in all_points:
    p.plot_point()

plt.show()
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

