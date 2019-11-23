#!/usr/bin/env python3

import cv2
import numpy as np


inputimage = 'realeasy.png'
cv_image = cv2.imread(inputimage)
hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

img_simple = cv2.imread('images/img_simple.png', cv2.IMREAD_GRAYSCALE)
img_realeasy = cv2.imread('images/img_realeasy.png', cv2.IMREAD_GRAYSCALE)
img_realhard = cv2.imread('images/img_realhard.png', cv2.IMREAD_GRAYSCALE)

orb = cv2.ORB()
kp1, des1 = orb.detectAndCompute(gray_image, None)
kp2, des2 = orb.detectAndCompute(img_realeasy, None)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x: x.distance)
print(len(matches))


if inputimage == 'images/simple.png':
    hsv = cv2.blur(hsv, (20, 20))
    lower_filter = np.array([30, 120, 50])
    upper_filter = np.array([50, 180, 200])
elif inputimage == 'images/realeasy.png':
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (40, 40))
    lower_filter = np.array([30, 100, 60])
    upper_filter = np.array([60, 170, 130])
elif inputimage == 'images/realhard.png':
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    hsv = cv2.blur(hsv, (10, 10))
    lower_filter = np.array([30, 0, 20])
    upper_filter = np.array([50, 200, 255])

mask = cv2.inRange(hsv, lower_filter, upper_filter)
res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

# cv2.imshow('hsv cv_image', hsv)
# cv2.imshow('res cv_image', res)
# cv2.waitKey(0)


# Grayscale
gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
# cv2.imshow('Canny Edges After Contouring', gray_res)

# Find Canny edges
# edged = cv2.Canny(gray_res, 150, 150)
ret, edged = cv2.threshold(gray_res, 40, 255, 0)

# Finding Contours
contours, hierarchy = cv2.findContours(
    edged,
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
# cv2.imshow('Canny Edges After Contouring', edged)

# Draw all contours
# -1 signifies drawing all contours
cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 3)

cv2.imshow('Contours', cv_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
