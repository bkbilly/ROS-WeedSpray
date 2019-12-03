#!/usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy


def showPlot(img):
    for i, col in enumerate(['b', 'g', 'r']):
        hist = cv2.calcHist([img], [i], None, [256], [0, 256])
        plt.plot(hist, color=col)
        plt.xlim([0, 256])
    plt.show()


def filter_colors(cv_image, runtype):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    if runtype == 'simple':
        # hsv = cv2.blur(hsv, (10, 10))
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_filter = np.array([30, 120, 0])
        upper_filter = np.array([50, 180, 200])
    if runtype == 'simple_inv':
        # hsv = cv2.blur(hsv, (10, 10))
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_filter = np.array([0, 0, 0])
        upper_filter = np.array([255, 80, 255])
    elif runtype == 'realeasy':
        hsv = cv2.blur(hsv, (40, 40))
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_filter = np.array([0, 10, 40])
        upper_filter = np.array([60, 150, 255])
    elif runtype == 'realeasy_inv':
        # hsv = cv2.blur(hsv, (40, 40))
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_filter = np.array([30, 30, 0])
        upper_filter = np.array([100, 90, 40])
    elif runtype == 'realhard':
        hsv = cv2.blur(hsv, (40, 40))
        # hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
        lower_filter = np.array([40, 40, 0])
        upper_filter = np.array([100, 80, 200])
    elif runtype == 'realhard_inv':
        hsv = cv2.blur(hsv, (40, 40))
        hsv = cv2.GaussianBlur(hsv, ksize=(17,17), sigmaX=10)
        lower_filter = np.array([0, 90, 0])
        upper_filter = np.array([255, 100, 255])
    elif runtype == 'ground':
        # hsv = cv2.blur(hsv, (40, 40))
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_filter = np.array([0, 30, 30])
        upper_filter = np.array([20, 140, 80])
    elif runtype == 'ground_inv':
        # hsv = cv2.blur(hsv, (40, 40))
        hsv = cv2.GaussianBlur(hsv, ksize=(17, 17), sigmaX=10)
        lower_filter = np.array([30, 0, 10])
        upper_filter = np.array([90, 255, 150])

    mask = cv2.inRange(hsv, lower_filter, upper_filter)
    res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    # cv2.imshow('hsv', hsv)
    return res, mask


def get_contours(res, cv_image):
    # Grayscale
    gray_res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Canny Edges After Contouring', gray_res)

    # Find Canny edges
    # edged = cv2.Canny(gray_res, 150, 150)
    ret, edged = cv2.threshold(gray_res, 20, 255, 0)

    # Finding Contours
    contours, hierarchy = cv2.findContours(
        edged,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # contours = max(contours, key=cv2.contourArea)

    threshold_area = 300
    filtered_contours = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > threshold_area:
            filtered_contours.append(cnt)
    # cv2.imshow('Canny Edges After Contouring', edged)
    contours = copy.copy(cv_image)
    cv2.drawContours(contours, filtered_contours, -1, (0, 255, 0), 1)
    return contours, filtered_contours


inputimage = ['images/plants3/ros_plant3_0.jpg', 'realhard_inv']
inputimage = ['images/plants3/ros_plant3_1.jpg', 'realhard_inv']
inputimage = ['images/plants1/ros_plant0.jpg', 'simple_inv']
inputimage = ['images/plants1/ros_plant0_1.jpg', 'simple_inv']
inputimage = ['images/plants1/ros_plant0_2.jpg', 'simple_inv']
# inputimage = ['images/plants2/train/ros_plant2_0.jpg', 'realeasy_inv']
# inputimage = ['images/plants2/train/ros_plant2_1.jpg', 'realeasy_inv']
cv_image = cv2.imread(inputimage[0])
# hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

# showPlot(cv_image)
ground, ground_mask = filter_colors(cv_image, 'ground')
ground_inv, ground_inv_mask = filter_colors(cv_image, 'ground_inv')
plant, plant_mask = filter_colors(ground_inv, inputimage[1])
contours, filtered_contours = get_contours(plant, cv_image)
# contours_s = cv2.resize(contours, (0, 0), fx=0.5, fy=0.5)

# cv2.imshow('Mask', plant)
plt.subplot(2, 2, 3)
plt.imshow(plant)
plt.subplot(2, 2, 2)
plt.imshow(ground_mask + plant_mask)
plt.subplot(2, 2, 1)
plt.imshow(cv2.resize(contours, (0, 0), fx=0.5, fy=0.5))
# plt.show()

# cv2.imshow('Mask', ground_mask + plant_mask)
# cv2.imshow('Mask_plant', mask_plant)
# cv2.imshow('final', contours)
cv2.imshow('Contours', contours)
cv2.waitKey(0)
cv2.destroyAllWindows()
