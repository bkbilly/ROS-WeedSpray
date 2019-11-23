#!/usr/bin/env python3

import cv2
import numpy as np
import matplotlib.pyplot as plt  # load image using cv's imread(nameoffile)

img = cv2.imread('realeasy.png')

gscale = cv2.Canny(img, 1, 360, apertureSize=3)

plt.plot(), plt.imshow(gscale)
plt.show()

