#!/usr/bin/env python
"""local_view_image.py: Node:local_view_image"""
__author__ = 'Hanyao Guo, Yifei Du'

import numpy as np
import cv2

def crop_img(img, area_threshold = 0.002,
             top_crop_percent = 20, bottom_crop_percent = 25, padding_width = 30):
    r_lower = (0, 127, 76)
    r_upper = (10, 204, 229)
    rr_lower = (170, 153, 76)
    rr_upper = (180, 204, 229)
    b_lower = (100, 102, 12)
    b_upper = (125, 204, 102)
    g_lower = (55, 51, 25)
    g_upper = (80, 229, 204)

    color_ranges = [
        [r_lower, r_upper],  # Example color range for the sign
        [rr_lower, rr_upper],
        [b_lower, b_upper],
        [g_lower, g_upper]  # Add more color ranges if needed
    ]
    # Read the image
    original_img = img.copy()
    # Calculate the crop percentages in pixels and crop the img
    top_crop = int(top_crop_percent / 100 * original_img.shape[0])
    bottom_crop = int(bottom_crop_percent / 100 * original_img.shape[0])
    cropped_img = original_img[top_crop:-bottom_crop, :]
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    # Initialize variables for the maximum contour and its area
    max_contour = None
    max_contour_area = 0

    # indicating whether an object has been found
    find_object = False

    # Iterate over the provided color ranges
    for i in range(len(color_ranges)):
        
        # Create a binary mask using the current color range
        color_range = color_ranges[i]
        lower_color = np.array(color_range[0])
        upper_color = np.array(color_range[1])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the contour with the maximum area among the provided color ranges
        if contours:
            contour_max = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour_max)
            # contour so large that it is not possible to be noise
            if area > 100:
                if i > 1 and find_object:
                    return None
                else:
                    find_object = True

            if area > max_contour_area:
                max_contour = contour_max
                max_contour_area = area

    # Check if a valid contour is found
    if max_contour is None:
        return None

    # Calculate the contour area to original image area ratio and check if > threshold
    img_area = img.shape[0] * img.shape[1]
    ratio = max_contour_area / img_area
    if ratio < area_threshold:
        return None

    # Create a bounding box around the sign and crop img
    x, y, w, h = cv2.boundingRect(max_contour)
    # Expand the bounding box by the specified padding width
    x = max(0, x - padding_width)
    y = max(0, y - padding_width)
    w = min(original_img.shape[1] - x, w + 2 * padding_width)
    h = min(original_img.shape[0] - y, h + 2 * padding_width)
    # Draw the expanded bounding box on the original image
    cropped_img = original_img[y + top_crop:y + h + top_crop, x:x + w]

    return cropped_img