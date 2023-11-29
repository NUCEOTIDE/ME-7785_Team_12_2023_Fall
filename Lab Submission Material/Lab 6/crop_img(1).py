#!/usr/bin/env python
"""local_view_image.py: Node:local_view_image"""
__author__ = 'Hanyao Guo, Yifei Du'



import numpy as np
import cv2

def adjust_range(color_ranges):
    hsv_ranges = []

    for color_bound in color_ranges:
        # Extract individual values from the color bounds
        h_lower, s_lower, v_lower = color_bound[0]
        h_upper, s_upper, v_upper = color_bound[1]

        # Scale the values to OpenCV HSV ranges
        h_lower_opencv = int(h_lower / 2)  # OpenCV uses values in the range [0, 179] for H
        s_lower_opencv = int(s_lower * 2.55)  # OpenCV uses values in the range [0, 255] for S
        v_lower_opencv = int(v_lower * 2.55)  # OpenCV uses values in the range [0, 255] for V

        h_upper_opencv = int(h_upper / 2)
        s_upper_opencv = int(s_upper * 2.55)
        v_upper_opencv = int(v_upper * 2.55)

        # Append the HSV range to the list
        hsv_ranges.append([(h_lower_opencv, s_lower_opencv, v_lower_opencv),
                            (h_upper_opencv, s_upper_opencv, v_upper_opencv)])

    return hsv_ranges


def blurring(image):
    new_image = cv2.GaussianBlur(image, ksize=(21, 21), sigmaX=0)
    new_image = cv2.medianBlur(new_image, ksize=7)
    return new_image

def crop_img(img, area_threshold = 0.002,
             top_crop_percent = 20, bottom_crop_percent = 25, 
             blur_kernel_size = (15, 15), padding_width = 30):
    r_lower = (0, 127.5, 76.5)
    r_upper = (10, 204, 229.5)
    rr_lower = (170, 153, 76.5)
    rr_upper = (179, 204, 229.5)
    b_lower = (100, 102, 12.75)
    b_upper = (125, 204, 102)
    g_lower = (55, 51, 25.5)
    g_upper = (80, 229.5, 204)

    color_ranges = [
        [r_lower, r_upper],  # Example color range for the sign
        [rr_lower, rr_upper],
        [b_lower, b_upper],
        [g_lower, g_upper]  # Add more color ranges if needed
    ]
    # color_ranges = adjust_range(color_ranges)  # Adjust to opencv hsv ranges
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