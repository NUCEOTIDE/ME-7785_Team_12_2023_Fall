#!/usr/bin/env python
"""local_view_image.py: Node:local_view_image"""
__author__ = 'Hanyao Guo, Yifei Du'



import numpy as np
import cv2

import os
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

def shape_adjustment(image):
    width, length = image.shape
    new_image = cv2.resize(image, (int(length / 2), int(width / 2)))
    return new_image

def blurring(image):
    new_image = cv2.GaussianBlur(image, ksize=(21, 21), sigmaX=0)
    new_image = cv2.medianBlur(new_image, ksize=7)
    return new_image

def contrasting(image):
    max_pixel = np.max(image)
    min_pixel = np.min(image)
    delta = max_pixel - min_pixel
    new_image = (image - min_pixel) * (255 / delta)
    return np.uint8(new_image)

def crop_img(image_path, color_ranges, area_threshold = 0.02, 
             top_crop_percent = 20, bottom_crop_percent = 25, 
             blur_kernel_size = (15, 15), padding_width = 30):
    # area_threshold = 0.2
    hsv_ranges = []

    # Read the image
    img = cv2.imread(image_path)
    original_img = img.copy()
    # Calculate the crop percentages in pixels and crop the img
    top_crop = int(top_crop_percent / 100 * original_img.shape[0])
    bottom_crop = int(bottom_crop_percent / 100 * original_img.shape[0])
    cropped_img = original_img[top_crop:-bottom_crop, :]
    cropped_img = blurring(cropped_img)
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    
    # Initialize variables for the maximum contour and its area
    max_contour = None
    max_contour_area = 0

    # Iterate over the provided color ranges
    for color_range in color_ranges:
        
        # Create a binary mask using the current color range
        lower_color = np.array(color_range[0])
        upper_color = np.array(color_range[1])
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Find the contour with the maximum area among the provided color ranges
        if contours:
            contour_max = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour_max)

            if area > max_contour_area:
                max_contour = contour_max
                max_contour_area = area

    # Check if a valid contour is found
    if max_contour is None:
        print("No valid contour found. No cropping.")
        return img

    # Calculate the contour area to original image area ratio and check if > threshold
    img_area = img.shape[0] * img.shape[1]
    ratio = max_contour_area / img_area
    print("ratio is: " + str(ratio))
    if ratio < area_threshold:
        print("Contour area ratio is below the threshold. No cropping.")
        return img

    # Create a bounding box around the sign and crop img
    x, y, w, h = cv2.boundingRect(max_contour)
    # Expand the bounding box by the specified padding width
    x = max(0, x - padding_width)
    y = max(0, y - padding_width)
    w = min(original_img.shape[1] - x, w + 2 * padding_width)
    h = min(original_img.shape[0] - y, h + 2 * padding_width)
    # Draw the expanded bounding box on the original image
    cv2.rectangle(original_img, (x, y + top_crop), (x + w, y + h + top_crop), (0, 255, 0), 2)
    cropped_img = original_img[y + top_crop:y + h + top_crop, x:x + w]

    # Display the original and cropped images
    cv2.imshow("Original Image", original_img)
    cv2.imshow("Masked Image", mask)
    cv2.imshow("Cropped Image", cropped_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return cropped_img

# Example usage:
directory_path = "/Users/jeffdu/Library/CloudStorage/OneDrive-Personal/Academic OneDrive/Courses Grad/Robotics Fundamental Courses/BME 7785 Introduction to Robotics Research/2023Fimgs/"
count = 0
# Define the range of colors for the sign in HSV
r_lower = (0, 25, 30)
r_upper = (20, 80, 90)
rr_lower = (340, 25, 30)
rr_upper = (360, 80, 90)
b_lower = (200, 20, 5)
b_upper = (250, 70, 60)
g_lower = (110, 20, 10)
g_upper = (160, 90, 80)

color_ranges = [
    [r_lower, r_upper],  # Example color range for the sign
    [rr_lower, rr_upper],
    [b_lower, b_upper],
    [g_lower, g_upper]   # Add more color ranges if needed
]
hsv_ranges_opencv = adjust_range(color_ranges) # Adjust to opencv hsv ranges
# Iterate directory
for path in os.listdir(directory_path):
    # check if current path is a file
    if os.path.isfile(os.path.join(directory_path, path)):
        count += 1
for img_num in range(1, count):
    input_image_path = directory_path + str(img_num) + ".jpg"
    if os.path.isfile(input_image_path):
        output_image = crop_img(input_image_path, hsv_ranges_opencv, 0.002)


