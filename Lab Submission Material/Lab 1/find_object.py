#!/usr/bin/env python
"""find_object.py: Tracking Yellow Circles"""
__author__ = 'Hanyao Guo, Yifei Du'


import cv2
import numpy as np

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

if __name__ == '__main__':

    cap = cv2.VideoCapture(0)

    while True:

        ret, frame = cap.read()
        image = frame

        image = contrasting(image)
        image = np.uint8(np.clip(image[:,:,1] * 0.5 + image[:,:,2] * 0.5 - image[:,:,0] * 0.85,0,255))
        cv2.imshow('Original Image', image)
        cv2.waitKey(1)

        image = shape_adjustment(image)
        image = blurring(image)
        image = cv2.threshold(image,50,100,cv2.THRESH_BINARY)[-1]

        contours, hierarchy = cv2.findContours(image,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
        image = cv2.merge([image,image,image])
        num = len(contours)
        circle_list = []
        for i in range(num):
            S = cv2.contourArea(contours[i])
            P = cv2.arcLength(contours[i],True)
            if S > 0:
                ratio = (P ** 2) / S

                if ratio > 13 and ratio < 14.5:
                    circle_list.append(i)
                    min_x = np.min(contours[i][:,:,0])
                    max_x = np.max(contours[i][:,:,0])
                    min_y = np.min(contours[i][:, :, 1])
                    max_y = np.max(contours[i][:, :, 1])
                    cv2.rectangle(image,(min_x,min_y),(max_x,max_y),color=(255,0,0),thickness=2)
                    print('top-left:',(min_x,min_y),'\tbottom-right:',(max_x,max_y))

        for i in range(len(circle_list)):
            cv2.drawContours(image, contours[circle_list[i]], -1, (0,255,0), 3)

        cv2.imshow('Binary Image',image)
        cv2.waitKey(1)