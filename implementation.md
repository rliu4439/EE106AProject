---
title: Implementation
layout: template
filename: implementation
order: 2
--- 

# Implementation


## Hardware Used and Built

## Parts of Design

## Software
The software for this project was written using Python on a Linux machine.
### Lane tracking
- Input: video stream
- Get 2D way points:
  - 1st method
    - Using filter function from OpenCV to extract the red(color of the lane) region from the raw image based on  HSV space
    - Extracting Edge by Canny edge detection.
    - Using Hough line detection algorithm to get two straight outlines of the lane.
    - Averaging over the two lines to get the middle line.
    - Selecting three 2D way points on the line.
  - 2nd method
    - Using filter function from OpenCV to extract the red(color of the lane) region from the raw image based on  HSV space.
    - Removing noisy pixels by finding the biggest connected component based on connected component labeling.
    - Collect all the pixels in the biggest connected component
    - Selecting three fixed y coordinates
    - Averaging over the x coordinates with the same fixed y coordinates to get the 2D way points.
 - Using RealSense API to transform the 2D coordinates into 3D
 - Output: three 3D waypoints
  

## Complete System Overview
