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
    - Selecting three 2D way points on the line
  - 2nd method
    - 
    -
    -
 - Using RealSense API to transform the 2D coordinates into 3D
 - Output: three 3D waypoints
First, I used filter function to extract the red line from the raw image based on HSV space, then based on the edge extracted by Canny edge detection, used Hough line detection algorithm to get the two straight outlines of the lane. Finally, by average over the two lines to get the center of the lane and select three way points in it. To obtain a more robust lane tracking, the second method applied several improvements. After the filtering based on HSV space, I first found the biggest connect component based on the labeling component.Then average over the points coordinates with three selected y coordinate values.

  

## Complete System Overview
