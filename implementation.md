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
- Input: Video stream
- Get 2D way points:
  - 1st method
    - Using filter function from OpenCV to extract the red(color of the lane) region from the raw image based on  HSV space
    - Extracting Edge by Canny edge detection.
    - Using Hough line detection algorithm to get two straight outlines of the lane.
    - Averaging over the two lines to get the middle line.
    - Selecting three 2D way points on the line.
    <p float="left">
  <img src="images/pasted image 0.png" width="210" /> &rarr;
  <img src="images/line.png" width="210" />  &rarr;
  <img src="images/point.png" width="210" />
  
  - 2nd method
    - Using filter function from OpenCV to extract the red(color of the lane) region from the raw image based on  HSV space.
    - Removing noisy pixels by finding the biggest connected component based on connected component labeling.
    - Collect all the pixels in the biggest connected component.
    - Selecting three fixed y coordinates.
    - Averaging over the x coordinates with the same fixed y coordinates to get the 2D way points.
    
    <p float="left">
  <img src="images/method2_white.png" width="210" />  &rarr;
  <img src="images/method2_largest.png" width="210" />  &rarr;
  <img src="images/method2_final.png" width="210" />

 - Using RealSense API to transform the 2D coordinates into 3D
 - Output: Three 3D waypoints
  

### Obstacle Detection
- Input: Depth data from a RealSense camera at 30 FPS.
- Approach
	- Filter depth data to only keep pixels with a certain depth. 
	- Determine depth of ground/red lane line. 
	- Keep all pixels that have a depth greater than the depth of the ground and less than 3 meters. 
		- This removes some extraneous points due to the carpet
	- Use OpenCV to create contours and bounding boxes for all remaining pixel groups
	- Filter the remaining bounding boxes by object position, size, distance, and object height
		- Filtering by object size and height helps remove some incorrect bounding boxes caused by incorrect depth data.



    <p float="left">
  <img src="images/depth_image.png" width="210" />  &rarr;
  <img src="images/depth_mask.png" width="210" />  &rarr;
  <img src="images/depth_bounding_boxes.png" width="210" />
	- Generate 3D location of objects using the RealSense API to convert pixel coordinates and depth to a 3D coordinate
- Output: List of centers of obstacles detected
<img src="images/bounding_box_result.png" width="640" />
## Complete System Overview
