---
title: Conclusion
layout: template
filename: conclusion
order: 4
mathjax: true
--- 

# Conclusion

Our finished solution met most of our design criteria. We were able to build a vehicle with custom components that was able to follow a lane line on a track with multiple turns with a moderate movement speed. We also were able to detect and generate bounding boxes for obstacles based on our depth images. Our traction controller was able to prevent slip from occurring on low friction surfaces. Unfortunately, we did not have enough time to fully test and integrate the obstacle detection algorithm with the controller. 

## Difficulties Faced

**Software Compatibility issues**
- Due to all of the software running on the Jetson Nano, there were multiple cases when the software we needed was unavailable or unable to install on the vehicle. For example, one library that we needed, CVXPY, was unable to install on the Jetson Nano. This required us to implement QP by hand instead of using a prebuilt package.

**RealSense Camera issues**
- The RealSense camera has a limited frame rate of 30 FPS for color and depth images. This prevented us from moving at higher speeds since we were unable to get fast enough image data. In addition, the depth camera data was very sensitive to lighting and some pixels had incorrect depth labeling. Even with adjustments to some of the camera settings to increase the accuracy, the incorrect labeling caused the appearance of extra obstacles which needed to be filtered. The camera was also unable to detect smaller obstacles that were a few meters away so the car would need to be much closer to small obstacles before it could avoid it.

**Hardware Failures**
- Throughout the project, we had to prototype and design hardware like the encoders and build out our car. We faced some hardware failures such as being unable to control the car with a joystick and seemingly random shut offs when using the Jetson Nano. Additionally, the original steering servo broke the day before our presentation, forcing us to spend time searching for and installing a new servo. 

## Future Work and Improvements

Future improvements that could be made include iterating on and refining the encoder design in order to improve its accuracy and reliability. Future iterations could also improve upon the precision for wheel velocity measurements. We would also like to fully implement and tune MPC using QP formulation and the CBF constraints. Additional testing could also be done with the traction controller to ensure that slip ratio is effectively controlled in cornering situations, as our testing was limited to straight line acceleration.

Another improvement that could be made is having a dynamic controller that would allow seamless hand-off to a human operator. This would allow the car to drive autonomously until a human operator requests that the controls be passes to them. Even in this scenario, some functionality such as object detection and avoidance would be active to prevent the human driver from driving into obstacles like a wall.

Additionally, creating a more robust object detection system would be another improvement as the current system will occasionally produce bounding boxes due to inaccuracies in the depth data. Although the box only shows up in one frame and will disappear after, the controller would perform better without these inaccuracies. With the current system, we had to perform tuning and filtering to remove incorrect boxes and more testing and tuning would be needed to improve its performance in different situations and environments.