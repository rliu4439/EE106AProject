---
title: Conclusion
layout: template
filename: conclusion
order: 4
mathjax: true
--- 

# Conclusion

Our finished solution met most of our design criteria. We were able to build a vehicle with custom components that was able to follow a lane line on a track with multiple turns with a moderate movement speed. We also were able to detect and generate bounding boxes for obstacles based on our depth images. Our traction controller was able to prevent slip from occuring on low friction surfaces. Unfortunately, we did not have enough time to fully test and integrate the obstacle detection algorithm with the controller. 

## Difficulties Faced

**Software Compatibility issues**
- Due to all the software running on a Jetson Nano, there were multiple cases when the software we needed was unavailable or unable to install on the vehicle. For example, one library that we needed, CVXPY, was unable to install on the Jetson Nano. This required us to implement QP by hand instead of using a prebuilt package.

**RealSense Camera issues**
- The RealSense camera has a limited frame rate of 30 FPS for color and depth images. This prevented us from moving at higher speeds since we were unable to get the image data we needed. In addition, the depth camera data was very sensitive to lighting and some pixels had incorrect depth labelling. Even with adjustments to some of the camera settings to increase the accuracy, the incorrect labelling caused the appearance of extra obstacles which needed to be filtered. The camera was also unable to detect smaller obstacles that were a couple meters away so the car would need to be much closer to small obstacles before it could avoid it. 

