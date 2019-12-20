---
title: Team
layout: template
filename: team
mathjax: true
order: 5
--- 

# Team Bios

### Mike Estrada

- **Bio**: I'm a 1st year PhD student in EECS studying Control Systems and Robotics.

- **Contributions**: I had two main contributions to this project. First, I was responsible for prototyping and designing the wheel encoders and the associated arduino software in collaboration with Yuri Murakami.  This invovlved the iterative design and 3D printing of encoder disks that were installed on the inside of the wheels to trigger line sensors as the wheels rotated. The rate at which each sensor was triggered allowed us to estimate the velocity of the four wheels independently and average their values to get an estimate of the body velocity. Second, I was responsible for writing the LQR controller that was intended to be the basis of the Model Predictive Control approach we planned on implementing. Due to software compatability issues, this required me to reformulate the LQR problem as a quadratic program and manually write the formulation for a quadratic program solver.

### Lu Yu
- **Bio**: I'm a 1st year EECS master student interested in robust computer vision and image processing. 

- **Contributions**: My main task involved designing the lane traking algorithm for the car. This involved collecting data, reaserching existing lane traking algorithms, installing the appropriate libraries on the vehicle, designing the algorithms and testing the methods on control algorithms. I came up with two methods to detect the lane and generate the way points by some robust image processing based on OpenCV and used the RealSense API to translate the 2D coordinates to 3D which can be used by the control algorithms. This function would use video streams as input and output the three 3D way points that would be used by the LQR controller. I also helped with the testing and debugging of the controls algorithms used for line following.

### Ryan Liu
- **Bio**: I'm a 4th year EECS major interested in machine learning and image processing. 

- **Contributions**: My main task involved creating the object detection system for the car. This involved collecting data, installing the appropriate libraries on the vehicle and designing the algorithm. I used OpenCV to detect objects in the images and used the RealSense API to calculate the real-world position of an object which was also needed for line following. This function would output the coordinate of objects so the controller could avoid them. I also worked on adjustments to the RealSense camera's configuration to improve the depth data output. I helped with the testing and debugging of the controls algorithms used for line following.

### Jake Ramirez
- **Bio**: I'm a 5th year Masters student in Mechanical Engineering interested in electric vehicles.

- **Contributions**: My main contributions included writing the code for the line following controller and integrating team member's code together. I wrote the LQR Controller which solves the Riccati equation for the simplified bicycle model as well as developed our "Smart" PID Controller which was used to debug vision issues more easily. I tuned the LQR gains in Simulink and helped create our object avoidance controller. I used prewritten code to determine the reachset for our chosen obstacle size as well as implement a controller with object avoidance capabilities in simulation.

### Kevin Chen

- **Bio**: I'm a 4th year Mechanical Engineering undergrad interested in aerospace applications, particularly space flight and exploration.

- **Contributions**: My main contributions included writing the code for the traction control algorithm and testing the controller. This involved researching existing control algorithms, choosing a simple proportional controller to limit wheel slip ratio, and collecting wheel velocity data to observe the effects of the controller. I also assisted with debugging and fixing the hardware.
