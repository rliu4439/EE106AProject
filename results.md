---
title: Results
layout: template
filename: results
order: 3
mathjax: true
--- 

# Results

Our vehicle is able to successfully follow a red lane line and perform close to 90 degree turns at a moderate speed. It is also able to detect obstacles in its path and determine the location of each obstacle. On low friction surfaces, the traction controller prevents the vehicle from slipping, allowing the car to continue at a higher speed on slippery surfaces. 

## Line Following

{% include youtubePlayer.html id="9pow2xkvkpc" %}
{% include youtubePlayer.html id="4Ti7HfuU9rI" %}


## Obstacle Detection
{% include youtubePlayer.html id="MYUPyN8ZT10" %}

### Control Barrier Function
{% include youtubePlayer.html id="Vc6IkMbb7Xk" %}

## Traction Control

### Example of wheel slip

{% include youtubePlayer.html id="f4ebHDm4GFc" %}

### Test Data

Wheel speeds from each wheel of the car were recorded to observe the effect of our traction controller. The velocities of the front wheels were averaged to yield the front axle velocity, and the same with the rear to yield the rear axle velocity. In an ideal case with no slip the rear and front velocities should match. By applying tape to the front tires of the car to reduce their friction we expect the front tires to lose traction and spin freely compared to the rears, which should appear in the data as the front velocity consistently being greater than the rear.

<p><img src="images/tracdata.png" width="600" height="auto" style="display:block; margin: 0 auto" ></p>

This sample of data from one of our test runs with traction control enabled shows this behavior initially, with front velocity higher than the rear velocity. But with the traction controller limiting the motor power, we see a dip in the velocity of both axles as the controller throttles down the power to attempt to reach a slip ratio of 0.1. Once this is obtained, the velocities both increase again but now stay around the same value, indicating that the controller is effective in reducing wheel slip.

## Autonomous Driving

{% include youtubePlayer.html id="bSmr1zyeXP4" %}
