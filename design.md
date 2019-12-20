---
title: Design
layout: template
filename: design
order: 1
--- 

# Design

## Traction Control

### Motivation
High performance vehicles at both the full size and RC size scales are typically limited primarily by the total amount of frictional force that their tires can provide. This friction is needed to accelerate, brake, and turn, and much research has gone into figuring out how to increase the total amount of traction available. Under high speed cornering or sudden acceleration conditions, the amount of frictional force needed to maintain static friction between the ground and the tire can exceed what the tire is able to provide. When this occurs, the tire starts sliding instead of rolling (often referred to as "wheel slip") and vehicle traction and acceleration are both lost. Traction control algorithms are designed to detect when wheel slip occurs and modulate the throttle so that the vehicle can regain traction as quickly as possible.

### Existing Research
Research into existing traction control algorithms revealed an [IEEE paper](https://ieeexplore-ieee-org.libproxy.berkeley.edu/document/6402343) titled *Model Predictive PID Traction Control Systems for Electric Vehicles* published by Tohru Kawabe. In this paper, Kawabe quantifies wheel slip using the slip ratio <img src="https://render.githubusercontent.com/render/math?math=\lambda"> defined as
<img src="https://render.githubusercontent.com/render/math?math=\lambda = \frac{V_{\omega}V}{V_\omega}">
where <img src="https://render.githubusercontent.com/render/math?math=V_{\omega}"> is the wheel velocity and $V$ is the vehicle body velocity. Physically, a slip ratio of 0 corresponds to wheel velocity exactly matching body velocity. A positive slip ratio corresponds to wheel velocity exceeding body velocity, such as traction loss during acceleration from a stand still when the torque sent to the tire exceeds the frictional force available. A negative slip ratio corresponds to body velocity exceeding wheel velocity, such as traction loss during high speed cornering when the tire is no longer able to continue rolling and begins sliding.
The exact relationship between slip ratio and the amount of frictional force that the tire can provide is related through a formula called the Magic-Formula developed through testing data:
<a href="https://www.codecogs.com/eqnedit.php?latex=\mu(\lambda)=-c_{road}\times&space;1.1\times(e^{-35\lambda}-e^{-0.35\lambda})" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\mu(\lambda)=-c_{road}\times&space;1.1\times(e^{-35\lambda}-e^{-0.35\lambda})" title="\mu(\lambda)=-c_{road}\times 1.1\times(e^{-35\lambda}-e^{-0.35\lambda})" /></a>
where $\mu$ is the coefficient of friction and $c_{road}$ is a parameter that depends on the condition of the road being driven on.
Plotting this equation yields the following graph:
<img src="images/magicformula.gif" width="480" height="auto">
which indicates that in general, <img src="https://render.githubusercontent.com/render/math?math=V_{\\lambda}"> is maximized at a slip ratio of 0.1. This therefore drove the design of our traction controller to limit wheel slip ratio to a range of 0.05 to 0.15 with a desired slip ratio of 0.1.