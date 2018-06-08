# Udacity Self Driving Car Nanodegree
## Unscented Kalman Filter Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. The objective is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. In contrast to a [previous project](https://github.com/raymondngiam/CarND-Extended-Kalman-Filter-Project) which uses extended Kalman filter with a constant velocity motion model, this project implements an unscented Kalman filter using the constant turn rate and velocity (CTRV) motion model.

---

### Video Demo

![Demo](/images/small.gif)

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

---

### Implementation Summary

**State Transition Model**

Using a CTRV model, the process model is as described below:

![](https://latex.codecogs.com/gif.latex?\begin{pmatrix}p_{x}'&space;%5C&space;p_{y}'&space;%5C&space;v'&space;%5C&space;\psi&space;'&space;%5C&space;\dot{\psi}'&space;\end{pmatrix}&space;=&space;\begin{pmatrix}p_{x}&space;%5C&space;p_{y}&space;%5C&space;v&space;%5C&space;\psi&space;%5C&space;\dot{\psi}&space;\end{pmatrix}&space;&plus;&space;\begin{pmatrix}\frac{v}{\dot{\psi}}(sin(\psi&plus;\dot{\psi}\Delta&space;t)-sin(\psi))&space;%5C&space;\frac{v}{\dot{\psi}}(-cos(\psi&plus;\dot{\psi}\Delta&space;t)&plus;cos(\psi))&space;%5C&space;0&space;%5C&space;\psi\Delta&space;t&space;%5C&space;0&space;\end{pmatrix}&space;&plus;&space;\begin{pmatrix}0.5(\Delta&space;t)^{2}cos(\psi)\cdot&space;\nu&space;_{a}&space;%5C&space;0.5(\Delta&space;t)^{2}sin(\psi)\cdot&space;\nu&space;_{a}&space;%5C&space;\Delta&space;t&space;\cdot&space;\nu&space;_{a}&space;%5C&space;0.5(\Delta&space;t)^{2}\cdot&space;\nu&space;_{\ddot{\psi}}&space;%5C&space;\Delta&space;t&space;\cdot&space;\nu&space;_{\ddot{\psi}}&space;\end{pmatrix})


