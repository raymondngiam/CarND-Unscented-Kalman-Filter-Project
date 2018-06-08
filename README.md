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

![](http://mathurl.com/y7u5nzww)


