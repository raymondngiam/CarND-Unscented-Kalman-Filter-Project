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

A CTRV motion model is depicted in the diagram below:

![CTRV Motion Model](/images/screenshot-from-2017-02-27-20-45-49.png)

Using a CTRV model, the process model is as described below:

![](https://latex.codecogs.com/gif.latex?\mathbf{x'}=f(\mathbf{x},\mathbf{\nu}))

where ![](https://latex.codecogs.com/gif.latex?\mathbf{x}) is the state vector and ![](https://latex.codecogs.com/gif.latex?\mathbf{\nu}) is the process noise vector.

![](https://latex.codecogs.com/gif.latex?%5Cmathbf%7Bx%7D%3D%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%5C%5Cp_%7By%7D%5C%5Cv%5C%5C%5Cpsi%5C%5C%5Cdot%7B%5Cpsi%7D%5Cend%7Bpmatrix%7D)

![](https://latex.codecogs.com/gif.latex?%5Cmathbf%7B%5Cnu%7D%3D%5Cbegin%7Bpmatrix%7D%5Cnu_%7Ba%7D%5C%5C%5Cnu_%7B%5Cddot%7B%5Cpsi%7D%7D%5Cend%7Bpmatrix%7D)

and

![](https://latex.codecogs.com/gif.latex?\nu_{a}\sim&space;\mathcal{N}(0,\sigma_{a}^{2}))

![](https://latex.codecogs.com/gif.latex?\nu_{\ddot{\psi}}\sim&space;\mathcal{N}(0,\sigma_{\ddot{\psi}}^{2})))

The CTRV model can then be represented as follows:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%27%20%5C%5C%20p_%7By%7D%27%20%5C%5C%20v%27%20%5C%5C%20%5Cpsi%20%27%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%27%20%5Cend%7Bpmatrix%7D%20%3D%20%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%20%5C%5C%20p_%7By%7D%20%5C%5C%20v%20%5C%5C%20%5Cpsi%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5Cend%7Bpmatrix%7D%20%2B%20%5Cbegin%7Bpmatrix%7D%5Cfrac%7Bv%7D%7B%5Cdot%7B%5Cpsi%7D%7D%28sin%28%5Cpsi%2B%5Cdot%7B%5Cpsi%7D%5CDelta%20t%29-sin%28%5Cpsi%29%29%20%5C%5C%20%5Cfrac%7Bv%7D%7B%5Cdot%7B%5Cpsi%7D%7D%28-cos%28%5Cpsi%2B%5Cdot%7B%5Cpsi%7D%5CDelta%20t%29%2Bcos%28%5Cpsi%29%29%20%5C%5C%200%20%5C%5C%20%5Cpsi%5CDelta%20t%20%5C%5C%200%20%5Cend%7Bpmatrix%7D%20%2B%20%5Cbegin%7Bpmatrix%7D0.5%28%5CDelta%20t%29%5E%7B2%7Dcos%28%5Cpsi%29%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%200.5%28%5CDelta%20t%29%5E%7B2%7Dsin%28%5Cpsi%29%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%200.5%28%5CDelta%20t%29%5E%7B2%7D%5Ccdot%20%5Cnu%20_%7B%5Cddot%7B%5Cpsi%7D%7D%20%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu%20_%7B%5Cddot%7B%5Cpsi%7D%7D%20%5Cend%7Bpmatrix%7D) when ![](https://latex.codecogs.com/gif.latex?\dot{\psi}\neq0)

where




