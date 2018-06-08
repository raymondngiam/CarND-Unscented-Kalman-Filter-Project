# Udacity Self Driving Car Nanodegree
## Unscented Kalman Filter Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. The objective is to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. In contrast to a [previous project](https://github.com/raymondngiam/CarND-Extended-Kalman-Filter-Project) which uses extended Kalman filter with a constant velocity (CV) motion model, this project implements an unscented Kalman filter using the constant turn rate and velocity (CTRV) motion model.

---

### Video Demo

![Demo](/images/small.gif)

Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

---

### Implementation Summary

**State Transition Model**

A CTRV motion model is depicted in the diagram below:

<img src="/images/screenshot-from-2017-02-27-20-45-49.png" width="500">

Using a CTRV model, the process model is as described below:

![](https://latex.codecogs.com/gif.latex?\mathbf{x'}=f(\mathbf{x},\mathbf{\nu}))

where ![](https://latex.codecogs.com/gif.latex?\mathbf{x}) is the state vector and ![](https://latex.codecogs.com/gif.latex?\mathbf{\nu}) is the process noise vector.

![](https://latex.codecogs.com/gif.latex?%5Cmathbf%7Bx%7D%3D%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%5C%5Cp_%7By%7D%5C%5Cv%5C%5C%5Cpsi%5C%5C%5Cdot%7B%5Cpsi%7D%5Cend%7Bpmatrix%7D)

![](https://latex.codecogs.com/gif.latex?%5Cmathbf%7B%5Cnu%7D%3D%5Cbegin%7Bpmatrix%7D%5Cnu_%7Ba%7D%5C%5C%5Cnu_%7B%5Cddot%7B%5Cpsi%7D%7D%5Cend%7Bpmatrix%7D)

![](https://latex.codecogs.com/gif.latex?\nu_{a}) is the longitudinal acceleration noise, and is normally distributed with mean zero and variance ![](https://latex.codecogs.com/gif.latex?\sigma_{a}^{2})

![](https://latex.codecogs.com/gif.latex?\nu_{a}\sim&space;\mathcal{N}(0,\sigma_{a}^{2}))

![](https://latex.codecogs.com/gif.latex?\ddot{\psi}) is the yaw acceleration noise, and is normally distributed with mean zero and variance ![](https://latex.codecogs.com/gif.latex?\sigma_{\ddot{\psi}}^{2})

![](https://latex.codecogs.com/gif.latex?\nu_{\ddot{\psi}}\sim&space;\mathcal{N}(0,\sigma_{\ddot{\psi}}^{2})))

The CTRV model can then be represented as follows:

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%27%20%5C%5C%20p_%7By%7D%27%20%5C%5C%20v%27%20%5C%5C%20%5Cpsi%20%27%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%27%20%5Cend%7Bpmatrix%7D%20%3D%20%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%20%5C%5C%20p_%7By%7D%20%5C%5C%20v%20%5C%5C%20%5Cpsi%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5Cend%7Bpmatrix%7D%20%2B%20%5Cbegin%7Bpmatrix%7D%5Cfrac%7Bv%7D%7B%5Cdot%7B%5Cpsi%7D%7D%28sin%28%5Cpsi%2B%5Cdot%7B%5Cpsi%7D%5CDelta%20t%29-sin%28%5Cpsi%29%29%20%5C%5C%20%5Cfrac%7Bv%7D%7B%5Cdot%7B%5Cpsi%7D%7D%28-cos%28%5Cpsi%2B%5Cdot%7B%5Cpsi%7D%5CDelta%20t%29%2Bcos%28%5Cpsi%29%29%20%5C%5C%200%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%5CDelta%20t%20%5C%5C%200%20%5Cend%7Bpmatrix%7D%20%2B%20%5Cbegin%7Bpmatrix%7D0.5%28%5CDelta%20t%29%5E%7B2%7Dcos%28%5Cpsi%29%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%200.5%28%5CDelta%20t%29%5E%7B2%7Dsin%28%5Cpsi%29%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%200.5%28%5CDelta%20t%29%5E%7B2%7D%5Ccdot%20%5Cnu%20_%7B%5Cddot%7B%5Cpsi%7D%7D%20%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu%20_%7B%5Cddot%7B%5Cpsi%7D%7D%20%5Cend%7Bpmatrix%7D) when ![](https://latex.codecogs.com/gif.latex?\dot{\psi}\neq0)

and

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%27%20%5C%5C%20p_%7By%7D%27%20%5C%5C%20v%27%20%5C%5C%20%5Cpsi%20%27%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%27%20%5Cend%7Bpmatrix%7D%20%3D%20%5Cbegin%7Bpmatrix%7Dp_%7Bx%7D%20%5C%5C%20p_%7By%7D%20%5C%5C%20v%20%5C%5C%20%5Cpsi%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5Cend%7Bpmatrix%7D%20%2B%20%5Cbegin%7Bpmatrix%7Dv%5Ccdot%20cos%28%5Cpsi%29%5Ccdot%20%5CDelta%20t%20%5C%5C%20v%5Ccdot%20sin%28%5Cpsi%29%5Ccdot%20%5CDelta%20t%20%5C%5C%200%20%5C%5C%20%5Cdot%7B%5Cpsi%7D%5CDelta%20t%20%5C%5C%200%20%5Cend%7Bpmatrix%7D%20%2B%20%5Cbegin%7Bpmatrix%7D0.5%28%5CDelta%20t%29%5E%7B2%7Dcos%28%5Cpsi%29%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%200.5%28%5CDelta%20t%29%5E%7B2%7Dsin%28%5Cpsi%29%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu%20_%7Ba%7D%20%5C%5C%200.5%28%5CDelta%20t%29%5E%7B2%7D%5Ccdot%20%5Cnu%20_%7B%5Cddot%7B%5Cpsi%7D%7D%20%5C%5C%20%5CDelta%20t%20%5Ccdot%20%5Cnu%20_%7B%5Cddot%7B%5Cpsi%7D%7D%20%5Cend%7Bpmatrix%7D) when ![](https://latex.codecogs.com/gif.latex?\dot{\psi}=0)

**Measurement Models**

Details regarding the measurement model for lidar and radar were covered in the previous project on [extended Kalman filter](https://github.com/raymondngiam/CarND-Extended-Kalman-Filter-Project).

**Unscented Kalman Filter - Overview**

When dealing with nonlinear process or measurement models, the extended Kalman filter uses the Jacobian matrix to linearize nonlinear functions. The unscented Kalman filter, on the other hand, does not need to linearize non-linear functions; instead, the unscented Kalman filter takes representative points from a Gaussian distribution, a.k.a. sigma points. These points will be plugged into the nonlinear equations, and based on these nonlinearly transformed sigma points, new mean and covariance of the group of sigma points are computed. These will not provide the real mean and covariance as the real predicted distribution, but in many cases, it gives a useful approximation.

The following figure illustrates the rough idea of an unscented transformation:

<img src="/images/Screenshot from 2018-06-08 22-04-35-2.png" width="500">

**Unscented Kalman Filter - Augmentation and Sigma Point Generation**

![](https://latex.codecogs.com/gif.latex?n_{a}=7)

![](https://latex.codecogs.com/gif.latex?x_%7Ba%2Ck%7D%3D%5Cbegin%7Bpmatrix%7D%0D%0Ap_%7Bx%7D%5C%5C%0D%0Ap_%7Bx%7D%5C%5C%0D%0Av%5C%5C%0D%0A%5Cpsi%5C%5C%0D%0A%5Cdot%7B%5Cpsi%7D%5C%5C%0D%0A%5Cnu_%7Ba%7D%5C%5C%0D%0A%5Cnu_%7B%5Cddot%7B%5Cpsi%7D%7D%0D%0A%5Cend%7Bpmatrix%7D)

![](https://latex.codecogs.com/gif.latex?%5Cmathbf%7BP%7D_%7Ba%2Ck%7Ck%7D%3D%5Cbegin%7Bbmatrix%7D%5Cmathbf%7BP%7D_%7Ba%2Ck%7Ck%7D%260%5C%5C0%26%5Cmathbf%7BQ%7D%5Cend%7Bbmatrix%7D)

![](https://latex.codecogs.com/gif.latex?%5Cmathcal%7BX%7D_%7Ba%2Ck%7Ck%7D%3D%5Cbegin%7Bbmatrix%7D%0D%0Ax_%7Ba%2Ck%7Ck%7D%26x_%7Ba%2Ck%7Ck%7D%2B%5Csqrt%7B%28%5Clambda%2Bn_%7Ba%7D%29%5Cmathbf%7BP%7D_%7Ba%2Ck%7Ck%7D%29%7D%26x_%7Ba%2Ck%7Ck%7D-%5Csqrt%7B%28%5Clambda%2Bn_%7Ba%7D%29%5Cmathbf%7BP%7D_%7Ba%2Ck%7Ck%7D%29%7D%0D%0A%5Cend%7Bbmatrix%7D)

where

![](https://latex.codecogs.com/gif.latex?%5Clambda%3D3-n_%7Ba%7D)

**Unscented Kalman Filter - Predicting Sigma Points and Calculating Mean and Covariance**

<img src="/images/Screenshot from 2018-06-08 23-07-49-2.png" width="750">

**Unscented Kalman Filter - Measurement Prediction**
