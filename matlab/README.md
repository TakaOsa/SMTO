# Matlab implementation of SMTO

This folder contains the matlab implementation of SMTO, which was used for the 2D experiments in the [paper](https://arxiv.org/abs/2003.07054).
Since the optimization with SMTO is a stochastic process, the result will be different for each run.

## How to use

`FourLinkSMTO_demo.m`
shows the example of the trajectory optimization for a 2D four-link manipulator.

`FourLinkSMTOend_demo.m`
shows the example of optimizing a trajectory including the goal orientation of the end-effector for a 2D four-link manipulator.

`PathPlanning2D_demo.m`
shows the example of path planning on 2D space. This example does not use the trajectory update with CHOMP.

`WeightedDensityEstimation_demo.m`
shows the behavior of the cost-weighted density estimation. You can see  Figure 1 in the [paper](https://arxiv.org/abs/2003.07054).

All the codes in this repository are shared under  a CC Attribution-NonCommerical license.  
