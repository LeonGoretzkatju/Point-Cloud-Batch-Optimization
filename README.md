# Direct Batch Optimization for Multiview Registration

## Author: Xiangchen Liu, Yingyu Wang, Liang Zhao

This is the MATLAB code for implementation of our paper "Direct Batch Optimization for Multiview Registration", in this work, we simultaneously optimize robot poses and global 3D grid map, which is totally different from current method which depends on the optimized poses first then use optimized poses to update global map.

### TODO LIST:

Jacobian of observation term w.r.t robot poses $J_p$ is okay, but Jacobian of observation term w.r.t global map $J_D$ has some problem.
