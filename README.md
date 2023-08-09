# Direct Batch Optimization for Multiview Registration

## Author: Xiangchen Liu

This is the MATLAB code for implementation of our paper "Direct Batch Optimization for Multiview Registration", in this work, we simultaneously optimize robot poses and global 3D grid map, which is totally different from current method which depends on the optimized poses first then use optimized poses to update global map.

## Main Contribution

1. Formulate the **Grid map-based SLAM** problem as an optimization problem where the **robot poses and the Grid map are optimized at the same time.**

2. A **variation of the Gauss-Newton** solver is proposed for obtaining global optimal solution and rejecting outliers, without the need of initial accurate guess.

3. The **Data Association** isn’t needed during whole optimization process.
