# PnL-Ioc

**Indoor Camera Pose Estimation from Room Layouts and Image Outer Corners**
<br>

## Introduction

A new PnL-IOC algorithm is proposed which has two implementations according to the room layout types. The first one considers
six layouts with more than two boundary lines where 3D correspondenceestimation of IOCs creates sufficient line correspondences for
camera pose estimation. The second one is an extended version to handle two challenging layouts with only two coplanar boundaries where 
correspondence estimation of IOCs is ill-posed due to insufficient conditions. Thus we develop a variant of PnL-IOC, called the
Coplanar P3L (CP3L) method which is embedded with the powerful NSGA-II algorithm to estimate the correspondences of IOCs. At the last step,
the camera pose is jointly optimized with 3D correspondence refinement of IOCs in the iterative Gauss-Newton algorithm.
Experiment results on both simulated and real images show the advantages of PnL-IOC including CP3L on the accuracy and robustness of
camera pose estimation from eight different room layouts over the existing PnL methods.


