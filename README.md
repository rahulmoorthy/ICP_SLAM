# ICP_SLAM

The project implementation mainly consists of Key frame creation, ICP Registration and ICP Iteration.ICP Track function is called in icp_slam_node.cpp and the method is defined in icp_slam.cpp

Key frames are created based on any of the following:
  1) If the L2 distance between current and last keyframes is greater than the defined value.
  2) If difference in rotation between last and current keyframes are greater than the defined value.
  3) If the time difference is greater than the defined value.
In case of the first key frame, we set the last key frame scan to the current scan.
If any of the above conditions are satisfied then the following was implemented:

### Calculate Current Pose:

1) Calculated the current pose based on the wheel encoder values. For the initial keyframe, the last and the current keyframes are same but for every successive iteration, the   pose difference is based on the last keyframe and the current keyframe poses. This value is passed to ICP Registration.
2) After ICP Registration has returned the refined pose, we convert that transform(which is wrt to last key frame) back to with respect to the odom then change the frame id to     /odom and child frame id to /map. Then this transform is used to update the map and broadcasting using tf_broadcaster_.sendTransform.

### ICP Registration:

1) Computed the current and last scan matrices using laserScanToPointMat function. The written function converts the laser scan data into point coordinates and returns a n*2        matrix.
2) The transformPointmat() method converts the current scan with respect to transform between the current and last scans. This is now the updated current scan. This is only used    to compute errors.
3) ICP Iteration is then called in a loop till the transforms converge(the error stops changing) or sufficient iterations are achieved.

### ICP Iteration:

1) In ICP Iteration, we call closestPoints() by passing the last scan and updated current scan matrices. This method would return closest distances and closest indices. Mean and    Standard Deviation values are then computed on the closest distances using the given method meanAndStdDev().
2) We then reordered the current scan matrix according to the closest indices vector. For every value in closest indices,
   the corresponding index in current scan was taken and the coordinate values were appended into a new matrix, so each
   index in both last and reordered matrix refers to a match.
3) Based on the threshold condition (Mean + 2* Std), the matrices were trimmed from last and current scans. The two
   matrices received are the trimmed matrices (x and p).
4) Next, centre of mass was computed against x and p matrices. Also the corresponding coordinate points were subtracted from the centre of mass matrix(2 *1).

5) W was then computed by taking the transpose of x and multiplying it with p. This results in a W matrix of size 2*2. We then performed Singular Value Decomposition (SVD) to      compute rotation and translation (R and t).
6) The R and t are then transformed to obtained a refined pose which is then returned. For every iteration, error is then calculated by L2 distance between x and p’(obtained        from refined pose).
