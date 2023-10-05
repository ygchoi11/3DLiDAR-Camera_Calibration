# 3DLiDAR-Camera_Calibration

## Introduction
This is an example for testing an offline 3D LiDAR-camera extrinsic calibration, based on an orthogonal trihedral target with a checkered pattern on each plane.
It is implemented using OpenCV 4.6.0. __Please refer to OpenCV installation and settings [here](https://opencv.org/).__ <br/>
The synthetic data was generated using Blensor. The Blensor is a free open-source simulation package for LIDAR/LADAR and Kinect sensors.
__The simulation is available [here](https://www.blensor.org/)__. <br/>

## Examples
There are four examples in `tester.h`: <br/>
`testBlensor()`, `findCameraIntrinsicInReal()`, `calibrateInReal()`, `calibrateInReal2()`.

### Test on synthetic data
In `testBlensor()`, the accuracy of the estimated pose is evaluated by comparing it to the ground-truth pose. <br/> 
The test results for "pointcloud_noisy_positional_0.07.pcd" are below. <br/>
1. __Pose estimation and error results__ <br/>
<img src="results/01.blensor_result.png" width = "80%" /> <br/>
1. __Input image and point cloud__ <br/>
<img src="results/02-1.blensor_image.png" width = "80%" /> <br/>
<img src="results/02-2.blensor_pointcloud.png" width = "80%" /> <br/>
1. __Relative pose accuracy verification__ <br/>
Each corresponding color of the point cloud is obtained from the image and represented as 3D points in the LiDAR coordinates. <br/>
<img src="results/03.blensor_colorization.png" width = "80%" /> <br/>
The point cloud is projected from LiDAR coordinates to image coordinates and overlaid with the image.
3D points on the target are classified into three planes by the estimated LiDAR pose.
It is represented in RGB colors. <br/>
<img src="results/04.blensor_check_relative_pose_l2c.png" width = "80%" /> <br/>

### Camera calibration
`findCameraIntrinsicInReal()` is an example of camera calibration. <br/>
1. __Camera intrinsic and distortion parameters__ <br/>
<img src="results/05.realworld_intrinsic.png" width = "80%" /> <br/>

### Test on Real-world data
In `calibrateInReal()`, the 3D LiDAR-camera extrinsic calibration is tested in a real environment. <br/> 
1. __The estimated relative pose (LiDAR-to-Camera)__ <br/>
<img src="results/06.realworld_result.png" width = "80%" /> <br/>
1. __Input image and point cloud__ <br/>
<img src="results/07-1.realworld_image.png" width = "80%" /> <br/>
<img src="results/07-2.realworld_pointcloud.png" width = "80%" /> <br/>
1. __Relative pose verification__ <br/>
Each corresponding color of the point cloud is obtained from the image and represented as 3D points in the LiDAR coordinates. <br/>
<img src="results/08.realworld_colorization.png" width = "80%" /> <br/>
The point cloud is projected from LiDAR coordinates to image coordinates and overlaid with the image.
3D points on the target are classified into three planes by the estimated LiDAR pose.
It is represented in RGB colors. <br/>
<img src="results/09-1.realworld_check_relative_pose_l2c.png" width = "80%" /> <br/>
In the image below, the point cloud is colored according to distance. <br/>
<img src="results/09-2.realworld_check_relative_pose_l2c.png" width = "80%" /> <br/>
1. __Colorization problem__ <br/>
As the point cloud is rotated by relative pose, some 3D points overlap in the image coordinates.
The blue points in the image on the left below actually represent a greater distance than the green points.
In other words, the blue points should actually be colored with the color of the wall behind the box,
but since the color behind the box cannot be seen in the image, it is colored with the color of the box.
This problem arises from the colorization algorithm. <br/>
<img src="results/colorization_problem.png" width = "80%" /> <br/>

### Manual selection of relative pose
Our calibration method automatically selects the relative pose with the smallest rotation angle among the three possibilities.
For LiDAR-camera integrations with a large relative rotation angle, manual selection is required. <br/>
For example, `calibrateInReal2()` assumes the camera is mounted upside down. In other words, the input image is rotated 180 degrees around the camera's principal axis. <br/>
1. __Input image__ <br/>
<img src="results/10.realworld_image_ud.png" width = "80%" /> <br/>
1. __The relative pose with the smallest rotation angle__ <br/>
The coordinate axes are correctly aligned with the target, but taking the image and point cloud into account, we can see that this result is incorrect. <br/>
<img src="results/11-1.realworld_ud_check_relative_pose_l2c_1.png" width = "80%" /> <br/>
<img src="results/11-2.realworld_ud_check_relative_pose_l2c_1.png" width = "80%" /> <br/>
1. __Second relative pose for manual selection__ <br/>
<img src="results/11-3.realworld_ud_check_relative_pose_l2c_2.png" width = "80%" /> <br/>
<img src="results/11-4.realworld_ud_check_relative_pose_l2c_2.png" width = "80%" /> <br/>
1. __Third relative pose for manual selection__ <br/>
<img src="results/11-5.realworld_ud_check_relative_pose_l2c_3.png" width = "80%" /> <br/>
<img src="results/11-6.realworld_ud_check_relative_pose_l2c_3.png" width = "80%" /> <br/>
1. __Conclusion__ <br/>
Checkered patterns of three different sizes enable the camera to identify the three planes.
Using a target with different plane sizes allows LiDAR to identify the three planes.
However, these conditions require the sensor to fully observe the target.
We think that this preparation made the use of the algorithm complex and cumbersome. <br/>
In our method, if possible, we recommend choosing a sufficiently large target and placing it close to the sensors.
It doesn’t matter if the target is not fully observed in LiDAR’s FoV.
Observation of many points leads to more accurate results. <br/>
The result below shows that the algorithm works even though only some points on the target are used. <br/>
<img src="results/12-1.realworld_pt_check_relative_pose_l2c.png" width = "80%" /> <br/>
<img src="results/12-2.realworld_pt_check_relative_pose_l2c.png" width = "80%" /> <br/>

