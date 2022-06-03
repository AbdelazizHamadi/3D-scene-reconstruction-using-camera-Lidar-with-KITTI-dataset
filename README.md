# 3D scene reconstruction using camera and Lidar with KITTI dataset
 This work gives a clear view about 3D reconstruction pipeline from start to finish with KITTI dataset

 ## Project Description
 In this project we work with a sample of KITTI dataset (50 frames) where we are going to reconstruct the 3D scene with the material provided by the dataset (cameras, Lidar).
## KITTI Dataset Blue Print

 ![](Results/setup_top_view.png)

## KITTI Dataset's camera's view
As we can see here the car is in a straight forward movement  

![](Results/KITTI_POV_camera2.gif)

 ### Project pipeline   
<ol>
<li> Load dataset with pykitti library </li>
<li> Get all the position matrices for each frame </li>
<li> Get Lidar poses for each frame </li>
<li> Extract calibration matrix that transforms from IMU coordinates to Lidar coordinates </li>
<li> Remove points that have depth close to the camera (<5) </li>
<li> Extract projection matrix that transforms from 3D world coordinates to camera coordinates (second camera of KITTI)</li>
<li> Normalizing the fourth element </li>
<li> Extract second camera's calibration </li>
<li> Now after getting all the information needed we run a loop through frames:</li>
<ol>
<li> Get the captured new points in Lidar coordinates </li>
<li> Calculate 2D points, Projected points in camera coordinates</li>
<li> Normalizing by the third value </li>
<li> Get image info (RGB values, aka images in the KITTI dataset) </li>
<li> Create mask that filters the points by the dimension of the image, to remove the ones that falls outside the frame </li>
<li> Create mask that filters the points by the dimension of the image, to remove the ones that falls outside the frame </li>
**Note** : Our new 2D points coords that are projected to the camera are now the pixels' position </li>
<li> get the RGB value of each point </li>
<li> Concatenate all info in one array (x, y, z, r, g, b) </li>
</ol>
<li> Draw all point with plyfile library (to open in MeshLab)</li>
</ol>



 ## Results


![](Results/results.gif)
