# 3D Object Detection Project

In this project, we use the Waymo Open Dataset to explore 3D lidar sensor data. The sensor data is used to get hands-on experience on 3D object detection algorithms. Basically, 3D object detection is derived from the image-based approach. The trick is to transform the three-dimensional information to a two-dimensional
bird view (BEV). The lidar image can then be represented by three channels: the height, the intensity and the density. Similarly to the image color spaces, we can now apply the image object detection algorithm on the lidar data. The project guides us through the several steps of the data processing.

## Range Image
The first task was to extract the range image from the Waymo recording. From the range image, we were required to convert the channels "range" and "intensity" from floating-point data to an 8-bit integer value range. Additionally, we also had to handle outliers within the data. Otherwise, the outliers would lead to a distorted image making the analysis unfeasible. The image below depicts both the range (top) and intensity (bottom) image. The view is cropped to the yaw angle range of [-90, +90] degrees.

![img][doc/mid07.png]

## Point cloud
In this task, we had to display the lidar point cloud using the Open3D library. In addition, we were supposed to inspect the vehicles.


![img][doc/mid01.png]![img][doc/mid02.png]![img][doc/mid03.png]![img][doc/mid04.png]![img][doc/mid05.png]![img][doc/mid06.png]



