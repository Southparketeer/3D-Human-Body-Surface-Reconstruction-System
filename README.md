# 3D Human Body Surface Reconstruction System
User interface for capture. Take depth and RGB streams from two RGBD sensors. Generate 3D partial meshes for different capture poses. 

## Data Acquisition
### System Setup
Our capture system consists of two Kinect v2 sensors, one 28" twin camera slide bar, and tripod. The sensors are vertically mounted on the two ends of the slide bar to reach an optimal accuracy and capture volume. The PCIe parallel USB is installed on a PC for real time data acquisition by multiple sensors. The figure below shows our capture system.

<p align="center">
   <img width="600" src= demo/Capture_System.PNG>
</p>

### Capture
During the scan, the subject stands upright at approximately 125cm from the sensors holding an “A pose”, i.e. arms open roughly 45 degrees and feet open roughly 45cm. The data acquisition takes eight scans (each representing a pose) in total with the user rotating roughly 45 degrees between each scan and holding the pose for approximately 1 second. 30 frames of depth image and 1 frame of color image are collected for each scan. 

## Processing Pipeline
<p align="center">
   <img width="600" src= demo/Pipeline_Overview.png>
</p>
### System Front-end 
In the front-end, our hardware design aims to maximize the data accuracy by exploring the sensor noise pattern. We analyze sensor noise characteristics through the system calibration, which includes an experiment to measure and model the sensor depth bias as a function of distance and a standard sensor intrinsic and extrinsic calibration. Accordingly, we set up our system to achieve a good sensor performance and the depth bias is corrected in real-time during capture. The depth and color images are the outputs of our front-end, which will be processed in the back-end for 3D reconstruction. 
### System Back-end 
In the back-end, we propose a non-rigid registration framework that is appropriate for our semi-nonrigid pose assumption (i.e. various human body poses appear as high degree of deformation around skeletal joints while modest deformations appear around link segments). Partial scan meshes are reconstructed during mesh preprocessing. Then, skeletal joint positions are inferred through multimodality registration. With partial scan meshes and skeleton data, the 3D human body surface is reconstructed through our non-rigid registration framework.

## Mesh Preprocessing
Eight partial scan meshes are generated in this step corresponding to the eight poses. To generate the partial scan mesh efficiently and accurately, we first reconstruct low resolution (8mm) meshes from the two cameras to get an optimal extrinsic transformation. Then, we generate a high resolution (2.6mm) mesh by fusing two cameras’ depth images with the calculated transformation matrix. Partial meshes are generated using depth images from both sensors. An adaptive size Truncated Signed Distance Function (TSDF) volume (i.e. the size of volume fits the bounding box of scanned subject) is employed to optimize the memory and computation efficiency as well as to average out some of the sensor noise. The partial meshes are extracted from the TSDF volume using Marching Cubes. One extra scan without a subject is taken to estimate the ground plane parameters for ground clipping during preprocessing. 

## Demo
### System Input

### System Output
