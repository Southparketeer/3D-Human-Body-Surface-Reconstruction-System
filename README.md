# 3D Human Body Surface Reconstruction System Front-end for Large Scale Data Collection
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

## Demo
