# LiDAR Cone Detector
This pipeline is used to detect cones in a PointCloud2 message, a data structure obtained from a LiDAR sensor.
# Motivation
In the Formula SAE Student Competition, the track is delimited by cones of two different colors: yellow and blue. While it is possible to detect the cones' positions using cameras, LiDAR provides significantly higher precision and reliability. Therefore, we use LiDAR data to detect the cones that define the track boundaries.


