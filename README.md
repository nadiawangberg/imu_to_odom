IMU to odometry
===============


This repository is heavily based on eth's odom predictor package https://github.com/ethz-asl/odom_predictor. It is 
a simple ros node that integrates IMU data to estimate an odometry and a tf transform. 

### Disclaimer
IMU should never be used alone to estimate an odometry (as done in this package). The reason for this is that slightly noisy IMU data will cause the position to drift A LOT over time. The purpose of this node is visualing the IMU data. It can also be used as a part of a Kalman filter or something similar. 
