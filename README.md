This is an implementation of the EKF localizationn with known correspondences from the book Probabilistic Robotics by Thrun et. al. I wrote this code to help me understand the material better. So others may find it useful as well.

One addition I made was handling the case when angular velocity is close to zero. The original code found in the book would result in a divide by zero.

###Requirements
This demo code requires the following libraries
- SDL2
- Eigen

I've only tested this on a Linux machine but it should on Mac and Windows because both libraries are cross platform.

###Compiling
$ mkdir build
$ cd build
$ cmake ..
$ make
