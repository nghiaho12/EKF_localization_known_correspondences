#pragma once

// landmark detection noise parameters
static const double LANDMARK_RANGE_SIGMA = 20.0;
static const double LANDMARL_ANGLE_SIGMA = 2*M_PI/180;

// motion noise parameters
static const double ALPHA1 = 0.005;
static const double ALPHA2 = 0.001;
static const double ALPHA3 = 0.001;
static const double ALPHA4 = 0.005;


// properties of the robot
static const double FOV = 45*M_PI/180;
static const double DETECTION_RANGE = 200;
