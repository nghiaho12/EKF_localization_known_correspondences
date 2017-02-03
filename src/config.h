#pragma once

// landmark detection noise parameters
static const double LANDMARK_RANGE_SIGMA = 20.0;
static const double LANDMARK_ANGLE_SIGMA = 2*M_PI/180;

// motion noise parameters
static const double ALPHA1 = 2;
static const double ALPHA2 = 0.1;
static const double ALPHA3 = 0.0;
static const double ALPHA4 = 1;

// properties of the robot
static const double FOV = 45*M_PI/180;
static const double DETECTION_RANGE = 200;

static const double ELLIPSE_CHI = 2.4477;
