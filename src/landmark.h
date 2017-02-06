#pragma once

struct Landmark
{
    float x;
    float y;

    float r, g, b;

    // Relative to robot
    double range;
    double theta;
};
