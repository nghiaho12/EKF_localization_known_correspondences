#pragma once

struct Landmark
{
    float x;
    float y;
    float r, g, b; // RGB color

    // Sensor reading relative to robot's pose
    double range;
    double bearing;
};
