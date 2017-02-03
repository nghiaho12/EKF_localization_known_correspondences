#pragma once

#include <cmath>

class Robot
{
public:
    // camera
    bool in_view(double x, double y);

    // controls
    void set_velocity(double vel) { m_vel = vel; }
    void set_yaw_velocity(double yaw_vel) { m_yaw_vel = yaw_vel; }
    void update(double dt);

    // setter
    void x(double _x) { m_x = _x; }
    void y(double _y) { m_y = _y; }
    void yaw(double _yaw) { m_yaw = _yaw; }

    // getter
    double x() { return m_x; }
    double y() { return m_y; }
    double yaw() { return m_yaw; }

    double vel() { return m_vel; }
    double yaw_vel() { return m_yaw_vel; }

    // noisy version
    double vel_noisy();
    double yaw_noisy();

private:
    // robot state
    double m_x = 0;
    double m_y = 0;
    double m_vel = 0;
    double m_yaw = 0;
    double m_yaw_vel = 0;
};
