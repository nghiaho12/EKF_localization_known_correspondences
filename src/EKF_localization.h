#pragma once

#include <Eigen/Dense>

#include "landmark.h"

class EKF_localization
{
public:
    EKF_localization();

    void update(double vel, double yaw_vel, const std::vector<Landmark> &landmarks, double dt);

    void set_state(double x, double y, double yaw);
    double x() { return m_mu(0); }
    double y() { return m_mu(1); }
    double yaw() { return m_mu(2); }

    double ellipse_angle() { return m_ellipse_angle; }
    double ellipse_major() { return m_ellipse_major; }
    double ellipse_minor() { return m_ellipse_minor; }

private:
    void init_ekf();
    void calc_error_ellipse();

private:
    Eigen::Matrix<double, 3, 1> m_mu; // estimated state
    Eigen::Matrix<double, 3, 3> m_cov; // state covarianc

    // noise matrix
    Eigen::Matrix<double, 2, 2> m_landmark_cov;
    Eigen::Matrix<double, 2, 2> m_motion_cov;

    // error ellipse for (x, y);
    double m_ellipse_major;
    double m_ellipse_minor;
    double m_ellipse_angle;
};
