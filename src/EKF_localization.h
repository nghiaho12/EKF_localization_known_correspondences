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

private:
    void init_ekf();

private:
    Eigen::Matrix<double, 3, 1> m_mu; // estimated state
    Eigen::Matrix<double, 3, 3> m_cov; // state covarianc

    // noise matrix
    Eigen::Matrix<double, 2, 2> m_landmark_cov;
    Eigen::Matrix<double, 2, 2> m_motion_cov;
};
