#include "EKF_localization.h"
#include "config.h"
#include <iostream>

using namespace std;
using namespace Eigen;

EKF_localization::EKF_localization()
{
    init_ekf();
}

void EKF_localization::init_ekf()
{
    m_landmark_cov.setZero();
    m_landmark_cov(0, 0) = LANDMARK_RANGE_SIGMA*LANDMARK_RANGE_SIGMA;
    m_landmark_cov(1, 1) = LANDMARL_ANGLE_SIGMA*LANDMARL_ANGLE_SIGMA;

    cout << "m_landmark_cov " << m_landmark_cov << endl;

    m_cov.setIdentity();
}

void EKF_localization::set_state(double x, double y, double yaw)
{
    m_mu(0) = x;
    m_mu(1) = y;
    m_mu(2) = yaw;
}

void EKF_localization::update(double vel, double yaw_vel, const std::vector<Landmark> &landmarks, double dt)
{
    const double EPS = 0.0001;

    Matrix<double, 3, 3> G;
    Matrix<double, 3, 2> V;
    Matrix<double, 2, 2> M;
    Matrix<double, 3, 1> MU;
    Matrix<double, 3, 3> COV;

    G.setIdentity();
    M.setZero();
    V.setZero();

    double theta = yaw();
    double w = yaw_vel;

    // Simplified
    M(0, 0) = ALPHA1*vel*vel + ALPHA2*w*w;
    M(1, 1) = ALPHA3*vel*vel + ALPHA3*w*w;

    double r = vel/yaw_vel;

    V(2, 1) = dt;

    if (fabs(yaw_vel) > EPS) {
        // Jacobian
        G(0, 2) = -r*cos(theta) + r*cos(theta + w*dt);
        G(1, 2) = -r*sin(theta) + r*sin(theta + w*dt);

        V(0, 0) = (-sin(theta) + sin(theta + w*dt)) / w;
        V(0, 1) = vel*(sin(theta) - sin(theta + w*dt))/(w*w) + (vel*cos(theta + w*dt)*dt)/w;
        V(1, 0) = (cos(theta) - cos(theta + w*dt))/w;
        V(1, 1) = -vel*(cos(theta) - cos(theta + w*dt))/(w*w) + (vel*sin(theta + w*dt)*dt)/w;
        V(2, 0) = 0;

        // Prediction
        MU(0) = m_mu(0) - r*sin(theta) + r*sin(theta + w*dt);
        MU(1) = m_mu(1) + r*cos(theta) - r*cos(theta + w*dt);
        MU(2) = m_mu(2) + w*dt;

        COV = G*m_cov*G.transpose() + V*M*V.transpose();
    } else {
        // TODO: Handle degenerate case when w = 0;
        // Use L'Hopital rule for lim w -> 0
        G(0, 2) = -vel*sin(theta)*dt;
        G(1, 2) = -vel*cos(theta)*dt;

        V(0, 0) = cos(theta)*dt;
        V(0, 1) = vel*sin(theta)*dt*dt/2 - vel*sin(theta)*dt*dt;
        V(1, 0) = sin(theta)*dt;
        V(1, 1) = -vel*cos(theta)*dt*dt/2 + vel*cos(theta)*dt*dt;
        V(2, 0) = 0;

        MU(0) = m_mu(0) + vel*cos(theta)*dt;
        MU(1) = m_mu(1) + vel*sin(theta)*dt;
        MU(2) = m_mu(2);

        COV = m_cov + V*M*V.transpose();
    }

    m_mu = MU;
    m_cov = COV;

    cout << endl;
    cout << m_cov << endl;
}
