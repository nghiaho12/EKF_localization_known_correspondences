#include "EKF_localization.h"

#include <iostream>

#include "config.h"
#include "robot.h"

using namespace std;
using namespace Eigen;

EKF_localization::EKF_localization()
{
    init_ekf();
}

void EKF_localization::init_ekf()
{
    m_landmark_cov.setZero();
    m_landmark_cov(0, 0) = pow(LANDMARK_RANGE_SIGMA, 2);
    m_landmark_cov(1, 1) = pow(LANDMARK_ANGLE_SIGMA, 2);

    m_cov.setZero();
}

void EKF_localization::set_state(double x, double y, double yaw)
{
    m_mu(0) = x;
    m_mu(1) = y;
    m_mu(2) = yaw;
}

void EKF_localization::update(double v, double w, const std::vector<Landmark> &landmarks, double dt)
{
    // Implementation based on
    // http://probabilistic-robotics.informatik.uni-freiburg.de/corrections1/pg204-206.pdf
    // I've kept variables name to be the same where possible.

    // I added special handling for w ~ 0. The original implementation does not handle this.
    const double EPS = 0.0001;

    Matrix<double, 3, 3> G;
    Matrix<double, 3, 2> V;
    Matrix<double, 2, 2> M;
    Matrix<double, 3, 1> MU;
    Matrix<double, 3, 3> COV;

    // correction step
    Matrix<double, 3, 1> ZHAT;
    Matrix<double, 3, 1> Z;
    Matrix<double, 3, 3> H;
    Matrix<double, 3, 3> S;
    Matrix<double, 3, 3> Q;
    Matrix<double, 3, 3> K;
    Matrix<double, 3, 3> I;

    G.setIdentity();
    V.setZero();
    M.setZero();
    Q.setZero();
    H.setZero();
    I.setIdentity();

    m_dt = dt;

    double theta = yaw(); // previous yaw

    // noise
    M(0, 0) = ALPHA1*v*v + ALPHA2*w*w;
    M(1, 1) = ALPHA3*v*v + ALPHA4*w*w;

    V(2, 0) = 0;
    V(2, 1) = dt;

    // check if angular velocity is close to zero
    if (fabs(w) > EPS) {
        // Jacobian
        G(0, 2) = -(v/w)*cos(theta) + (v/w)*cos(theta + w*dt);
        G(1, 2) = -(v/w)*sin(theta) + (v/w)*sin(theta + w*dt);

        V(0, 0) = (-sin(theta) + sin(theta + w*dt))/w;
        V(1, 0) = ( cos(theta) - cos(theta + w*dt))/w;
        V(0, 1) =  v*(sin(theta) - sin(theta + w*dt))/(w*w) + v*cos(theta + w*dt)*dt/w;
        V(1, 1) = -v*(cos(theta) - cos(theta + w*dt))/(w*w) + v*sin(theta + w*dt)*dt/w;

        // Prediction
        MU(0) = m_mu(0) - (v/w)*sin(theta) + (v/w)*sin(theta + w*dt);
        MU(1) = m_mu(1) + (v/w)*cos(theta) - (v/w)*cos(theta + w*dt);
        MU(2) = m_mu(2) + w*dt;

        COV = G*m_cov*G.transpose() + V*M*V.transpose();
    } else {
        // Handle case when w ~ 0
        // Use L'Hopital rule with lim w -> 0
        G(0, 2) = -v*sin(theta)*dt;
        G(1, 2) =  v*cos(theta)*dt;

        V(0, 0) = cos(theta)*dt;
        V(1, 0) = sin(theta)*dt;
        V(0, 1) = -v*sin(theta)*dt*dt*0.5;
        V(1, 1) =  v*cos(theta)*dt*dt*0.5;

        MU(0) = m_mu(0) + v*cos(theta)*dt;
        MU(1) = m_mu(1) + v*sin(theta)*dt;
        MU(2) = m_mu(2);

        COV = G*m_cov*G.transpose() + V*M*V.transpose();
    }

    for (const auto &l : landmarks) {
        Z(0) = l.range;
        Z(1) = l.bearing;
        Z(2) = 0; // landmark signature, not used

        if (fabs(l.range) > EPS) {
            double range, bearing;
            Robot::landmark_range_bearing(l, MU(0), MU(1), MU(2), range, bearing);

            ZHAT(0) = range;
            ZHAT(1) = bearing;
            ZHAT(2) = 0; // landmark signature, not used

            H(0, 0) = -(l.x - MU(0))/range;
            H(0, 1) = -(l.y - MU(1))/range;
            H(0, 2) = 0;
            H(1, 0) =  (l.y - MU(1))/(range*range);
            H(1, 1) = -(l.x - MU(0))/(range*range);
            H(1, 2) =  -1;

            Q(0, 0) = pow(l.range*DETECTION_RANGE_ALPHA, 2);
            Q(1, 1) = pow(DETECTION_ANGLE_SIGMA, 2);
            Q(2, 2) = 1; // arbitrary noise for signature

            S = H*COV*H.transpose() + Q;

            K = COV*H.transpose()*S.inverse();
            MU = MU + K*(Z - ZHAT);
            COV = (I - K*H)*COV;
        }
    }

    m_mu = MU;
    m_cov = COV;

    // cap yaw to [-pi, pi]
    if (m_mu(2) > M_PI) {
        m_mu(2) -= 2*M_PI;
    } else if (m_mu(2) < -M_PI) {
        m_mu(2) += 2*M_PI;
    }

    calc_error_ellipse();
}

void EKF_localization::calc_error_ellipse()
{
    SelfAdjointEigenSolver<Matrix2d> a(m_cov.block<2, 2>(0, 0));

    double e0 = sqrt(a.eigenvalues()(0));
    double e1 = sqrt(a.eigenvalues()(1));

    if (e0 > e1) {
        m_ellipse_angle = atan2(a.eigenvectors()(1,0), a.eigenvectors()(0,0));
        m_ellipse_major = e0;
        m_ellipse_minor = e1;
    } else {
        m_ellipse_angle = atan2(a.eigenvectors()(1,1), a.eigenvectors()(0,1));
        m_ellipse_major = e1;
        m_ellipse_minor = e0;
    }
}
