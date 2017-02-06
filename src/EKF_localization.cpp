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
    m_landmark_cov(0, 0) = LANDMARK_RANGE_SIGMA*LANDMARK_RANGE_SIGMA;
    m_landmark_cov(1, 1) = LANDMARK_ANGLE_SIGMA*LANDMARK_ANGLE_SIGMA;

    cout << "m_landmark_cov " << m_landmark_cov << endl;

    m_cov.setZero();
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

    // correction step
    Matrix<double, 3, 1> ZHAT;
    Matrix<double, 3, 1> Z;
    Matrix<double, 3, 3> H;
    Matrix<double, 3, 3> S;
    Matrix<double, 3, 3> Q;
    Matrix<double, 3, 3> K;
    Matrix<double, 3, 3> I;

    G.setIdentity();
    M.setZero();
    V.setZero();
    Q.setZero();
    H.setZero();
    I.setIdentity();

    double theta = yaw();
    double w = yaw_vel;

    // noise
    M(0, 0) = ALPHA1*vel*vel + ALPHA2*w*w;
    M(1, 1) = ALPHA3*vel*vel + ALPHA4*w*w;

    V(2, 1) = dt;

    // apply motion model
    if (fabs(yaw_vel) > EPS) {
        double r = vel/yaw_vel;
        double c = cos(theta);
        double s = sin(theta);
        double t = theta + w*dt;

        // Jacobian
        G(0, 2) = r*(-c + cos(t));
        G(1, 2) = r*(-s + sin(t));

        V(0, 0) = (-s + sin(t))/w;
        V(0, 1) =  vel*(s - sin(t))/(w*w) + r*cos(t)*dt;
        V(1, 0) = (c - cos(t))/w;
        V(1, 1) = -vel*(c - cos(t))/(w*w) + r*sin(t)*dt;
        V(2, 0) = 0;

        // Prediction
        MU(0) = m_mu(0) - r*s + r*sin(t);
        MU(1) = m_mu(1) + r*c - r*cos(t);
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

    for (const auto &l : landmarks) {
        Z(0) = l.range;
        Z(1) = l.theta;
        Z(2) = 0;

        double range, bearing;
        Robot::landmark_range_bearing(l, MU(0), MU(1), MU(2), range, bearing);

        ZHAT(0) = range;
        ZHAT(1) = bearing;
        ZHAT(2) = 0;

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

    m_mu = MU;

    m_cov = COV;

    calc_error_ellipse();
}

void EKF_localization::calc_error_ellipse()
{
    SelfAdjointEigenSolver<Matrix2d> eigensolver(m_cov.block<2, 2>(0, 0));
/*
   cout << "The eigenvalues of A are:\n" << eigensolver.eigenvalues() << endl;
   cout << "Here's a matrix whose columns are eigenvectors of A \n"
        << "corresponding to these eigenvalues:\n"
        << eigensolver.eigenvectors() << endl;
*/
    if (eigensolver.eigenvalues()(0) > eigensolver.eigenvalues()(1)) {
        m_ellipse_angle = atan2(eigensolver.eigenvectors()(0,1), eigensolver.eigenvectors()(0,0));
        m_ellipse_major = sqrt(eigensolver.eigenvalues()(0));
        m_ellipse_minor = sqrt(eigensolver.eigenvalues()(1));
    } else {
        m_ellipse_angle = atan2(eigensolver.eigenvectors()(1,1), eigensolver.eigenvectors()(1,0));
        m_ellipse_major = sqrt(eigensolver.eigenvalues()(1));
        m_ellipse_minor = sqrt(eigensolver.eigenvalues()(0));
    }

    //cout << "angle: " << m_ellipse_angle * 180 / M_PI << endl;
    //cout << "major/minor: " << m_ellipse_major << " " << m_ellipse_minor << endl;
}
