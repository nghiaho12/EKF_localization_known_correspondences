#include "robot.h"
#include <cmath>
#include <random>

#include "config.h"

using namespace std;

static random_device g_r;

void Robot::update(double dt)
{
    m_yaw = m_yaw + m_yaw_vel*dt;

    m_x = m_x + m_vel*cos(m_yaw)*dt;
    m_y = m_y + m_vel*sin(m_yaw)*dt;
}

bool Robot::in_view(double x, double y)
{
    double dx = x - m_x;
    double dy = y - m_y;
    double dist = sqrt(dx*dx + dy*dy);

    if (dist < DETECTION_RANGE) {
        dx /= dist;
        dy /= dist;

        double heading_x = cos(m_yaw);
        double heading_y = sin(m_yaw);

        double dot = dx*heading_x + dy*heading_y;

        double theta = acos(dot);

        if (theta < FOV*0.5) {
            return true;
        }
    }

    return false;
}

double Robot::vel_noisy()
{
    default_random_engine e(g_r());

    std::normal_distribution<double> dice(0, 1);

    return m_vel + m_vel*ALPHA1*dice(e);
}

double Robot::yaw_noisy()
{
    default_random_engine e(g_r());

    std::normal_distribution<double> dice(0, 1);

    return m_yaw + m_yaw_vel*ALPHA3*dice(e);
}
