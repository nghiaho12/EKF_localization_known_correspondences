#pragma once

#include <GL/gl.h>
#include <SDL2/SDL.h>
#include <string>
#include <vector>

#include "robot.h"
#include "landmark.h"
#include "EKF_localization.h"

class OpenGLDisplay
{
public:
    void main(std::string title, int width, int height);

private:
    // Display stuff
    void setup_opengl(int width, int height);
    void process_events();
    void display();
    void render_landmarks();
    void render_robot();
    void render_robot(float x, float y, float yaw, float r, float g, float b);

    void init_robot();
    void init_landmarks();

    void update_robot();


private:
    static constexpr float ROBOT_RADIUS = 10;
    static constexpr float ROBOT_HEADING = 20;
    static constexpr float LANDMARK_RADIUS = 5;
    static const int MAX_VEL = 100;
    static const int MAX_YAW_VEL = 60;

    bool m_quit = false;
    int m_width;
    int m_height;

    SDL_Window *m_window = nullptr;
    SDL_GLContext m_context;

    uint32_t m_last_tick = 0;
    int m_yaw_vel_sp = 0;
    int m_vel_sp = 0;

    Robot m_robot;
    std::vector<Landmark> m_landmarks;

    double m_dt = 0;

    EKF_localization m_ekf;
};
