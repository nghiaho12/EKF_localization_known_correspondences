#include "opengl_display.h"
#include <iostream>
#include <GL/glu.h>
#include <unistd.h>

#include "config.h"

using namespace std;

void OpenGLDisplay::main(std::string title, int width, int height)
{
    m_width = width;
    m_height = height;

    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        cerr <<  "Video initialization failed: " << SDL_GetError() << endl;
        return;
    }

    m_window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (m_window == NULL) {
        cerr << "Window could not be created! SDL Error: " << SDL_GetError() << endl;
        return;
    }

    //Create context
    m_context = SDL_GL_CreateContext(m_window);
    if (m_context == NULL) {
        cerr << "OpenGL context could not be created! SDL Error: " << SDL_GetError() << endl;
        return;
    }

    setup_opengl(width, height);
    init_robot();
    init_landmarks();

    while (!m_quit) {
        m_dt = (SDL_GetTicks() - m_last_tick) / 1000.0;

        process_events();
        update_robot();
        display();

        m_last_tick = SDL_GetTicks();

        usleep(10000);
    }
}

void OpenGLDisplay::setup_opengl(int width, int height)
{
    // for 2d drawing
    glDepthMask(GL_FALSE);  // disable writes to Z-Buffer
    glDisable(GL_DEPTH_TEST);

    // for font
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // for transparent texture
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void OpenGLDisplay::process_events()
{
    SDL_Event event;

    while (SDL_PollEvent(&event)) {
        float x = event.motion.x;
        float y = m_height - event.motion.y - 1;

        switch (event.type) {
            case SDL_MOUSEBUTTONDOWN: {
                if (event.button.button == SDL_BUTTON_LEFT) {

                }

                break;
            }

            case SDL_KEYDOWN: {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    m_quit = true;
                } else if (event.key.keysym.sym == SDLK_LEFT) {
                    m_yaw_vel_sp += MAX_YAW_VEL;

                    if (m_yaw_vel_sp > MAX_YAW_VEL) {
                        m_yaw_vel_sp = MAX_YAW_VEL;
                    }
                } else if (event.key.keysym.sym == SDLK_RIGHT) {
                    m_yaw_vel_sp -= MAX_YAW_VEL;

                    if (m_yaw_vel_sp < -MAX_YAW_VEL) {
                        m_yaw_vel_sp = -MAX_YAW_VEL;
                    }
                } else if (event.key.keysym.sym == SDLK_UP) {
                    m_vel_sp += MAX_VEL;

                    if (m_vel_sp > MAX_VEL) {
                        m_vel_sp = MAX_VEL;
                    }
                } else if (event.key.keysym.sym == SDLK_DOWN) {
                    m_vel_sp -= MAX_VEL;

                    if (m_vel_sp < -MAX_VEL) {
                        m_vel_sp = -MAX_VEL;
                    }
                } else if (event.key.keysym.sym == SDLK_SPACE) {
                    m_vel_sp = 0;
                    m_yaw_vel_sp = 0;
                }

                break;
            }

            case SDL_KEYUP: {
                m_vel_sp = 0;
                m_yaw_vel_sp = 0;
                break;
            }
        }
    }
}

void OpenGLDisplay::display()
{
    glViewport(0, 0, m_width, m_height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, m_width, 0, m_height);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    render_robot();
    render_landmarks();

    SDL_GL_SwapWindow(m_window);
}

void OpenGLDisplay::init_robot()
{
    m_robot.x(m_width/2);
    m_robot.y(m_height/2);
    m_robot.yaw(M_PI_2);

    m_ekf.set_state(m_robot.x(), m_robot.y(), m_robot.yaw());
}

void OpenGLDisplay::init_landmarks()
{
    Landmark l;

    l.x = 100;
    l.y = 100;
    l.r = 1;
    l.g = 0;
    l.b = 0;

    m_landmarks.push_back(l);

    l.x = m_width - 100;
    l.y = 100;
    l.r = 0;
    l.g = 1;
    l.b = 0;

    m_landmarks.push_back(l);

    l.x = 100;
    l.y = m_height - 100;
    l.r = 0;
    l.g = 1;
    l.b = 1;

    m_landmarks.push_back(l);

    l.x = m_width - 100;
    l.y = m_height - 100;
    l.r = 1;
    l.g = 0;
    l.b = 1;

    m_landmarks.push_back(l);
}

void OpenGLDisplay::render_robot()
{
    glColor3f(0, 1, 0);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_LINE_LOOP);
    for (float a=0; a < 360; a += 10) {
        float x = m_robot.x() + ROBOT_RADIUS*cos(a*M_PI/180);
        float y = m_robot.y() + ROBOT_RADIUS*sin(a*M_PI/180);

        glVertex2f(x, y);
    }
    glEnd();

    // heading arrow
    glBegin(GL_LINES);
    glVertex2f(m_robot.x(), m_robot.y());
    glVertex2f(m_robot.x() + ROBOT_HEADING*cos(m_robot.yaw()), m_robot.y() + ROBOT_HEADING*sin(m_robot.yaw()));
    glEnd();

    // fov cone
    double fov = FOV;
    double range = DETECTION_RANGE;

    double x1 = m_robot.x() + range*cos(m_robot.yaw() - fov/2);
    double y1 = m_robot.y() + range*sin(m_robot.yaw() - fov/2);

    double x2 = m_robot.x() + range*cos(m_robot.yaw() + fov/2);
    double y2 = m_robot.y() + range*sin(m_robot.yaw() + fov/2);

    glBegin(GL_LINES);
    glVertex2f(m_robot.x(), m_robot.y());
    glVertex2f(x1, y1);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(m_robot.x(), m_robot.y());
    glVertex2f(x2, y2);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glEnd();

    glPopMatrix();
}

void OpenGLDisplay::render_landmarks()
{
    glPushMatrix();
    glLoadIdentity();

    for (auto &l : m_landmarks) {
        glColor3f(l.r, l.g, l.b);

        if (m_robot.in_view(l.x, l.y)) {
            glLineWidth(5.0);
        } else {
            glLineWidth(1.0);
        }

        glBegin(GL_LINE_LOOP);
        glVertex2f(l.x - LANDMARK_RADIUS,  l.y - LANDMARK_RADIUS);
        glVertex2f(l.x - LANDMARK_RADIUS,  l.y + LANDMARK_RADIUS);
        glVertex2f(l.x + LANDMARK_RADIUS,  l.y + LANDMARK_RADIUS);
        glVertex2f(l.x + LANDMARK_RADIUS,  l.y - LANDMARK_RADIUS);
        glEnd();
    }

    glLineWidth(1.0);

    glPopMatrix();
}

void OpenGLDisplay::update_robot()
{
    m_robot.set_velocity(m_vel_sp);
    m_robot.set_yaw_velocity(m_yaw_vel_sp * M_PI / 180);
    m_robot.update(m_dt);

    if (m_robot.x() < 0) {
        m_robot.x(0);
        m_robot.set_velocity(0);
    }

    if (m_robot.y() < 0) {
        m_robot.y(0);
        m_robot.set_velocity(0);
    }

    if (m_robot.x() >= m_width) {
        m_robot.x(m_width - 1);
        m_robot.set_velocity(0);
    }

    if (m_robot.y() >= m_height) {
        m_robot.y(m_height - 1);
        m_robot.set_velocity(0);
    }

    if (m_robot.vel()  || m_robot.yaw_vel()) {
        m_ekf.update(m_robot.vel_noisy(), m_robot.yaw_noisy(), m_landmarks, m_dt);
    }
}

