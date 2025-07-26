// Copyright 2021 DeepMind Technologies Limited
// Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.

#include "renderer.h"
#include <iostream>
#include <cstring>

Renderer::Renderer() = default;

Renderer::~Renderer()
{
    if (window)
        glfwDestroyWindow(window);

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(d);
    mj_deleteModel(m);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

// load and compile model
bool Renderer::load(const std::string &path)
{
    char error[1000];

    if (path.size() > 4 && path.substr(path.size() - 4) == ".mjb")
    {
        m = mj_loadModel(path.c_str(), nullptr);
    }
    else
    {
        m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    }

    if (!m)
    {
        std::cerr << "Failed to load model: " << error << std::endl;
        return false;
    }

    // make data
    d = mj_makeData(m);

    // init GLFW
    if (!glfwInit())
    {
        std::cerr << "Could not initialize GLFW\n";
        return false;
    }

    // create window, make OpenGL context current, request v-sync
    window = glfwCreateWindow(1200, 900, "MuJoCo Viewer", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Could not create GLFW window\n";
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // store pointer to this instance for callback access
    glfwSetWindowUserPointer(window, this);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    return true;
}

void Renderer::run()
{
    while (!glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0)
        {
            mj_step(m, d);
        }

        handle_frame();
    }
}

void Renderer::handle_frame()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

// keyboard callback
void Renderer::keyboard(GLFWwindow *window, int key, int, int act, int)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
        auto *self = static_cast<Renderer *>(glfwGetWindowUserPointer(window));
        mj_resetData(self->m, self->d);
        mj_forward(self->m, self->d);
    }
}

// mouse button callback
void Renderer::mouse_button(GLFWwindow *window, int, int, int)
{
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void Renderer::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
    {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left)
    {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
        action = mjMOUSE_ZOOM;
    }

    auto *self = static_cast<Renderer *>(glfwGetWindowUserPointer(window));
    // move camera
    mjv_moveCamera(self->m, action, dx / height, dy / height, &self->scn, &self->cam);
}

// scroll callback
void Renderer::scroll(GLFWwindow *window, double, double yoffset)
{
    auto *self = static_cast<Renderer *>(glfwGetWindowUserPointer(window));
  // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(self->m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &self->scn, &self->cam);
}
