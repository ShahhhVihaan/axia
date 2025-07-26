// Copyright 2021 DeepMind Technologies Limited
// Licensed under the Apache License, Version 2.0. See LICENSE file in the project root for details.

#pragma once

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <string>

class Renderer
{
public:
    Renderer();
    ~Renderer();

    bool load(const std::string &path);
    void run();

private:
    // callbacks
    static void keyboard(GLFWwindow *window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    void handle_frame();

    // MuJoCo data structures
    mjModel *m = NULL; // MuJoCo model
    mjData *d = NULL;  // MuJoCo data
    mjvCamera cam;     // abstract camera
    mjvOption opt;     // visualization options
    mjvScene scn;      // abstract scene
    mjrContext con;    // custom GPU context

    GLFWwindow *window = nullptr;

    // mouse interaction
    static inline bool button_left = false;
    static inline bool button_middle = false;
    static inline bool button_right = false;
    static inline double lastx = 0, lasty = 0;
};
