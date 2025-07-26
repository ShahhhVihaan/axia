#pragma once

#include <mujoco/mujoco.h>
#include <string>

class Simulation {
public:
    Simulation();
    ~Simulation();

    bool loadModel(const std::string& path);
    void reset();
    void step();

    mjModel* model();
    mjData* data();

private:
    mjModel* m_model = nullptr;
    mjData* m_data = nullptr;
};
