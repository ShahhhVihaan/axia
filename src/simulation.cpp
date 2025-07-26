#include "simulation.h"
#include <iostream>
#include <cstring>

Simulation::Simulation() = default;

Simulation::~Simulation() {
    mj_deleteData(m_data);
    mj_deleteModel(m_model);
}

bool Simulation::loadModel(const std::string& path) {
    char error[1000];
    if (path.size() > 4 && path.substr(path.size() - 4) == ".mjb") {
        m_model = mj_loadModel(path.c_str(), nullptr);
    } else {
        m_model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    }

    if (!m_model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return false;
    }

    m_data = mj_makeData(m_model);
    return true;
}

void Simulation::reset() {
    if (m_model && m_data) {
        mj_resetData(m_model, m_data);
        mj_forward(m_model, m_data);
    }
}

void Simulation::step() {
    if (m_model && m_data) {
        mj_step(m_model, m_data);
    }
}

mjModel* Simulation::model() {
    return m_model;
}

mjData* Simulation::data() {
    return m_data;
}
