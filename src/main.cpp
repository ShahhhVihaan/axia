#include <mujoco/mujoco.h>
#include <iostream>

int main() {
    std::cout << "MuJoCo version: " << mj_versionString() << std::endl;
    return 0;
}
