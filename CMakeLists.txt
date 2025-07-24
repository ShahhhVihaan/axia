cmake_minimum_required(VERSION 3.16)
project(MyMuJoCoProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Explicit MuJoCo path
set(MUJOCO_DIR "/home/vihaan/mujoco-3.3.4")

include_directories(${MUJOCO_DIR}/include)
link_directories(${MUJOCO_DIR}/lib)

add_executable(my_app src/main.cpp)

target_link_libraries(my_app mujoco pthread m dl)
