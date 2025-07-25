cmake_minimum_required(VERSION 3.16)
project(axia)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MUJOCO_VERSION 3.3.4)
set(MUJOCO_NAME mujoco-${MUJOCO_VERSION})
set(MUJOCO_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_NAME}-linux-x86_64.tar.gz")

set(MUJOCO_ARCHIVE "${CMAKE_BINARY_DIR}/mujoco.tar.gz")
set(MUJOCO_BASE_DIR "${CMAKE_BINARY_DIR}/_deps")
set(MUJOCO_EXTRACTED_DIR "${MUJOCO_BASE_DIR}/${MUJOCO_NAME}")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_EXTRACTED_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_EXTRACTED_DIR}/lib")


if(NOT EXISTS "${MUJOCO_ARCHIVE}")
  message(STATUS "Downloading MuJoCo from ${MUJOCO_URL}")
  file(DOWNLOAD "${MUJOCO_URL}" "${MUJOCO_ARCHIVE}"
       EXPECTED_HASH SHA256=ecf1a17459a342badf2b4f32dd4677a6a0e5fd393c5143993eb3e81b8e44609b
       SHOW_PROGRESS) # TODO: use the sha256 file as well
endif()

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(STATUS "Extracting MuJoCo to ${MUJOCO_EXTRACTED_DIR}")
  file(MAKE_DIRECTORY "${MUJOCO_BASE_DIR}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf "${MUJOCO_ARCHIVE}"
    WORKING_DIRECTORY "${MUJOCO_BASE_DIR}"
    RESULT_VARIABLE EXTRACT_RESULT
  )
  if(NOT EXTRACT_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to extract MuJoCo archive")
  endif()
endif()


include_directories("${MUJOCO_INCLUDE_DIR}")
link_directories("${MUJOCO_LIB_DIR}")

add_executable(main src/main.cpp)
target_link_libraries(main mujoco glfw pthread m dl)
