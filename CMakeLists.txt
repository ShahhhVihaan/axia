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
       SHOW_PROGRESS)
endif()


# TODO: Add checksum
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


# Menagerie
set(MENAGERIE_URL "https://github.com/deepmind/mujoco_menagerie/archive/refs/heads/main.zip")
set(MENAGERIE_ARCHIVE "${CMAKE_BINARY_DIR}/menagerie.zip")
set(MENAGERIE_DIR "${CMAKE_BINARY_DIR}/menagerie")

if(NOT EXISTS "${MENAGERIE_ARCHIVE}")
  message(STATUS "Downloading MuJoCo Menagerie...")
  file(DOWNLOAD "${MENAGERIE_URL}" "${MENAGERIE_ARCHIVE}"
       SHOW_PROGRESS)
endif()

if(NOT EXISTS "${MENAGERIE_DIR}/README.md")
  message(STATUS "Extracting MuJoCo Menagerie...")
  file(MAKE_DIRECTORY "${MENAGERIE_DIR}")
  execute_process(
    COMMAND unzip -oq "${MENAGERIE_ARCHIVE}" -d "${MENAGERIE_DIR}"
    RESULT_VARIABLE UNZIP_RESULT
  )
  if(NOT UNZIP_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to extract MuJoCo Menagerie archive")
  endif()

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            "${MENAGERIE_DIR}/mujoco_menagerie-main"
            "${CMAKE_BINARY_DIR}/menagerie-link"
    RESULT_VARIABLE SYMLINK_RESULT
  )
  if(NOT SYMLINK_RESULT EQUAL 0)
    message(WARNING "Failed to create symlink to menagerie")
  endif()

endif()



include_directories("${MUJOCO_INCLUDE_DIR}")
link_directories("${MUJOCO_LIB_DIR}")

add_executable(main src/main.cpp)
target_link_libraries(main mujoco glfw pthread m dl)
