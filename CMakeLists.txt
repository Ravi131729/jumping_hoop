cmake_minimum_required(VERSION 3.0.2)
project(jumping_hoop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# ---------------- OpenGL (prefer GLVND to silence warning) ----------------
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)           # provides OpenGL::GL

# ---------------- MuJoCo: try CONFIG first, then manual fallback ----------
# Try the installed CMake package (created by `cmake --install .`)
set(MUJOCO_HINT_DIRS /usr/local /usr/local/lib/cmake/mujoco /usr/local/lib64/cmake/mujoco)
find_package(mujoco QUIET CONFIG HINTS ${MUJOCO_HINT_DIRS})

set(HAVE_MUJOCO_CONFIG OFF)
if(mujoco_FOUND)
  set(HAVE_MUJOCO_CONFIG ON)           # using mujoco::mujoco target
else()
  # Fallback: manual search for headers and lib in /usr/local
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
            HINTS /usr/local/include /usr/include)
  find_library(MUJOCO_LIBRARY mujoco
               HINTS /usr/local/lib /usr/lib)
  if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "MuJoCo not found. Ensure it is installed in /usr/local or provide MUJOCO paths.")
  endif()
endif()

# ---------------- GLFW: prefer CMake package, fallback to pkg-config ------
set(GLFW_TARGET "")
find_package(glfw3 QUIET)               # defines target `glfw` in some distros
if(TARGET glfw)
  set(GLFW_TARGET glfw)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GLFW3 REQUIRED glfw3)    # requires: sudo apt install libglfw3-dev
endif()

# ---------------- Catkin package ------------------------------------------
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  $<IF:$<BOOL:${HAVE_MUJOCO_CONFIG}>,,${MUJOCO_INCLUDE_DIR}>
  $<IF:$<BOOL:${GLFW_TARGET}>,,${GLFW3_INCLUDE_DIRS}>
)

link_directories(
  $<IF:$<BOOL:${GLFW_TARGET}>,,${GLFW3_LIBRARY_DIRS}>
)

# ---------------- Executables ---------------------------------------------

# Headless publisher (optional — uncomment if you also want it)
# add_executable(mj_min_node src/mj_min_node.cpp)
# if(HAVE_MUJOCO_CONFIG)
#   target_link_libraries(mj_min_node
#     ${catkin_LIBRARIES}
#     mujoco::mujoco
#     m dl pthread
#   )
# else()
#   target_link_libraries(mj_min_node
#     ${catkin_LIBRARIES}
#     ${MUJOCO_LIBRARY}
#     m dl pthread
#   )
# endif()

# Viewer + publisher node built from your file:
add_executable(testhoop_node src/testhoop.cpp)
if(HAVE_MUJOCO_CONFIG)
  target_link_libraries(testhoop_node
    ${catkin_LIBRARIES}
    mujoco::mujoco
    OpenGL::GL
    $<IF:$<BOOL:${GLFW_TARGET}>,${GLFW_TARGET},${GLFW3_LIBRARIES}>
    m dl pthread
  )
else()
  target_link_libraries(testhoop_node
    ${catkin_LIBRARIES}
    ${MUJOCO_LIBRARY}
    OpenGL::GL
    $<IF:$<BOOL:${GLFW_TARGET}>,${GLFW_TARGET},${GLFW3_LIBRARIES}>
    m dl pthread
  )
endif()
