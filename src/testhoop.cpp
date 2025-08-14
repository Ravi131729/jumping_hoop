// mj_min_viz_node.cpp (ROS1)
// Build with the CMakeLists below.
// Run: rosrun jumping_hoop mj_min_viz_node _model:=/abs/path/to/hoop.xml _tmax:=15.0 _topic:=/mujoco/qpos

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <cstdio>

// Global MuJoCo state for simplicity (single node)
static mjModel* m = nullptr;
static mjData*  d = nullptr;
static mjvCamera cam;
static mjvOption opt;
static mjvScene  scn;
static mjrContext con;
static GLFWwindow* window = nullptr;

// GLFW error callback
static void glfw_error_callback(int error, const char* desc) {
  fprintf(stderr, "GLFW error %d: %s\n", error, desc);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mj_min_viz_node");
  ros::NodeHandle nh("~");

  // Parameters
  std::string model_path; nh.param<std::string>("model", model_path, std::string("hoop.xml"));
  double tmax; nh.param("tmax", tmax, 15.0);
  std::string topic; nh.param<std::string>("topic", topic, std::string("/mujoco/qpos"));

  // Load MuJoCo
  char error[1024] = {0};
  m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    ROS_ERROR_STREAM("MuJoCo load error: " << error << " (model=" << model_path << ")");
    return 1;
  }
  d = mj_makeData(m);

  // Init GLFW / viewer
  glfwSetErrorCallback(glfw_error_callback);
  if (!glfwInit()) {
    ROS_ERROR("Failed to init GLFW (check DISPLAY / drivers).");
    mj_deleteData(d); mj_deleteModel(m);
    return 1;
  }

  // Create window + GL context
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  window = glfwCreateWindow(1280, 720, "MuJoCo (jumping_hoop)", nullptr, nullptr);
  if (!window) {
    ROS_ERROR("Failed to create GLFW window.");
    glfwTerminate();
    mj_deleteData(d); mj_deleteModel(m);
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // vsync

  // MuJoCo visualization state
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // Camera similar to your Python viewer
  cam.type = mjCAMERA_FREE;
  cam.distance = 1.0;
  cam.azimuth  = 90.0;
  cam.elevation = -10.0;

  // ROS publisher
  ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(topic, 10);
  ROS_INFO_STREAM("Started. nq=" << m->nq << " timestep=" << m->opt.timestep
                  << "  publishing: " << topic);

  // Timing
  const double dt = m->opt.timestep;
  auto wall_t0 = std::chrono::steady_clock::now();

  // Main loop
  while (ros::ok() && !glfwWindowShouldClose(window) && d->time < tmax) {
    auto step_start = std::chrono::steady_clock::now();
    double dt_wall = std::chrono::duration<double>(step_start - wall_t0).count();

    // Example control schedule (your original)
    if (dt_wall < 6.0)         d->ctrl[0] = 0.03;
    else if (dt_wall < 8.0)    d->ctrl[0] = 0.08;
    else if (dt_wall < 10.0)   d->ctrl[0] = 0.30;
    else if (dt_wall < 12.0)   d->ctrl[0] = 0.20;
    else                       d->ctrl[0] = 0.0;

    // Step simulation
    mj_step(m, d);

    // Publish qpos
    std_msgs::Float64MultiArray msg;
    msg.data.resize(m->nq);
    for (int i = 0; i < m->nq; ++i) msg.data[i] = d->qpos[i];
    pub.publish(msg);

    // Camera follows free body position if present
    if (m->nq >= 3) {
      cam.lookat[0] = d->qpos[0];
      cam.lookat[1] = d->qpos[1];
      cam.lookat[2] = d->qpos[2];
    }

    // Render
    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    mjrRect viewport = {0, 0, width, height};

    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();

    ros::spinOnce();

    // Keep roughly real-time
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start).count();
    if (elapsed < dt) std::this_thread::sleep_for(std::chrono::duration<double>(dt - elapsed));
  }

  // Cleanup
  mjr_freeContext(&con);
  mjv_freeScene(&scn);
  if (window) glfwDestroyWindow(window);
  glfwTerminate();
  mj_deleteData(d);
  mj_deleteModel(m);
  ROS_INFO("MuJoCo viz node finished.");
  return 0;
}
