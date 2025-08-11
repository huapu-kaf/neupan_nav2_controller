#include "neupan_nav2_controller/neupan_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <cstdlib>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace neupan_nav2_controller
{

NeuPANController::~NeuPANController()
{
  cleanupPython();
}

void NeuPANController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  node_ = parent;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  python_initialized_ = false;

  RCLCPP_INFO(logger_, "Configuring NeuPAN Controller Plugin: %s", plugin_name_.c_str());

  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_velocity", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".robot_type", rclcpp::ParameterValue("omni"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".laser_topic", rclcpp::ParameterValue("/scan"));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".dune_model_path", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".neupan_config_path", rclcpp::ParameterValue(""));

  node->get_parameter(plugin_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);
  node->get_parameter(plugin_name_ + ".robot_type", robot_type_);
  node->get_parameter(plugin_name_ + ".laser_topic", laser_topic_);
  node->get_parameter(plugin_name_ + ".dune_model_path", dune_model_path_);
  node->get_parameter(plugin_name_ + ".neupan_config_path", neupan_config_path_);

  RCLCPP_INFO(
    logger_, "NeuPAN Controller configured with robot_type: %s, max_linear_vel: %.2f, max_angular_vel: %.2f",
    robot_type_.c_str(), max_linear_velocity_, max_angular_velocity_);
  RCLCPP_INFO(
    logger_, "DUNE model path: %s", dune_model_path_.c_str());
  RCLCPP_INFO(
    logger_, "NeuPAN config path: %s", neupan_config_path_.c_str());

  // Initialize Python interpreter and NeuPAN module
  if (!initializePython()) {
    throw nav2_core::PlannerException("Failed to initialize Python interpreter and NeuPAN module!");
  }

  // Subscribe to laser scan
  laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      latest_laser_scan_ = msg;
    });

  RCLCPP_INFO(logger_, "NeuPAN Controller configured successfully");
}

void NeuPANController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up NeuPAN Controller");
  
  cleanupPython();
  laser_sub_.reset();
  
  RCLCPP_INFO(logger_, "NeuPAN Controller cleaned up successfully");
}

void NeuPANController::activate()
{
  RCLCPP_INFO(logger_, "Activating NeuPAN Controller");
  
  if (!python_initialized_) {
    if (!initializePython()) {
      throw nav2_core::PlannerException("Failed to reinitialize Python during activation!");
    }
  }
  
  RCLCPP_INFO(logger_, "NeuPAN Controller activated successfully");
}

void NeuPANController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating NeuPAN Controller");
  // Keep Python initialized for potential reactivation
  RCLCPP_INFO(logger_, "NeuPAN Controller deactivated successfully");
}

void NeuPANController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_DEBUG(logger_, "New global plan set with %zu waypoints", path.poses.size());
  
  // Convert Nav2 path to NeuPAN initial path format
  if (python_initialized_ && neupan_core_instance_ && !path.poses.empty()) {
    convertNav2PathToNeuPAN(path);
  }
}

geometry_msgs::msg::TwistStamped NeuPANController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  auto cmd = geometry_msgs::msg::TwistStamped();
  cmd.header.stamp = pose.header.stamp;
  cmd.header.frame_id = pose.header.frame_id;

  // Check if we have a valid plan
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "No global plan available");
    return cmd;
  }

  // Check if goal is reached
  if (goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    RCLCPP_INFO(logger_, "Goal reached!");
    return cmd;  // Return zero velocity
  }

  // Get current robot state
  std::vector<double> robot_state = {
    pose.pose.position.x,
    pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation)
  };

  // Get obstacle points from laser scan
  auto laser_scan = getLatestLaserScan();
  if (!laser_scan) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 1000, "No laser scan available");
    return cmd;
  }

  auto obstacle_points = laserScanToObstaclePoints(laser_scan, pose);

  // Call NeuPAN planner
  if (!callNeuPANPlanner(robot_state, obstacle_points, cmd.twist)) {
    RCLCPP_WARN(logger_, "NeuPAN planner failed, returning zero velocity");
    return cmd;
  }

  // Apply velocity limits
  cmd.twist.linear.x = std::clamp(cmd.twist.linear.x, -max_linear_velocity_, max_linear_velocity_);
  cmd.twist.linear.y = std::clamp(cmd.twist.linear.y, -max_linear_velocity_, max_linear_velocity_);
  cmd.twist.angular.z = std::clamp(cmd.twist.angular.z, -max_angular_velocity_, max_angular_velocity_);

  RCLCPP_DEBUG(
    logger_, "NeuPAN computed velocity: linear=[%.3f, %.3f], angular=%.3f",
    cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z);

  return cmd;
}

void NeuPANController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_linear_velocity_ *= speed_limit;
    max_angular_velocity_ *= speed_limit;
  } else {
    max_linear_velocity_ = speed_limit;
    max_angular_velocity_ = speed_limit;
  }
  
  RCLCPP_INFO(
    logger_, "Speed limit set: linear=%.2f, angular=%.2f", 
    max_linear_velocity_, max_angular_velocity_);
}

bool NeuPANController::initializePython()
{
  if (python_initialized_) {
    return true;
  }

  RCLCPP_INFO(logger_, "Initializing Python interpreter for NeuPAN");

  // Initialize Python interpreter
  if (!Py_IsInitialized()) {
    Py_Initialize();
    if (!Py_IsInitialized()) {
      RCLCPP_ERROR(logger_, "Failed to initialize Python interpreter");
      return false;
    }
  }

  // Initialize NumPy
  if (_import_array() < 0) {
    RCLCPP_ERROR(logger_, "Failed to initialize NumPy C API");
    return false;
  }

  // Add paths to Python path - try multiple common locations
  PyRun_SimpleString("import sys");
  PyRun_SimpleString("import os");
  
  // Get the path of the current package and search relative to it
  PyRun_SimpleString("import rclpy");
  PyRun_SimpleString("import rclpy.logging");
  
  // Try to find NeuPAN in common installation locations
  std::string vendor_base_path;
  try {
    vendor_base_path = ament_index_cpp::get_package_share_directory("neupan_nav2_controller");
    vendor_base_path += "/vendor/NeuPAN-main";
  } catch (const std::exception &e) {
    vendor_base_path.clear();
  }
  // Normalize path separators just in case
  std::replace(vendor_base_path.begin(), vendor_base_path.end(), '\\', '/');

  std::vector<std::string> search_paths = {
    // Package vendored copy (preferred)
    vendor_base_path.empty() ? std::string("") : std::string("sys.path.append('" + vendor_base_path + "')"),
    vendor_base_path.empty() ? std::string("") : std::string("sys.path.append('" + vendor_base_path + "/neupan')"),
    // Relative to current working directory
    "sys.path.append(os.path.join(os.getcwd(), 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.getcwd(), '..', 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.getcwd(), '..', '..', 'src', 'NeuPAN-main'))",
    // Try to find in home directory or typical ROS2 workspace locations  
    "sys.path.append(os.path.expanduser('~/NeuPAN-main'))",
    "sys.path.append(os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.path.expanduser('~'), 'nav2_ws', 'src', 'NeuPAN-main'))",
    // Try system-wide Python package location
    "sys.path.append('/usr/local/lib/python3.10/site-packages')",
    "sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')"
  };
  
  for (const auto& path_cmd : search_paths) {
    PyRun_SimpleString(path_cmd.c_str());
  }

  // Import necessary modules
  PyRun_SimpleString("import numpy as np");
  
  // Import neupan module directly for algorithm access
  PyObject* neupan_module = PyImport_ImportModule("neupan.neupan");
  if (!neupan_module) {
    RCLCPP_ERROR(logger_, "Failed to import neupan.neupan module. Make sure NeuPAN is installed and accessible.");
    PyErr_Print();
    return false;
  }

  // Get neupan class from the module
  PyObject* neupan_class = PyObject_GetAttrString(neupan_module, "neupan");
  if (!neupan_class) {
    RCLCPP_ERROR(logger_, "Cannot find neupan class in module");
    Py_DECREF(neupan_module);
    return false;
  }

  // Get init_from_yaml class method
  PyObject* init_func = PyObject_GetAttrString(neupan_class, "init_from_yaml");
  if (!init_func || !PyCallable_Check(init_func)) {
    RCLCPP_ERROR(logger_, "Cannot find init_from_yaml class method");
    Py_DECREF(neupan_class);
    Py_DECREF(neupan_module);
    return false;
  }

  // Prepare arguments for neupan initialization - smart config file search
  std::string config_file = neupan_config_path_;
  
  // Use default config if not specified - try multiple locations
  if (config_file.empty()) {
    std::vector<std::string> candidate_configs;
    
    if (robot_type_ == "omni") {
      candidate_configs = {
        vendor_base_path.empty() ? std::string("") : vendor_base_path + "/example/non_obs/omni/planner.yaml",
        "NeuPAN-main/example/non_obs/omni/planner.yaml",
        "../NeuPAN-main/example/non_obs/omni/planner.yaml",
        "../../src/NeuPAN-main/example/non_obs/omni/planner.yaml",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/NeuPAN-main/example/non_obs/omni/planner.yaml",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ros2_ws/src/NeuPAN-main/example/non_obs/omni/planner.yaml"
      };
    } else if (robot_type_ == "diff") {
      candidate_configs = {
        vendor_base_path.empty() ? std::string("") : vendor_base_path + "/example/non_obs/diff/planner.yaml",
        "NeuPAN-main/example/non_obs/diff/planner.yaml",
        "../NeuPAN-main/example/non_obs/diff/planner.yaml",
        "../../src/NeuPAN-main/example/non_obs/diff/planner.yaml",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/NeuPAN-main/example/non_obs/diff/planner.yaml",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ros2_ws/src/NeuPAN-main/example/non_obs/diff/planner.yaml"
      };
    } else { // acker or default
      candidate_configs = {
        vendor_base_path.empty() ? std::string("") : vendor_base_path + "/example/non_obs/acker/planner.yaml",
        "NeuPAN-main/example/non_obs/acker/planner.yaml", 
        "../NeuPAN-main/example/non_obs/acker/planner.yaml",
        "../../src/NeuPAN-main/example/non_obs/acker/planner.yaml",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/NeuPAN-main/example/non_obs/acker/planner.yaml",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/ros2_ws/src/NeuPAN-main/example/non_obs/acker/planner.yaml"
      };
    }
    
    // Try each candidate config file
    bool found = false;
    for (const auto& candidate : candidate_configs) {
      if (candidate.empty()) continue;
      std::ifstream test_file(candidate);
      if (test_file.good()) {
        config_file = candidate;
        found = true;
        test_file.close();
        break;
      }
    }
    
    if (!found) {
      RCLCPP_ERROR(logger_, "No default NeuPAN config file found for robot type: %s", robot_type_.c_str());
      RCLCPP_ERROR(logger_, "Please specify neupan_config_path parameter or ensure NeuPAN-main is accessible");
      Py_DECREF(init_func);
      Py_DECREF(neupan_class);
      Py_DECREF(neupan_module);
      return false;
    }
  } else {
    // Validate user-specified config file exists
    std::ifstream config_test(config_file);
    if (!config_test.good()) {
      RCLCPP_ERROR(logger_, "User-specified NeuPAN config file not found: %s", config_file.c_str());
      Py_DECREF(init_func);
      Py_DECREF(neupan_class);
      Py_DECREF(neupan_module);
      return false;
    }
    config_test.close();
  }

  PyObject* config_arg = PyUnicode_FromString(config_file.c_str());
  
  // Create pan dictionary with DUNE model path
  PyObject* pan_dict = PyDict_New();
  
  // Smart DUNE model path search
  std::string actual_model_path = dune_model_path_;
  
  if (actual_model_path.empty()) {
    // Try to find default model based on robot type
    std::vector<std::string> candidate_models;
    
    if (robot_type_ == "omni") {
      candidate_models = {
        // Package local models directory (highest priority)
        (vendor_base_path.empty() ? std::string("") : (vendor_base_path.substr(0, vendor_base_path.find("vendor/NeuPAN-main")) + std::string("model/omni_robot_default/model_5000.pth"))),
        // Vendored NeuPAN-main fallback
        vendor_base_path.empty() ? std::string("") : vendor_base_path + "/example/model/omni_robot_default/model_5000.pth",
        "NeuPAN-main/example/model/diff_robot_default/model_5000.pth",
        "../NeuPAN-main/example/model/diff_robot_default/model_5000.pth",
        "../../src/NeuPAN-main/example/model/diff_robot_default/model_5000.pth",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/NeuPAN-main/example/model/diff_robot_default/model_5000.pth"
      };
    } else if (robot_type_ == "diff") {
      candidate_models = {
        (vendor_base_path.empty() ? std::string("") : (vendor_base_path.substr(0, vendor_base_path.find("vendor/NeuPAN-main")) + std::string("model/diff_robot_default/model_5000.pth"))),
        vendor_base_path.empty() ? std::string("") : vendor_base_path + "/example/model/diff_robot_default/model_5000.pth",
        "NeuPAN-main/example/model/diff_robot_default/model_5000.pth",
        "../NeuPAN-main/example/model/diff_robot_default/model_5000.pth",
        "../../src/NeuPAN-main/example/model/diff_robot_default/model_5000.pth",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/NeuPAN-main/example/model/diff_robot_default/model_5000.pth"
      };
    } else { // acker
      candidate_models = {
        (vendor_base_path.empty() ? std::string("") : (vendor_base_path.substr(0, vendor_base_path.find("vendor/NeuPAN-main")) + std::string("model/acker_robot_default/model_5000.pth"))),
        vendor_base_path.empty() ? std::string("") : vendor_base_path + "/example/model/acker_robot_default/model_5000.pth",
        "NeuPAN-main/example/model/acker_robot_default/model_5000.pth",
        "../NeuPAN-main/example/model/acker_robot_default/model_5000.pth",
        "../../src/NeuPAN-main/example/model/acker_robot_default/model_5000.pth",
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "") + "/NeuPAN-main/example/model/acker_robot_default/model_5000.pth"
      };
    }
    
    // Try each candidate model file
    bool model_found = false;
    for (const auto& candidate : candidate_models) {
      if (candidate.empty()) continue;
      std::ifstream test_model(candidate);
      if (test_model.good()) {
        actual_model_path = candidate;
        model_found = true;
        test_model.close();
        break;
      }
    }

    // If still not found, scan package local models directory for any *.pth
    if (!model_found) {
      std::string package_share;
      try {
        package_share = ament_index_cpp::get_package_share_directory("neupan_nav2_controller");
      } catch (...) {}
      if (!package_share.empty()) {
        std::string models_dir = package_share + "/model/" + (robot_type_.empty() ? std::string("diff_robot_default") : (robot_type_ + std::string("_robot_default")));
        std::replace(models_dir.begin(), models_dir.end(), '\\', '/');
        // Fallback to a fixed filename without globbing: model_5000.pth
        std::ifstream test_best(models_dir + "/model_5000.pth");
        if (test_best.good()) {
          actual_model_path = models_dir + "/model_5000.pth";
          model_found = true;
          test_best.close();
        }
      }
    }
    
    if (!model_found) {
      RCLCPP_WARN(logger_, "No default DUNE model found for robot type: %s", robot_type_.c_str());
      RCLCPP_WARN(logger_, "Will use model path from config file");
    } else {
      RCLCPP_INFO(logger_, "Found default DUNE model: %s", actual_model_path.c_str());
    }
  } else {
    // Validate user-specified model file exists
    std::ifstream model_test(actual_model_path);
    if (!model_test.good()) {
      RCLCPP_ERROR(logger_, "User-specified DUNE model file not found: %s", actual_model_path.c_str());
      Py_DECREF(config_arg);
      Py_DECREF(pan_dict);
      Py_DECREF(init_func);
      Py_DECREF(neupan_class);
      Py_DECREF(neupan_module);
      return false;
    }
    model_test.close();
  }
  
  // Add model path to pan dictionary if found
  if (!actual_model_path.empty()) {
    PyObject* model_path_obj = PyUnicode_FromString(actual_model_path.c_str());
    PyDict_SetItemString(pan_dict, "dune_checkpoint", model_path_obj);
    Py_DECREF(model_path_obj);
    RCLCPP_INFO(logger_, "Using DUNE model: %s", actual_model_path.c_str());
  }
  
  PyObject* args = PyTuple_New(2);
  PyTuple_SetItem(args, 0, config_arg);
  PyTuple_SetItem(args, 1, pan_dict);

  // Call init_from_yaml to create neupan planner instance
  neupan_core_instance_ = PyObject_CallObject(init_func, args);
  
  Py_DECREF(args);
  Py_DECREF(init_func);
  Py_DECREF(neupan_class);
  Py_DECREF(neupan_module);

  if (!neupan_core_instance_) {
    RCLCPP_ERROR(logger_, "Failed to create NeuPAN planner instance");
    PyErr_Print();
    return false;
  }

  python_initialized_ = true;
  RCLCPP_INFO(logger_, "Python initialization completed successfully");
  return true;
}

void NeuPANController::cleanupPython()
{
  if (!python_initialized_) {
    return;
  }

  RCLCPP_INFO(logger_, "Cleaning up Python resources");

  if (neupan_core_instance_) {
    Py_DECREF(neupan_core_instance_);
    neupan_core_instance_ = nullptr;
  }


  python_initialized_ = false;
  RCLCPP_INFO(logger_, "Python cleanup completed");
}

bool NeuPANController::callNeuPANPlanner(
  const std::vector<double> & robot_state,
  const std::vector<std::vector<double>> & obstacle_points,
  geometry_msgs::msg::Twist & cmd_vel)
{
  if (!python_initialized_ || !neupan_core_instance_) {
    RCLCPP_ERROR(logger_, "Python not initialized");
    return false;
  }

  try {
    // Convert robot state to numpy array
    npy_intp robot_dims[2] = {3, 1};
    PyObject* robot_array = PyArray_SimpleNew(2, robot_dims, NPY_DOUBLE);
    if (!robot_array) {
      RCLCPP_ERROR(logger_, "Failed to create robot state array");
      return false;
    }

    double* robot_data = static_cast<double*>(PyArray_DATA((PyArrayObject*)robot_array));
    robot_data[0] = robot_state[0];  // x
    robot_data[1] = robot_state[1];  // y  
    robot_data[2] = robot_state[2];  // theta

    // Convert obstacle points to numpy array
    PyObject* obs_array = nullptr;
    if (!obstacle_points.empty()) {
      npy_intp obs_dims[2] = {2, static_cast<npy_intp>(obstacle_points.size())};
      obs_array = PyArray_SimpleNew(2, obs_dims, NPY_DOUBLE);
      if (!obs_array) {
        RCLCPP_ERROR(logger_, "Failed to create obstacle points array");
        Py_DECREF(robot_array);
        return false;
      }

      double* obs_data = static_cast<double*>(PyArray_DATA((PyArrayObject*)obs_array));
      for (size_t i = 0; i < obstacle_points.size(); ++i) {
        obs_data[i] = obstacle_points[i][0];  // x coordinate
        obs_data[i + obstacle_points.size()] = obstacle_points[i][1];  // y coordinate
      }
    } else {
      obs_array = Py_None;
      Py_INCREF(Py_None);
    }

    // Get forward method from neupan instance
    PyObject* forward_method = PyObject_GetAttrString(neupan_core_instance_, "forward");
    if (!forward_method || !PyCallable_Check(forward_method)) {
      RCLCPP_ERROR(logger_, "Cannot find forward method in NeuPAN planner");
      Py_XDECREF(forward_method);
      Py_DECREF(robot_array);
      if (obs_array != Py_None) Py_DECREF(obs_array);
      return false;
    }

    // Call neupan planner forward method with 3 parameters: state, points, velocities
    PyObject* args = PyTuple_New(3);
    PyTuple_SetItem(args, 0, robot_array);
    PyTuple_SetItem(args, 1, obs_array);
    Py_INCREF(Py_None);  // Increment reference count before stealing
    PyTuple_SetItem(args, 2, Py_None);  // velocities = None

    PyObject* result = PyObject_CallObject(forward_method, args);
    Py_DECREF(args);
    Py_DECREF(forward_method);

    if (!result) {
      RCLCPP_ERROR(logger_, "Failed to call NeuPAN planner");
      PyErr_Print();
      return false;
    }

    // Parse result tuple (action, info)
    if (!PyTuple_Check(result) || PyTuple_Size(result) != 2) {
      RCLCPP_ERROR(logger_, "Invalid return format from NeuPAN planner");
      Py_DECREF(result);
      return false;
    }

    PyObject* action = PyTuple_GetItem(result, 0);
    PyObject* info_dict = PyTuple_GetItem(result, 1);

    // Check if we should stop
    PyObject* stop_obj = PyDict_GetItemString(info_dict, "stop");
    PyObject* arrive_obj = PyDict_GetItemString(info_dict, "arrive");
    
    bool stop = (stop_obj && PyObject_IsTrue(stop_obj));
    bool arrive = (arrive_obj && PyObject_IsTrue(arrive_obj));

    if (stop || arrive) {
      // Return zero velocity
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      
      if (arrive) {
        RCLCPP_INFO(logger_, "Goal reached!");
      } else {
        RCLCPP_WARN(logger_, "NeuPAN stopped due to safety constraints");
      }
      
      Py_DECREF(result);
      return true;
    }

    // Extract velocity commands from action
    if (!PyArray_Check(action)) {
      RCLCPP_ERROR(logger_, "Action is not a numpy array");
      Py_DECREF(result);
      return false;
    }

    PyArrayObject* action_array = (PyArrayObject*)action;
    double* action_data = static_cast<double*>(PyArray_DATA(action_array));

    // Convert based on robot kinematics
    if (robot_type_ == "omni") {
      // Omnidirectional: action[0] = vx, action[1] = vy
      cmd_vel.linear.x = action_data[0];
      cmd_vel.linear.y = action_data[1];
      cmd_vel.angular.z = 0.0;
    } else if (robot_type_ == "diff") {
      // Differential: action[0] = linear_velocity, action[1] = angular_velocity
      cmd_vel.linear.x = action_data[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = action_data[1];
    } else if (robot_type_ == "acker") {
      // Ackermann: action[0] = linear_velocity, action[1] = steering_angle
      cmd_vel.linear.x = action_data[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = action_data[1];
    } else {
      RCLCPP_WARN(logger_, "Unknown robot type: %s, using differential drive", robot_type_.c_str());
      cmd_vel.linear.x = action_data[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = action_data[1];
    }

    Py_DECREF(result);
    
    RCLCPP_DEBUG(
      logger_, "NeuPAN computed velocity: linear=[%.3f, %.3f], angular=%.3f",
      cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in callNeuPANPlanner: %s", e.what());
    return false;
  }
}

std::vector<std::vector<double>> NeuPANController::laserScanToObstaclePoints(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  std::vector<std::vector<double>> obstacle_points;

  if (!scan) {
    return obstacle_points;
  }

  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];
    
    // Skip invalid measurements
    if (range < scan->range_min || range > scan->range_max || std::isnan(range) || std::isinf(range)) {
      continue;
    }

    // Calculate angle
    double angle = scan->angle_min + i * scan->angle_increment;
    double global_angle = robot_yaw + angle;

    // Convert to global coordinates
    double obs_x = robot_x + range * cos(global_angle);
    double obs_y = robot_y + range * sin(global_angle);

    obstacle_points.push_back({obs_x, obs_y});
  }

  RCLCPP_DEBUG(logger_, "Converted laser scan to %zu obstacle points", obstacle_points.size());
  return obstacle_points;
}

sensor_msgs::msg::LaserScan::ConstSharedPtr NeuPANController::getLatestLaserScan()
{
  return latest_laser_scan_;
}

void NeuPANController::convertNav2PathToNeuPAN(const nav_msgs::msg::Path & path)
{
  if (!python_initialized_ || !neupan_core_instance_) {
    RCLCPP_WARN(logger_, "Python not initialized, cannot set NeuPAN initial path");
    return;
  }

  try {
    // Create Python list of waypoints in NeuPAN format: [x, y, theta, 1]
    PyObject* waypoint_list = PyList_New(0);
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto& pose = path.poses[i].pose;
      
      double x = pose.position.x;
      double y = pose.position.y;
      double theta = tf2::getYaw(pose.orientation);
      
      // Create numpy array (4, 1) for [x, y, theta, 1]
      npy_intp dims[2] = {4, 1};
      PyObject* waypoint_array = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
      if (!waypoint_array) {
        RCLCPP_ERROR(logger_, "Failed to create waypoint array");
        Py_DECREF(waypoint_list);
        return;
      }
      
      double* waypoint_data = static_cast<double*>(PyArray_DATA((PyArrayObject*)waypoint_array));
      waypoint_data[0] = x;
      waypoint_data[1] = y;  
      waypoint_data[2] = theta;
      waypoint_data[3] = 1.0;  // NeuPAN format requirement
      
      PyList_Append(waypoint_list, waypoint_array);
      Py_DECREF(waypoint_array);
    }
    
    // Get set_initial_path method from neupan planner
    PyObject* set_path_method = PyObject_GetAttrString(neupan_core_instance_, "set_initial_path");
    if (!set_path_method || !PyCallable_Check(set_path_method)) {
      RCLCPP_ERROR(logger_, "Cannot find set_initial_path method");
      Py_DECREF(waypoint_list);
      return;
    }
    
    // Call set_initial_path(waypoint_list)
    PyObject* args = PyTuple_New(1);
    PyTuple_SetItem(args, 0, waypoint_list);
    
    PyObject* result = PyObject_CallObject(set_path_method, args);
    Py_DECREF(args);
    Py_DECREF(set_path_method);
    
    if (!result) {
      RCLCPP_ERROR(logger_, "Failed to set initial path in NeuPAN");
      PyErr_Print();
      return;
    }
    
    Py_DECREF(result);
    RCLCPP_INFO(logger_, "Successfully set NeuPAN initial path with %zu waypoints", path.poses.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in convertNav2PathToNeuPAN: %s", e.what());
  }
}

}  // namespace neupan_nav2_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(neupan_nav2_controller::NeuPANController, nav2_core::Controller) 