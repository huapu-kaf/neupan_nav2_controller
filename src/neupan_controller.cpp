#include "neupan_nav2_controller/neupan_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <dlfcn.h>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <limits>

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
  python_initialization_in_progress_ = false;
  python_initialization_failed_ = false;
  numpy_initialized_ = false;
  numpy_compatible_ = false;
  numpy_version_ = "";
  python_version_ = "";

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

  // Python initialization will be done in activate() to avoid blocking configure phase
  RCLCPP_INFO(logger_, "Python initialization deferred to activation phase");

  // Subscribe to laser scan
  laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      latest_laser_scan_ = msg;
    });

  // Create local plan publisher for visualization
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "local_plan", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(logger_, "NeuPAN Controller configured successfully");
}

void NeuPANController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up NeuPAN Controller");
  
  // Wait for Python initialization thread to complete
  if (python_init_thread_ && python_init_thread_->joinable()) {
    RCLCPP_INFO(logger_, "Waiting for Python initialization thread to complete...");
    python_init_thread_->join();
  }
  
  cleanupPython();
  laser_sub_.reset();
  local_plan_pub_.reset();
  
  RCLCPP_INFO(logger_, "NeuPAN Controller cleaned up successfully");
}

void NeuPANController::activate()
{
  RCLCPP_INFO(logger_, "Activating NeuPAN Controller");
  
  // Start Python initialization in background thread (non-blocking)
  if (!python_initialized_ && !python_initialization_in_progress_) {
    RCLCPP_INFO(logger_, "Starting Python initialization in background thread...");
    startAsyncPythonInitialization();
  }
  
  RCLCPP_INFO(logger_, "NeuPAN Controller activated successfully (Python initializing asynchronously)");
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

  // 强制输出调试信息
  RCLCPP_INFO(logger_, "🔍 [DEBUG] computeVelocityCommands called!");
  RCLCPP_INFO(logger_, "🔍 [DEBUG] Global plan size: %zu poses", global_plan_.poses.size());
  RCLCPP_INFO(logger_, "🔍 [DEBUG] Python initialized: %s", python_initialized_.load() ? "YES" : "NO");
  RCLCPP_INFO(logger_, "🔍 [DEBUG] Python initializing: %s", python_initialization_in_progress_.load() ? "YES" : "NO");
  RCLCPP_INFO(logger_, "🔍 [DEBUG] Python init failed: %s", python_initialization_failed_.load() ? "YES" : "NO");

  // Check if we have a valid plan
  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "No global plan available");
    return cmd;
  }

  // Check if goal is reached
  if (goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    RCLCPP_INFO(logger_, "🎯 Goal reached!");
    return cmd;  // Return zero velocity
  } else {
    RCLCPP_INFO(logger_, "🔍 [DEBUG] Goal not yet reached, continuing navigation");
  }

  // Get current robot state
  std::vector<double> robot_state = {
    pose.pose.position.x,
    pose.pose.position.y,
    tf2::getYaw(pose.pose.orientation)
  };

  // Get obstacle points from laser scan
  auto laser_scan = getLatestLaserScan();
  RCLCPP_INFO(logger_, "🔍 [DEBUG] Laser scan available: %s", laser_scan ? "YES" : "NO");
  if (!laser_scan) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 1000, "No laser scan available");
    return cmd;
  } else {
    RCLCPP_INFO(logger_, "🔍 [DEBUG] Laser scan has %zu ranges", laser_scan->ranges.size());
  }

  auto obstacle_points = laserScanToObstaclePoints(laser_scan, pose);

  // Call NeuPAN planner only if Python is initialized
  if (python_initialized_) {
    if (!callNeuPANPlanner(robot_state, obstacle_points, cmd.twist)) {
      RCLCPP_WARN(logger_, "NeuPAN planner failed, using fallback behavior");
      // Fallback: simple stop or basic obstacle avoidance
      return cmd;
    }
  } else if (python_initialization_in_progress_) {
    RCLCPP_INFO_THROTTLE(logger_, *node_.lock()->get_clock(), 2000, 
      "🔄 Python initialization in progress, using simple fallback controller");
    // Fallback: use simple goal-seeking behavior while initializing
    return computeSimpleFallbackVelocity(pose, global_plan_);
  } else if (python_initialization_failed_) {
    RCLCPP_DEBUG_THROTTLE(logger_, *node_.lock()->get_clock(), 10000, 
      "Python initialization failed, using fallback behavior");
    // Fallback: return zero velocity when initialization failed
    return cmd;
  } else {
    RCLCPP_DEBUG_THROTTLE(logger_, *node_.lock()->get_clock(), 5000, 
      "Python not initialized yet, using fallback behavior");
    return cmd;
  }

  // Apply velocity limits
  cmd.twist.linear.x = std::clamp(cmd.twist.linear.x, -max_linear_velocity_, max_linear_velocity_);
  cmd.twist.linear.y = std::clamp(cmd.twist.linear.y, -max_linear_velocity_, max_linear_velocity_);
  cmd.twist.angular.z = std::clamp(cmd.twist.angular.z, -max_angular_velocity_, max_angular_velocity_);

  // Create and publish local plan for visualization based on velocity commands
  // This shows the actual trajectory the robot will follow based on the computed velocities
  nav_msgs::msg::Path local_plan = generateVelocityBasedTrajectory(pose, cmd.twist);
  publishLocalPlan(pose, local_plan);

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

  // Initialize Python interpreter with workaround for symbol issues
  if (!Py_IsInitialized()) {
    // Try to avoid the undefined symbol issue by ensuring proper library loading order
    RCLCPP_INFO(logger_, "Pre-loading Python libraries to avoid symbol conflicts...");
    
    // Workaround for PyExc_RecursionError symbol issue: preload Python library
    void* python_lib = dlopen("/usr/lib/x86_64-linux-gnu/libpython3.10.so.1.0", RTLD_LAZY | RTLD_GLOBAL);
    if (python_lib) {
      RCLCPP_INFO(logger_, "Successfully preloaded Python shared library");
    } else {
      RCLCPP_WARN(logger_, "Failed to preload Python library: %s", dlerror());
    }
    
    // Set minimal environment to avoid conflicts
    Py_Initialize();
    if (!Py_IsInitialized()) {
      RCLCPP_ERROR(logger_, "Failed to initialize Python interpreter");
      return false;
    }
    
    // Force import of problematic modules early to detect issues
    RCLCPP_INFO(logger_, "Testing critical Python modules...");
    if (PyRun_SimpleString("import sys; print('Python sys module loaded')") != 0) {
      RCLCPP_ERROR(logger_, "Failed to load Python sys module");
      PyErr_Print();
      return false;
    }
    
    RCLCPP_INFO(logger_, "Python interpreter initialized successfully");
  }

  // Check NumPy compatibility first
  if (!checkNumpyCompatibility()) {
    RCLCPP_ERROR(logger_, "NumPy compatibility check failed");
    return false;
  }
  numpy_compatible_ = true;

  // Initialize NumPy with enhanced error handling
  if (!initializeNumpyWithFallback()) {
    RCLCPP_ERROR(logger_, "Failed to initialize NumPy with all fallback strategies");
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
    // NeuPAN source directory in current ROS2 workspace (HIGHEST PRIORITY)
    "sys.path.append('/home/robotmaster/ros2_ws/src/NeuPAN')",
    // Package vendored copy (preferred)
    vendor_base_path.empty() ? std::string("") : std::string("sys.path.append('" + vendor_base_path + "')"),
    vendor_base_path.empty() ? std::string("") : std::string("sys.path.append('" + vendor_base_path + "/neupan')"),
    // Relative to current working directory
    "sys.path.append(os.path.join(os.getcwd(), 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.getcwd(), '..', 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.getcwd(), '..', '..', 'src', 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.getcwd(), '..', '..', 'src', 'NeuPAN'))",
    // Try to find in home directory or typical ROS2 workspace locations  
    "sys.path.append(os.path.expanduser('~/NeuPAN-main'))",
    "sys.path.append(os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'NeuPAN-main'))",
    "sys.path.append(os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'NeuPAN'))",
    "sys.path.append(os.path.join(os.path.expanduser('~'), 'nav2_ws', 'src', 'NeuPAN-main'))",
    // Try system-wide Python package location
    "sys.path.append('/usr/local/lib/python3.10/site-packages')",
    "sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')"
  };
  
  for (const auto& path_cmd : search_paths) {
    PyRun_SimpleString(path_cmd.c_str());
  }

  // Import necessary modules
  RCLCPP_INFO(logger_, "NumPy initialization completed, importing NumPy for Python usage...");
  if (PyRun_SimpleString("import numpy as np") != 0) {
    RCLCPP_ERROR(logger_, "Failed to import NumPy in Python despite successful initialization");
    PyErr_Print();
    return false;
  }
  
  // Import neupan module directly for algorithm access
  PyObject* neupan_module = PyImport_ImportModule("neupan.neupan");
  if (!neupan_module) {
    RCLCPP_ERROR(logger_, "Failed to import neupan.neupan module. Make sure NeuPAN is installed and accessible.");
    PyErr_Print();
    return false;
  }

  // Get neupan class from the module (class name is lowercase 'neupan')
  PyObject* neupan_class = PyObject_GetAttrString(neupan_module, "neupan");
  if (!neupan_class) {
    RCLCPP_ERROR(logger_, "Cannot find neupan class in module");
    PyErr_Print();
    
    // Debug: Print available attributes in the module
    PyObject* dir_result = PyObject_Dir(neupan_module);
    if (dir_result) {
      PyObject* str_result = PyObject_Str(dir_result);
      if (str_result) {
        const char* attr_list = PyUnicode_AsUTF8(str_result);
        RCLCPP_ERROR(logger_, "Available attributes in neupan module: %s", attr_list);
        Py_DECREF(str_result);
      }
      Py_DECREF(dir_result);
    }
    
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
  
  // Prepare keyword arguments for init_from_yaml
  PyObject* kwargs_dict = PyDict_New();
  
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
      Py_DECREF(kwargs_dict);
      Py_DECREF(init_func);
      Py_DECREF(neupan_class);
      Py_DECREF(neupan_module);
      return false;
    }
    model_test.close();
  }
  
  // Add model path to kwargs dictionary if found
  if (!actual_model_path.empty()) {
    // Create nested pan dictionary for model path
    PyObject* pan_dict = PyDict_New();
    PyObject* model_path_obj = PyUnicode_FromString(actual_model_path.c_str());
    PyDict_SetItemString(pan_dict, "dune_checkpoint", model_path_obj);
    Py_DECREF(model_path_obj);
    
    // Add pan dictionary to kwargs
    PyDict_SetItemString(kwargs_dict, "pan_kwargs", pan_dict);
    Py_DECREF(pan_dict);
    RCLCPP_INFO(logger_, "Using DUNE model: %s", actual_model_path.c_str());
  }
  
  PyObject* args = PyTuple_New(1);
  PyTuple_SetItem(args, 0, config_arg);

  // Call init_from_yaml to create neupan planner instance
  neupan_core_instance_ = PyObject_Call(init_func, args, kwargs_dict);
  
  Py_DECREF(args);
  Py_DECREF(kwargs_dict);
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
    // Convert robot state to numpy array using Python interface to avoid C API issues
    PyObject* robot_array = nullptr;
    
    if (numpy_initialized_) {
      // Use C API if available
      npy_intp robot_dims[2] = {3, 1};
      robot_array = PyArray_SimpleNew(2, robot_dims, NPY_DOUBLE);
      if (robot_array) {
        double* robot_data = static_cast<double*>(PyArray_DATA((PyArrayObject*)robot_array));
        robot_data[0] = robot_state[0];  // x
        robot_data[1] = robot_state[1];  // y  
        robot_data[2] = robot_state[2];  // theta
      }
    }
    
    if (!robot_array) {
      // Fallback to Python interface
      RCLCPP_DEBUG(logger_, "Using Python-only NumPy array creation for robot state");
      PyRun_SimpleString(
        "import numpy as np\n"
        "__robot_state = np.array([[0.0], [0.0], [0.0]], dtype=np.float64)\n");
      
      PyObject* main_module = PyImport_AddModule("__main__");
      PyObject* main_dict = PyModule_GetDict(main_module);
      robot_array = PyDict_GetItemString(main_dict, "__robot_state");
      if (robot_array) {
        Py_INCREF(robot_array); // Keep reference since we're using it
        
        // Set values using Python
        char set_values_cmd[200];
        snprintf(set_values_cmd, sizeof(set_values_cmd), 
          "__robot_state[0,0] = %.6f; __robot_state[1,0] = %.6f; __robot_state[2,0] = %.6f",
          robot_state[0], robot_state[1], robot_state[2]);
        PyRun_SimpleString(set_values_cmd);
      }
    }
    
    if (!robot_array) {
      RCLCPP_ERROR(logger_, "Failed to create robot state array");
      return false;
    }

    // Convert obstacle points to numpy array
    PyObject* obs_array = nullptr;
    if (!obstacle_points.empty()) {
      if (numpy_initialized_) {
        // Use C API if available
        npy_intp obs_dims[2] = {2, static_cast<npy_intp>(obstacle_points.size())};
        obs_array = PyArray_SimpleNew(2, obs_dims, NPY_DOUBLE);
        if (obs_array) {
          double* obs_data = static_cast<double*>(PyArray_DATA((PyArrayObject*)obs_array));
          for (size_t i = 0; i < obstacle_points.size(); ++i) {
            obs_data[i] = obstacle_points[i][0];  // x coordinate
            obs_data[i + obstacle_points.size()] = obstacle_points[i][1];  // y coordinate
          }
        }
      }
      
      if (!obs_array) {
        // Fallback to Python interface
        RCLCPP_DEBUG(logger_, "Using Python-only NumPy array creation for obstacles");
        
        // Build obstacle points string
        std::string obs_data = "[";
        for (size_t i = 0; i < obstacle_points.size(); ++i) {
          if (i > 0) obs_data += ", ";
          obs_data += "[" + std::to_string(obstacle_points[i][0]) + ", " + 
                     std::to_string(obstacle_points[i][1]) + "]";
        }
        obs_data += "]";
        
        std::string create_obs_cmd = "import numpy as np\n__obs_points = np.array(" + obs_data + ", dtype=np.float64).T\n";
        PyRun_SimpleString(create_obs_cmd.c_str());
        
        PyObject* main_module = PyImport_AddModule("__main__");
        PyObject* main_dict = PyModule_GetDict(main_module);
        obs_array = PyDict_GetItemString(main_dict, "__obs_points");
        if (obs_array) {
          Py_INCREF(obs_array);
        }
      }
      
      if (!obs_array) {
        RCLCPP_ERROR(logger_, "Failed to create obstacle points array");
        Py_DECREF(robot_array);
        return false;
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
      PyObject* waypoint_array = nullptr;
      
      if (numpy_initialized_) {
        npy_intp dims[2] = {4, 1};
        waypoint_array = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
        if (waypoint_array) {
          double* waypoint_data = static_cast<double*>(PyArray_DATA((PyArrayObject*)waypoint_array));
          waypoint_data[0] = x;
          waypoint_data[1] = y;  
          waypoint_data[2] = theta;
          waypoint_data[3] = 1.0;  // NeuPAN format requirement
        }
      }
      
      if (!waypoint_array) {
        // Fallback to Python interface
        char create_waypoint_cmd[300];
        snprintf(create_waypoint_cmd, sizeof(create_waypoint_cmd), 
          "import numpy as np\n__waypoint = np.array([[%.6f], [%.6f], [%.6f], [1.0]], dtype=np.float64)\n",
          x, y, theta);
        PyRun_SimpleString(create_waypoint_cmd);
        
        PyObject* main_module = PyImport_AddModule("__main__");
        PyObject* main_dict = PyModule_GetDict(main_module);
        waypoint_array = PyDict_GetItemString(main_dict, "__waypoint");
        if (waypoint_array) {
          Py_INCREF(waypoint_array);
        }
      }
      
      if (!waypoint_array) {
        RCLCPP_ERROR(logger_, "Failed to create waypoint array");
        Py_DECREF(waypoint_list);
        return;
      }
      
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

bool NeuPANController::checkNumpyCompatibility()
{
  RCLCPP_INFO(logger_, "Checking NumPy compatibility...");

  // Get Python version
  PyObject* sys_module = PyImport_ImportModule("sys");
  if (!sys_module) {
    RCLCPP_ERROR(logger_, "Failed to import sys module");
    return false;
  }

  PyObject* version_info = PyObject_GetAttrString(sys_module, "version_info");
  if (version_info) {
    PyObject* major = PyObject_GetAttrString(version_info, "major");
    PyObject* minor = PyObject_GetAttrString(version_info, "minor");
    if (major && minor) {
      long major_val = PyLong_AsLong(major);
      long minor_val = PyLong_AsLong(minor);
      python_version_ = std::to_string(major_val) + "." + std::to_string(minor_val);
      RCLCPP_INFO(logger_, "Python version: %s", python_version_.c_str());
    }
    Py_XDECREF(major);
    Py_XDECREF(minor);
    Py_DECREF(version_info);
  }
  Py_DECREF(sys_module);

  // Try to get NumPy version in a safe way
  if (PyRun_SimpleString("import sys") != 0) {
    RCLCPP_ERROR(logger_, "Failed to import sys in Python");
    return false;
  }

  // First, try to check if numpy is importable at all
  // Use exec mode for multi-line statements instead of eval
  PyRun_SimpleString(
    "try:\n"
    "    import numpy\n"
    "    __numpy_version = numpy.__version__\n"
    "    __numpy_success = True\n"
    "except Exception as e:\n"
    "    __numpy_version = str(e)\n"
    "    __numpy_success = False\n");

  // Get the results from the global namespace
  PyObject* main_module = PyImport_AddModule("__main__");
  PyObject* main_dict = PyModule_GetDict(main_module);
  PyObject* success_obj = PyDict_GetItemString(main_dict, "__numpy_success");
  PyObject* version_obj = PyDict_GetItemString(main_dict, "__numpy_version");
  
  if (!success_obj || !version_obj) {
    RCLCPP_ERROR(logger_, "Failed to get NumPy test results");
    return false;
  }

  // Extract results
  bool numpy_importable = false;
  if (PyBool_Check(success_obj)) {
    numpy_importable = (success_obj == Py_True);
  }
  
  if (PyUnicode_Check(version_obj)) {
    const char* version_str = PyUnicode_AsUTF8(version_obj);
    if (version_str) {
      numpy_version_ = std::string(version_str);
    }
  }

  if (!numpy_importable) {
    RCLCPP_ERROR(logger_, "NumPy is not importable: %s", numpy_version_.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "NumPy version: %s", numpy_version_.c_str());

  // Test basic NumPy functionality
  PyRun_SimpleString(
    "try:\n"
    "    import numpy as np\n"
    "    arr = np.array([1, 2, 3])\n"
    "    result = arr.sum()\n"
    "    __numpy_func_success = True\n"
    "except Exception as e:\n"
    "    __numpy_func_success = False\n"
    "    __numpy_func_error = str(e)\n");

  PyObject* func_success_obj = PyDict_GetItemString(main_dict, "__numpy_func_success");
  bool numpy_functional = false;
  if (func_success_obj && PyBool_Check(func_success_obj)) {
    numpy_functional = (func_success_obj == Py_True);
  }

  if (!numpy_functional) {
    RCLCPP_ERROR(logger_, "NumPy basic functionality test failed");
    return false;
  }

  RCLCPP_INFO(logger_, "NumPy compatibility check passed");
  return true;
}

bool NeuPANController::initializeNumpyWithFallback()
{
  RCLCPP_INFO(logger_, "Initializing NumPy with fallback strategies...");

  // Strategy 0: Try to work around the PyExc_RecursionError symbol issue
  RCLCPP_INFO(logger_, "Attempting to preload NumPy to resolve symbol conflicts...");
  if (PyRun_SimpleString(
    "import sys\n"
    "import os\n"
    "# Force loading of NumPy with explicit error handling\n"
    "try:\n"
    "    import numpy\n"
    "    print(f'NumPy loaded successfully: {numpy.__version__}')\n"
    "    numpy_loaded = True\n"
    "except Exception as e:\n"
    "    print(f'NumPy preload failed: {e}')\n"
    "    numpy_loaded = False\n"
    ) == 0) {
    
    // Check if preload was successful
    PyObject* main_module = PyImport_AddModule("__main__");
    PyObject* main_dict = PyModule_GetDict(main_module);
    PyObject* numpy_loaded = PyDict_GetItemString(main_dict, "numpy_loaded");
    
    if (numpy_loaded && PyBool_Check(numpy_loaded) && numpy_loaded == Py_True) {
      RCLCPP_INFO(logger_, "NumPy preload successful, skipping C API initialization");
      numpy_initialized_ = false; // Use Python-only mode
      return true;
    }
  }
  PyErr_Clear();

  // Strategy 1: Standard NumPy C API initialization
  RCLCPP_INFO(logger_, "Trying standard NumPy C API initialization...");
  if (_import_array() >= 0) {
    RCLCPP_INFO(logger_, "Standard NumPy C API initialization succeeded");
    numpy_initialized_ = true;
    return true;
  }

  RCLCPP_WARN(logger_, "Standard NumPy C API initialization failed, trying alternatives...");
  PyErr_Clear(); // Clear any errors from the failed attempt

  // Strategy 2: Try importing NumPy in Python first, then initialize C API
  RCLCPP_INFO(logger_, "Trying Python-first NumPy initialization...");
  if (PyRun_SimpleString("import numpy") == 0) {
    // Now try C API again
    if (_import_array() >= 0) {
      RCLCPP_INFO(logger_, "Python-first NumPy initialization succeeded");
      numpy_initialized_ = true;
      return true;
    }
  }
  PyErr_Clear();

  // Strategy 3: Try alternative import methods
  return tryAlternativeNumpyImport();
}

bool NeuPANController::tryAlternativeNumpyImport()
{
  RCLCPP_INFO(logger_, "Trying alternative NumPy import methods...");

  // Alternative 1: Manual import array initialization
  PyObject* numpy_module = PyImport_ImportModule("numpy.core.multiarray");
  if (numpy_module) {
    RCLCPP_INFO(logger_, "Successfully imported numpy.core.multiarray");
    Py_DECREF(numpy_module);
    
    // Try import_array again
    if (_import_array() >= 0) {
      RCLCPP_INFO(logger_, "Alternative method 1 succeeded");
      numpy_initialized_ = true;
      return true;
    }
    PyErr_Clear();
  }

  // Alternative 2: Try with numpy.core._multiarray_umath
  PyObject* umath_module = PyImport_ImportModule("numpy.core._multiarray_umath");
  if (umath_module) {
    RCLCPP_INFO(logger_, "Successfully imported numpy.core._multiarray_umath");
    Py_DECREF(umath_module);
    
    if (_import_array() >= 0) {
      RCLCPP_INFO(logger_, "Alternative method 2 succeeded");
      numpy_initialized_ = true;
      return true;
    }
    PyErr_Clear();
  }

  // Alternative 3: Skip C API and work with Python objects only
  RCLCPP_WARN(logger_, "All NumPy C API initialization methods failed");
  RCLCPP_WARN(logger_, "Falling back to Python-only NumPy operations");
  
  // Test if we can at least work with NumPy through Python
  PyObject* test_result = PyRun_String(
    "import numpy as np; np.array([1,2,3])",
    Py_eval_input, PyEval_GetGlobals(), PyEval_GetLocals());
  
  if (test_result) {
    RCLCPP_INFO(logger_, "NumPy Python-only fallback mode activated");
    Py_DECREF(test_result);
    numpy_initialized_ = false; // C API not available
    return true; // But Python interface works
  }

  RCLCPP_ERROR(logger_, "All NumPy initialization strategies failed");
  return false;
}

void NeuPANController::startAsyncPythonInitialization()
{
  python_initialization_in_progress_ = true;
  python_initialization_failed_ = false;
  
  // Create background thread for Python initialization
  python_init_thread_ = std::make_unique<std::thread>(
    &NeuPANController::pythonInitializationWorker, this);
  
  RCLCPP_INFO(logger_, "Python initialization thread started");
}

void NeuPANController::pythonInitializationWorker()
{
  RCLCPP_INFO(logger_, "Background Python initialization started...");
  
  try {
    std::lock_guard<std::mutex> lock(python_mutex_);
    
    // Perform the actual Python initialization
    bool success = initializePython();
    
    python_initialization_in_progress_ = false;
    
    if (success) {
      RCLCPP_INFO(logger_, "✅ Background Python initialization completed successfully!");
      python_initialized_ = true;
      python_initialization_failed_ = false;
    } else {
      RCLCPP_ERROR(logger_, "❌ Background Python initialization failed!");
      python_initialized_ = false;
      python_initialization_failed_ = true;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in Python initialization thread: %s", e.what());
    python_initialization_in_progress_ = false;
    python_initialization_failed_ = true;
    python_initialized_ = false;
  }
}

nav_msgs::msg::Path NeuPANController::generateVelocityBasedTrajectory(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & cmd_vel)
{
  nav_msgs::msg::Path trajectory;
  trajectory.header.frame_id = pose.header.frame_id; // Use same frame as robot pose
  trajectory.header.stamp = pose.header.stamp;
  
  // Start from current robot pose
  geometry_msgs::msg::PoseStamped current_pose = pose;
  trajectory.poses.push_back(current_pose);
  
  // Parameters for trajectory prediction
  const double dt = 0.1;  // 100ms time steps
  const double prediction_time = 3.0;  // Predict 3 seconds ahead
  const int num_steps = static_cast<int>(prediction_time / dt);
  
  // If no velocity command, just return current pose
  if (std::abs(cmd_vel.linear.x) < 1e-3 && std::abs(cmd_vel.linear.y) < 1e-3 && std::abs(cmd_vel.angular.z) < 1e-3) {
    RCLCPP_DEBUG(logger_, "Zero velocity command, returning single point trajectory");
    return trajectory;
  }
  
  // Generate trajectory based on current velocity commands
  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;
  double theta = tf2::getYaw(current_pose.pose.orientation);
  
  for (int i = 1; i <= num_steps; ++i) {
    // For omnidirectional robot (robot_type == "omni")
    if (robot_type_ == "omni") {
      // Direct velocity in global frame
      x += cmd_vel.linear.x * dt;
      y += cmd_vel.linear.y * dt;
      // For omni robots, angular velocity might be used for orientation adjustment
      theta += cmd_vel.angular.z * dt;
    } else {
      // For differential drive and ackermann robots
      // Apply motion model: x' = x + v*cos(theta)*dt, y' = y + v*sin(theta)*dt
      double v = cmd_vel.linear.x; // Forward velocity
      double omega = cmd_vel.angular.z; // Angular velocity
      
      x += v * cos(theta) * dt;
      y += v * sin(theta) * dt;
      theta += omega * dt;
    }
    
    // Normalize theta to [-pi, pi]
    theta = std::atan2(sin(theta), cos(theta));
    
    // Create pose for this step
    geometry_msgs::msg::PoseStamped future_pose;
    future_pose.header = current_pose.header;
    future_pose.pose.position.x = x;
    future_pose.pose.position.y = y;
    future_pose.pose.position.z = 0.0;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);
    future_pose.pose.orientation = tf2::toMsg(quat);
    
    trajectory.poses.push_back(future_pose);
  }
  
  RCLCPP_DEBUG(logger_, "Generated velocity-based trajectory with %zu poses (%.1fs prediction)", 
    trajectory.poses.size(), prediction_time);
  
  return trajectory;
}

nav_msgs::msg::Path NeuPANController::generateLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = costmap_ros_->getBaseFrameID(); // Use base frame for local plan
  local_plan.header.stamp = pose.header.stamp;
  
  if (global_plan_.poses.empty()) {
    RCLCPP_DEBUG(logger_, "No global plan available for local plan generation");
    return local_plan;
  }
  
  try {
    // Transform robot pose to global plan frame
    geometry_msgs::msg::PoseStamped robot_pose_in_plan_frame;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose_in_plan_frame)) {
      RCLCPP_WARN(logger_, "Failed to transform robot pose to global plan frame");
      return local_plan;
    }
    
    // Find the closest point on the global plan
    size_t closest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
      double distance = std::hypot(
        global_plan_.poses[i].pose.position.x - robot_pose_in_plan_frame.pose.position.x,
        global_plan_.poses[i].pose.position.y - robot_pose_in_plan_frame.pose.position.y
      );
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = i;
      }
    }
    
    // Extract local segment from global plan (similar to pb_omni approach)
    double local_plan_distance = 5.0; // 5 meters lookahead
    double accumulated_distance = 0.0;
    
    for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
      // Transform each pose to robot base frame
      geometry_msgs::msg::PoseStamped global_pose = global_plan_.poses[i];
      geometry_msgs::msg::PoseStamped local_pose;
      
      if (transformPose(costmap_ros_->getBaseFrameID(), global_pose, local_pose)) {
        local_pose.pose.position.z = 0.0; // Ensure 2D
        local_plan.poses.push_back(local_pose);
        
        // Calculate accumulated distance
        if (i > closest_idx) {
          accumulated_distance += std::hypot(
            global_plan_.poses[i].pose.position.x - global_plan_.poses[i-1].pose.position.x,
            global_plan_.poses[i].pose.position.y - global_plan_.poses[i-1].pose.position.y
          );
          
          if (accumulated_distance > local_plan_distance) {
            break;
          }
        }
      }
    }
    
    RCLCPP_DEBUG(logger_, "Generated local plan with %zu poses", local_plan.poses.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "Exception in generateLocalPlan: %s", e.what());
  }
  
  return local_plan;
}

bool NeuPANController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & input_pose,
  geometry_msgs::msg::PoseStamped & output_pose)
{
  if (input_pose.header.frame_id == target_frame) {
    output_pose = input_pose;
    return true;
  }
  
  try {
    auto timeout = tf2::durationFromSec(0.1);
    output_pose = tf_->transform(input_pose, target_frame, timeout);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *node_.lock()->get_clock(), 1000,
      "Could not transform pose from %s to %s: %s",
      input_pose.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return false;
  }
}

geometry_msgs::msg::TwistStamped NeuPANController::computeSimpleFallbackVelocity(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & global_plan)
{
  auto cmd = geometry_msgs::msg::TwistStamped();
  cmd.header.stamp = pose.header.stamp;
  cmd.header.frame_id = pose.header.frame_id;
  
  if (global_plan.poses.empty()) {
    RCLCPP_DEBUG(logger_, "No global plan for fallback controller");
    return cmd; // Return zero velocity
  }
  
  // Simple goal-seeking behavior: move towards the next waypoint
  // Find the closest waypoint that's ahead of us
  geometry_msgs::msg::PoseStamped target_pose;
  bool found_target = false;
  
  double min_distance = std::numeric_limits<double>::max();
  size_t best_idx = 0;
  
  // Find closest point first
  for (size_t i = 0; i < global_plan.poses.size(); ++i) {
    double distance = std::hypot(
      global_plan.poses[i].pose.position.x - pose.pose.position.x,
      global_plan.poses[i].pose.position.y - pose.pose.position.y
    );
    
    if (distance < min_distance) {
      min_distance = distance;
      best_idx = i;
    }
  }
  
  // Look ahead from the closest point
  double lookahead_distance = 1.0; // 1 meter lookahead
  for (size_t i = best_idx; i < global_plan.poses.size(); ++i) {
    double distance = std::hypot(
      global_plan.poses[i].pose.position.x - pose.pose.position.x,
      global_plan.poses[i].pose.position.y - pose.pose.position.y
    );
    
    if (distance >= lookahead_distance) {
      target_pose = global_plan.poses[i];
      found_target = true;
      break;
    }
  }
  
  // If no lookahead target found, use the goal
  if (!found_target && !global_plan.poses.empty()) {
    target_pose = global_plan.poses.back();
    found_target = true;
  }
  
  if (!found_target) {
    return cmd; // Return zero velocity
  }
  
  // Calculate direction to target
  double dx = target_pose.pose.position.x - pose.pose.position.x;
  double dy = target_pose.pose.position.y - pose.pose.position.y;
  double distance_to_target = std::hypot(dx, dy);
  
  if (distance_to_target < 0.1) {
    return cmd; // Very close to target, stop
  }
  
  // Current robot orientation
  double robot_yaw = tf2::getYaw(pose.pose.orientation);
  double target_yaw = std::atan2(dy, dx);
  double yaw_error = target_yaw - robot_yaw;
  
  // Normalize angle to [-pi, pi]
  while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2 * M_PI;
  
  // Simple proportional controller
  double max_linear_speed = std::min(max_linear_velocity_, 0.3); // Conservative speed
  double max_angular_speed = std::min(max_angular_velocity_, 0.8);
  
  if (robot_type_ == "omni") {
    // Omnidirectional: direct motion toward target
    double speed_factor = std::min(1.0, distance_to_target / 2.0); // Slow down when close
    cmd.twist.linear.x = (dx / distance_to_target) * max_linear_speed * speed_factor;
    cmd.twist.linear.y = (dy / distance_to_target) * max_linear_speed * speed_factor;
    cmd.twist.angular.z = yaw_error * 0.5; // Gentle orientation adjustment
  } else {
    // Differential drive: turn first, then move
    if (std::abs(yaw_error) > 0.2) {
      // Need to turn significantly
      cmd.twist.linear.x = 0.1; // Slow forward motion while turning
      cmd.twist.angular.z = std::clamp(yaw_error * 1.0, -max_angular_speed, max_angular_speed);
    } else {
      // Mostly aligned, move forward
      double speed_factor = std::min(1.0, distance_to_target / 2.0);
      cmd.twist.linear.x = max_linear_speed * speed_factor;
      cmd.twist.angular.z = yaw_error * 0.5; // Fine angular adjustment
    }
    cmd.twist.linear.y = 0.0; // No lateral motion for differential drive
  }
  
  // Apply velocity limits
  cmd.twist.linear.x = std::clamp(cmd.twist.linear.x, -max_linear_velocity_, max_linear_velocity_);
  cmd.twist.linear.y = std::clamp(cmd.twist.linear.y, -max_linear_velocity_, max_linear_velocity_);
  cmd.twist.angular.z = std::clamp(cmd.twist.angular.z, -max_angular_velocity_, max_angular_velocity_);
  
  RCLCPP_DEBUG_THROTTLE(logger_, *node_.lock()->get_clock(), 2000,
    "🔄 Fallback controller: target distance=%.2f, yaw_error=%.2f, vel=[%.2f, %.2f, %.2f]",
    distance_to_target, yaw_error, cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z);
  
  return cmd;
}

void NeuPANController::publishLocalPlan(
  const geometry_msgs::msg::PoseStamped & pose,
  const nav_msgs::msg::Path & local_path)
{
  if (!local_plan_pub_) {
    return;
  }

  // Always publish local plan for visualization (even if no subscribers)
  local_plan_pub_->publish(local_path);
  RCLCPP_DEBUG(logger_, "Published local plan with %zu waypoints", local_path.poses.size());
}

}  // namespace neupan_nav2_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(neupan_nav2_controller::NeuPANController, nav2_core::Controller)
