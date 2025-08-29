#ifndef NEUPAN_NAV2_CONTROLLER__NEUPAN_CONTROLLER_HPP_
#define NEUPAN_NAV2_CONTROLLER__NEUPAN_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

// Python C API
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

namespace neupan_nav2_controller
{

/**
 * @class NeuPANController
 * @brief Nav2 controller plugin that wraps the NeuPAN neural path planning algorithm
 */
class NeuPANController : public nav2_core::Controller
{
public:
  /**
   * @brief Constructor
   */
  NeuPANController() = default;

  /**
   * @brief Destructor
   */
  ~NeuPANController() override;

  /**
   * @brief Configure controller on bringup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller on shutdown
   */
  void cleanup() override;

  /**
   * @brief Activate controller
   */
  void activate() override;

  /**
   * @brief Deactivate controller
   */
  void deactivate() override;

  /**
   * @brief Set the plan to track
   * @param path The global plan to track
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief Compute velocity commands for the robot
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Goal checker object
   * @return Velocity command
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief Set the speed limit
   * @param speed_limit Speed limit to set
   * @param percentage If true, speed limit is percentage from 0 to 1.0
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief Initialize Python interpreter and NeuPAN module
   */
  bool initializePython();

  /**
   * @brief Cleanup Python resources
   */
  void cleanupPython();

  /**
   * @brief Call NeuPAN planner from Python
   * @param robot_state Current robot state (x, y, theta)
   * @param obstacle_points Obstacle points from laser scan
   * @param cmd_vel Output velocity command
   * @return True if planning succeeded
   */
  bool callNeuPANPlanner(
    const std::vector<double> & robot_state,
    const std::vector<std::vector<double>> & obstacle_points,
    geometry_msgs::msg::Twist & cmd_vel);

  /**
   * @brief Convert laser scan to obstacle points
   * @param scan Laser scan message
   * @param robot_pose Current robot pose
   * @return Vector of obstacle points in global frame
   */
  std::vector<std::vector<double>> laserScanToObstaclePoints(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    const geometry_msgs::msg::PoseStamped & robot_pose);

  /**
   * @brief Get latest laser scan
   * @return Latest laser scan message
   */
  sensor_msgs::msg::LaserScan::ConstSharedPtr getLatestLaserScan();

  /**
   * @brief Convert Nav2 path to NeuPAN initial path format
   * @param path Nav2 path message
   */
  void convertNav2PathToNeuPAN(const nav_msgs::msg::Path & path);

  // Node and plugin information
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Current plan
  nav_msgs::msg::Path global_plan_;

  // Python integration
  PyObject * neupan_core_instance_;
  bool python_initialized_;

  // Laser scan subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_laser_scan_;

  // Parameters
  double max_linear_velocity_;
  double max_angular_velocity_;
  double goal_tolerance_;
  std::string robot_type_;
  std::string laser_topic_;
  std::string dune_model_path_;
  std::string neupan_config_path_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("NeuPANController")};
};

}  // namespace neupan_nav2_controller

#endif  // NEUPAN_NAV2_CONTROLLER__NEUPAN_CONTROLLER_HPP_