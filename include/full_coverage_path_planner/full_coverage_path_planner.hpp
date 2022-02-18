//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
/** include the libraries you need in your planner here */
/** for global path planner interface */
#pragma once

#include <angles/angles.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/node_utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <fstream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/common.hpp"
#include "full_coverage_path_planner/curve_generator.hpp"
#include "rclcpp/rclcpp.hpp"

using std::string;

#ifndef dabs
#define dabs(a) ((a) >= 0 ? (a) : -(a))
#endif
#ifndef dmin
#define dmin(a, b) ((a) <= (b) ? (a) : (b))
#endif
#ifndef dmax
#define dmax(a, b) ((a) >= (b) ? (a) : (b))
#endif
#ifndef clamp
#define clamp(a, lower, upper) dmax(dmin(a, upper), lower)
#endif

namespace full_coverage_path_planner
{

enum dir
{
  none = 0,
  right = 1,
  up = 2,
  left = -1,
  down = -2,
};

class FullCoveragePathPlanner
{
public:
  /**
   * @brief Default constructor for the NavFnROS object
   */
  FullCoveragePathPlanner();
  // FullCoveragePathPlanner(std::string name, costmap_2d::Costmap2DROS*
  // costmap_ros);

  ~FullCoveragePathPlanner() {}

  /**
   * @brief Publish a path for visualization purposes
   */
  void publishPlan(const std::vector<geometry_msgs::msg::PoseStamped> & path);

  // Cost for checking coverage. Perhaps define this in coverage costmap
  // plugin?
  static const unsigned char COVERAGE_COST = 65;

protected:
  /**
   * @brief Convert internal representation of a path to a ROS path
   * @param start Start pose of robot
   * @param goalpoints Goal points from Spiral Algorithm
   * @param plan Output plan variable
   */
  void parsePointlist2Plan(
    const geometry_msgs::msg::PoseStamped & start,
    std::list<Point_t> const & goalpoints,
    std::vector<geometry_msgs::msg::PoseStamped> & plan);

  /**
   * @brief Convert internal representation of a path to a vector of paths
   * @param path robot plan containing list of poses
   * @param enable_smoothing if true sections are stiched smoothly avoiding
   * turning in place
   * @param max_path_resolution Maximum desired resolution for smooth path
   * @param grid_size Grid size of original plan
   * @param path_vector Output path vector
   */
  void convertPlanToPathVector(
    const std::vector<geometry_msgs::msg::PoseStamped> & plan,
    const bool enable_smoothing, const double max_path_resolution,
    const double grid_size, std::vector<nav_msgs::msg::Path> & path_vector);

  /**
   * @brief Convert ROS Occupancy grid to internal grid representation, given
   * the size of a single tile
   * @param cpp_grid_ ROS occupancy grid representation. Cells higher that 65
   * are considered occupied
   * @param grid Internal map representation
   * @param grid_size Size (in meters) of a cell. This can be the robot's size
   * @param real_start Start position of the robot (in meters)
   * @param scaled_start Start position of the robot on the grid
   * @param yaw_start Start orientation of the robot (in radians)
   * @return If parsing the grid has succeeded, return true, otherwise false
   */
  bool parseGrid(
    nav2_costmap_2d::Costmap2D const * cpp_costmap,
    std::vector<std::vector<bool>> & grid, double grid_size,
    geometry_msgs::msg::PoseStamped const & real_start,
    Point_t & scaled_start, double & yaw_start);

  /**
   * @brief Create Quaternion from Yaw
   * @param yaw Orientation
   * @return Quaternion with desired yaw orientation
   */

  auto createQuaternionMsgFromYaw(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  }

  // Enumeration of possible rotate directions of a manoeuvre
  // When eAnyDirection is used, the shortest rotation is preferred.
  // If both directions are equal in length, counter-clockwise is the default
  enum eRotateDirection { eClockwise, eAnyDirection, eCounterClockwise };

  nav2_util::LifecycleNode::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  double vehicle_width_, plan_resolution_, tile_size_;
  int division_factor_, manoeuvre_resolution_, max_overlap_forward_,
    max_overlap_turn_;
  dPoint_t grid_origin_;
  bool initialized_;
  geometry_msgs::msg::PoseStamped previous_goal_;
  double previous_orientation_;
  std::string name_, global_frame_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::vector<eRotateDirection> turn_around_directions_;

  struct spiral_cpp_metrics_type
  {
    int visited_counter;
    int multiple_pass_counter;
    int accessible_counter;
  };
  spiral_cpp_metrics_type spiral_cpp_metrics_;
};

/**
 * Sort function for sorting Points on distance to a POI
 */
struct ComparatorForPointSort
{
  explicit ComparatorForPointSort(Point_t poi)
  : _poi(poi) {}

  bool operator()(const Point_t & first, const Point_t & second) const
  {
    return distanceSquared(first, _poi) < distanceSquared(second, _poi);
  }

private:
  Point_t _poi;
};

}  // namespace full_coverage_path_planner
