//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <fstream>
#include <list>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>

using std::string;

#ifndef FULL_COVERAGE_PATH_PLANNER_FULL_COVERAGE_PATH_PLANNER_H
#define FULL_COVERAGE_PATH_PLANNER_FULL_COVERAGE_PATH_PLANNER_H

#include "full_coverage_path_planner/common.h"

// #define DEBUG_PLOT

#ifndef dabs
#define dabs(a)     ((a) >= 0 ? (a):-(a))
#endif
#ifndef dmin
#define dmin(a, b)   ((a) <= (b) ? (a):(b))
#endif
#ifndef dmax
#define dmax(a, b)   ((a) >= (b) ? (a):(b))
#endif
#ifndef clamp
#define clamp(a, lower, upper)    dmax(dmin(a, upper), lower)
#endif

enum
{
  eDirNone = 0,
  eDirRight = 1,
  eDirUp = 2,
  eDirLeft = -1,
  eDirDown = -2,
};

namespace full_coverage_path_planner
{
class FullCoveragePathPlanner
{
public:
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  FullCoveragePathPlanner();
  FullCoveragePathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Publish a path for visualization purposes
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

  ~FullCoveragePathPlanner()
  {
  }

  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) = 0;

protected:
  /**
   * Convert internal representation of a to a ROS path
   * @param start Start pose of robot
   * @param goalpoints Goal points from Spiral Algorithm
   * @param plan  Output plan variable
   */
  void parsePointlist2Plan(const geometry_msgs::PoseStamped& start, std::list<Point_t> const& goalpoints,
                           std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * Convert ROS Occupancy grid to internal grid representation, given the size of a single tile
   * @param cpp_grid_ ROS occupancy grid representation. Cells higher that 65 are considered occupied
   * @param grid internal map representation
   * @param tileSize size (in meters) of a cell. This can be the robot's size
   * @param realStart Start position of the robot (in meters)
   * @param scaledStart Start position of the robot on the grid
   * @return success
   */
  bool parseGrid(nav_msgs::OccupancyGrid const& cpp_grid_,
                 std::vector<std::vector<bool> >& grid,
                 float robotRadius,
                 float toolRadius,
                 geometry_msgs::PoseStamped const& realStart,
                 Point_t& scaledStart);
  ros::Publisher plan_pub_;
  ros::ServiceClient cpp_grid_client_;
  nav_msgs::OccupancyGrid cpp_grid_;
  float robot_radius_;
  float tool_radius_;
  float plan_resolution_;
  float tile_size_;
  fPoint_t grid_origin_;
  bool initialized_;
  geometry_msgs::PoseStamped previous_goal_;

  struct spiral_cpp_metrics_type
  {
    int visited_counter;
    int multiple_pass_counter;
    int accessible_counter;
    double total_area_covered;
  };
  spiral_cpp_metrics_type spiral_cpp_metrics_;
};


/**
 * Sort function for sorting Points on distance to a POI
 */
struct ComparatorForPointSort
{
  explicit ComparatorForPointSort(Point_t poi) : _poi(poi)
  {
  }

  bool operator()(const Point_t& first, const Point_t& second) const
  {
    return distanceSquared(first, _poi) < distanceSquared(second, _poi);
  }

private:
  Point_t _poi;
};
}  // namespace full_coverage_path_planner
#endif  // FULL_COVERAGE_PATH_PLANNER_FULL_COVERAGE_PATH_PLANNER_H
