//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
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
#include <fstream>

using std::string;

#ifndef FULL_COVERAGE_PATH_PLANNER_SPIRAL_STC_H
#define FULL_COVERAGE_PATH_PLANNER_SPIRAL_STC_H

#include "full_coverage_path_planner/full_coverage_path_planner.h"
namespace full_coverage_path_planner
{
class SpiralSTC : public nav_core::BaseGlobalPlanner, private full_coverage_path_planner::FullCoveragePathPlanner
{
public:
  /**
   * Find a path that spirals inwards from init until an obstacle is seen in the grid
   * @param grid 2D grid of bools. true == occupied/blocked/obstacle
   * @param init start position
   * @param visited all the nodes visited by the spiral
   * @return list of nodes that form the spiral
   */
  static std::list<gridNode_t> spiral(std::vector<std::vector<bool> > const &grid, std::list<gridNode_t> &init,
                                      std::vector<std::vector<bool> > &visited);

  /**
   * Perform Spiral-STC (Spanning Tree Coverage) coverage path planning.
   * In essence, the robot moves forward until an obstacle or visited node is met, then turns right (making a spiral)
   * When stuck in the middle of the spiral, use A* to get out again and start a new spiral, until a* can't find a path to uncovered cells
   * @param grid
   * @param init
   * @return
   */
  static std::list<Point_t> spiral_stc(std::vector<std::vector<bool> > const &grid,
                                        Point_t &init,
                                        int &multiple_pass_counter,
                                        int &visited_counter);

private:
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief  Initialization function for the FullCoveragePathPlanner object
   * @param  name The name of this planner
   * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
};

}  // namespace full_coverage_path_planner
#endif  // FULL_COVERAGE_PATH_PLANNER_SPIRAL_STC_H
