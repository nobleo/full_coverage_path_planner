//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#pragma once

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <list>
#include <memory>
#include <string>
#include <vector>

// #include <pluginlib/class_list_macros.h>
#include "full_coverage_path_planner/full_coverage_path_planner.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include <angles/angles.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_map.hpp>

using namespace std::chrono_literals;
using std::string;
namespace full_coverage_path_planner
{
  class SpiralSTC : public nav2_core::GlobalPlanner , private full_coverage_path_planner::FullCoveragePathPlanner
  {
  public:
    /**
   * @brief constructor
   */
    SpiralSTC();

    /**
   * @brief destructor
   */
    ~SpiralSTC();

    /**
     * @brief Configuring plugin
     * @param parent Lifecycle node pointer
     * @param name Name of plugin map
     * @param tf Shared ptr of TF2 buffer
     * @param costmap_ros Costmap2DROS object
    */
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    /**
     * @brief Cleanup lifecycle node
     */
    void cleanup() override;

    /**
     * @brief Activate lifecycle node
     */
    void activate() override;

    /**
     * @brief Deactivate lifecycle node
     */
    void deactivate() override;

    /**
     * @brief Creating a plan from start and goal poses
     * @param start Start pose
     * @param goal Goal pose
     * @return nav_msgs::Path of the generated path
     */
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;

  protected:
    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start pose
     * @param goal The goal pose
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
                  std::vector<geometry_msgs::msg::PoseStamped> &plan);

    /**
     * @brief  Initialization function for the FullCoveragePathPlanner object
     * @param  name The name of this planner
     * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, nav2_costmap_2d::Costmap2DROS *costmap_ros);

    /**
     * Find a path that spirals inwards from init until an obstacle is seen in the grid
     * @param grid 2D grid of bools. true == occupied/blocked/obstacle
     * @param init start position
     * @param visited all the nodes visited by the spiral
     * @return list of nodes that form the spiral
     */
    static std::list<gridNode_t> spiral(std::vector<std::vector<bool>> const &grid, std::list<gridNode_t> &init,
                                        std::vector<std::vector<bool>> &visited);

    /**
     * Perform Spiral-STC (Spanning Tree Coverage) coverage path planning.
     * In essence, the robot moves forward until an obstacle or visited node is met, then turns right (making a spiral)
     * When stuck in the middle of the spiral, use A* to get out again and start a new spiral, until a* can't find a path to uncovered cells
     * @param grid
     * @param init
     * @return
     */
    static std::list<Point_t> spiral_stc(std::vector<std::vector<bool>> const &grid,
                                         Point_t &init,
                                         int &multiple_pass_counter,
                                         int &visited_counter);
  };

}  // namespace full_coverage_path_planner
