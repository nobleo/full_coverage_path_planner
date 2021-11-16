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
#include "visualization_msgs/msg/marker.hpp"

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
     * @brief Creating a plan from a start pose to a goal pose
     * @param start Start pose
     * @param goal Goal pose
     * @return nav_msgs::Path of the generated path
     */
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;


    visualization_msgs::msg::Marker cubeMarker(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b);

    visualization_msgs::msg::Marker lineStrip(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b);

    void visualizeGrid(std::vector<std::vector<bool>> const &grid, std::string name_space, float a, float r, float g, float b);

    void visualizeGridlines();

    void visualizeSpiral(std::list<gridNode_t> &spiral_nodes, std::string name_space, float w, float a, float r, float g, float b);

  protected:

    // Temporary for debugging or testing purposes
    int max_overlap;
    int max_overlap_forward = 0; // Maximum allowable overlapping grids between a forward menoeuvre and already visited grids
    int max_overlap_turn = 4; // Maximum allowable overlapping grids between a turning menoeuvre and already visited grids
    int spiral_counter = 0; // Limit the amount of spirals planned
    std::vector<std::vector<bool>> visited_copy; // Only mark the spots covered by spirals (not A*)

    // Absolute manoeuvre footprints (in the vehicle's frame)
    std::vector<nav2_costmap_2d::MapLocation> left_turn;
    std::vector<nav2_costmap_2d::MapLocation> forward;
    std::vector<nav2_costmap_2d::MapLocation> right_turn;

    // Relative manoeuvre footprints (in the vehicle's frame)
    std::vector<Point_t> left_turn_rel;
    std::vector<Point_t> forward_rel;
    std::vector<Point_t> right_turn_rel;

    // Local costmap objects for the planner to store the grid and manipulate footprints
    nav2_costmap_2d::Costmap2D coarse_grid;
    nav2_costmap_2d::Costmap2DROS *coarse_grid_ros;

    // Publishers for the planner output
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> grid_pub;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> spirals_pub;

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
     * @param yawStart start orientation
     * @param visited all the nodes visited by the spiral
     * @return list of nodes that form the spiral
     */
    std::list<gridNode_t> spiral(std::vector<std::vector<bool>> const &grid, std::list<gridNode_t> &init,
                                 double &yaw_start, std::vector<std::vector<bool>> &visited);

    /**
     * Perform Spiral-STC (Spanning Tree Coverage) coverage path planning.
     * In essence, the robot moves forward until an obstacle or visited node is met, then turns right (making a spiral)
     * When stuck in the middle of the spiral, use A* to get out again and start a new spiral, until a* can't find a path to uncovered cells
     * @param grid
     * @param init
     * @return
     */
    std::list<Point_t> spiral_stc(std::vector<std::vector<bool>> const &grid, Point_t &init, double &yaw_start,
                                  int &multiple_pass_counter, int &visited_counter);

    /**
     * Compute the cells in a grid that lie below a convex footprint
     * @param x_m the x coordinate of the map location around which the footprint is defined
     * @param y_m the y coordinate of the map location around which the footprint is defined
     * @param yaw the orientation of footprint
     * @param footprint_cells the output vector of map locations what are covered by the footprint
     * @return a boolean that indicates if the entire footprint lies inside the map boundaries
     */
    bool computeFootprintCells(int &x_m, int &y_m, double &yaw, std::string part, std::vector<nav2_costmap_2d::MapLocation> &footprint_cells);

    /**
     * Compute the cells in a grid make up the swept path of a manoeuvre build from footprints
     * @param x1 the x coordinate of the map location around which the initial footprint is defined
     * @param y1 the y coordinate of the map location around which the initial footprint is defined
     * @param x1 the x coordinate of the map location around which the final footprint is defined
     * @param y1 the y coordinate of the map location around which the final footprint is defined
     * @param yaw1 the starting orientation of the manoeuvre
     * @param man_grids the output vector of map locations that are covered by the manoeuvre
     * @return a boolean that indicates if the entire manoeuvre lies inside the map boundaries
     */
    bool computeManoeuvreFootprint(int &x1, int &y1, int &x2, int &y2, double &yaw1, std::string part, std::vector<nav2_costmap_2d::MapLocation> &man_grids);

    /**
     * Rotate a point on the grid around another point on the grid via conversion to world coordinates
     * @param poi_x the x coordinate of the point of interest that will rotate
     * @param poi_y the y coordinate of the point of interest that will rotate
     * @param icr_x the x coordinate of the instant center of rotation
     * @param icr_y the y coordinate of the instant center of rotation
     * @param yaw the angle with which the poi will rotate around the icr
     * @return the rotated point containing the x and y indices of the grid cell
     */
    Point_t rotatePoint(int poi_x, int poi_y, int &icr_x, int &icr_y, double &yaw);

    /**
     * Check if a cell lies within all of the map boundaries
     * @param x the x index of the cell of interest
     * @param y the y index of the cell of interest
     * @param x_max the maximum x index that lies inside the grid map
     * @param y_max the maximum y index that lies inside the grid map
     * @return a boolean that is true when the cell of interest lies inside the map boundaries
     */
    bool checkMapBounds(int x, int y, int &x_max, int &y_max);
  };
}  // namespace full_coverage_path_planner
