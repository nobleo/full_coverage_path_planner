//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/spiral_stc.hpp"

using nav2_util::declare_parameter_if_not_declared;
namespace full_coverage_path_planner
{
  SpiralSTC::SpiralSTC()
  {
  }

  SpiralSTC::~SpiralSTC()
  {
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Destroying plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
  }

  void SpiralSTC::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    if (!initialized_)
    {
      // Get node from parent
      node_ = parent.lock();
      name_ = name;

      // Currently this plugin does not use the costmap, instead request a map from a server
      // This will change in the future
      costmap_ = costmap_ros->getCostmap();
      coarse_grid_ros_= costmap_ros.get();
      global_frame_ = costmap_ros->getGlobalFrameID();

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),
                  "Configuring plugin %s of type NavfnPlanner", name_.c_str());

      // Create publishers to visualize the plan
      plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
      vis_pub_grid_ = node_->create_publisher<visualization_msgs::msg::Marker>("gridOverlay", 0);
      vis_pub_spirals_ = node_->create_publisher<visualization_msgs::msg::Marker>("spirals", 0);

      // Define  robot radius (radius) parameter
      double robot_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".robot_radius", rclcpp::ParameterValue(robot_radius_default));
      node_->get_parameter(name_ + ".robot_radius", robot_radius_);
      // Define  tool radius (radius) parameter
      double tool_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".tool_radius", rclcpp::ParameterValue(tool_radius_default));
      node_->get_parameter(name_ + ".tool_radius", tool_radius_);
      initialized_ = true;
    }
  }

  void SpiralSTC::activate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Activating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    vis_pub_grid_->on_activate();
    vis_pub_spirals_->on_activate();
  }

  void SpiralSTC::deactivate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Deactivating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    vis_pub_grid_->on_deactivate();
    vis_pub_spirals_->on_deactivate();
  }

  void SpiralSTC::cleanup()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Cleaning up plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    // TODO(CesarLopez) Add proper cleanup
  }

  nav_msgs::msg::Path SpiralSTC::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path global_path;
    SpiralSTC::makePlan(start, goal, global_path.poses);

    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    return global_path;
  }

  std::list<gridNode_t> SpiralSTC::spiral(std::vector<std::vector<bool>> const &grid, std::list<gridNode_t> &init,
                                          double &yaw_start, std::vector<std::vector<bool>> &visited)
  {
    std::list<gridNode_t> pathNodes(init); // Copy incoming list to pathNodes
    std::list<gridNode_t>::iterator it = --(pathNodes.end()); // Create iterator and let it point to the last element of end
    if (pathNodes.size() > 1) // If list is length 1, keep iterator at end
    {
      it--;                   // Let iterator point to second to last element
    }

    gridNode_t prev = *(it);

    if (pathNodes.size() > 1)
    {
      int dx = pathNodes.back().pos.x - prev.pos.x;
      int dy = pathNodes.back().pos.y - prev.pos.y;
      yaw_start = std::atan2(dy,dx); // Overwrite input yaw_start one pathNodes is larger than 1
    }

    // Mark initial footprint as visited
    gridNode_t spiral_start = pathNodes.back();
    std::vector<nav2_costmap_2d::MapLocation> init_cells;
    if (!FootprintCells(spiral_start.pos.x, spiral_start.pos.y, yaw_start, init_cells))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Starting footprint seems to be out of bounds!");
    }
    for (auto const cell : init_cells)
    {
      visited[cell.y][cell.x] = eNodeVisited;
    }

    std::vector<nav2_costmap_2d::MapLocation> man_grids;
    bool done = false;
    while (!done)
    {
      // Initialize spiral direction towards robot's y-axis
      int dx = 0;
      int dy = 1;
      int dx_prev;
      double yaw = yaw_start;
      if (it != pathNodes.begin())
      {
        // Turn counter-clockwise
        dx = pathNodes.back().pos.x - prev.pos.x;
        dy = pathNodes.back().pos.y - prev.pos.y;
        yaw = std::atan2(dy,dx);
        dx_prev = dx;
        dx = -dy;
        dy = dx_prev;
      }
      done = true; // This condition might change in the loop below
      // Loop over the three possible directions: left, forward, right (directions taken before counter-clockwise turn)
      for (size_t i = 0; i < 3; ++i)
      {
        int x1 = pathNodes.back().pos.x;
        int y1 = pathNodes.back().pos.y;
        int x2 = pathNodes.back().pos.x + dx;
        int y2 = pathNodes.back().pos.y + dy;

        // Compute grids to be covered by the manoeuvre and check if they are in map bounds
        man_grids.clear();

        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoeuvre from (x=%d, y=%d, yaw=%f) to (x=%d, y=%d, yaw=?)", x1, y1, yaw, x2, y2);

        bool man_is_free = true; // This condition might change in the loop below

        // Cast double outputs of cosine and sine to integers
        int cos_yaw = cos(yaw);
        int sin_yaw = sin(yaw);
        int x_max = coarse_grid_.getSizeInCellsX() - 1;
        int y_max = coarse_grid_.getSizeInCellsY() - 1;

        // Apply rotation to the relative manoeuvre cells to convert from robot frame to world frame
        // TODO(Aron): Make a function out of the rotation, including out of bounds check
        if (i == 0) // Relative left turn manoeuvre
        {
          man_grids.resize(left_turn_.size());
          for (uint i = 0; i < left_turn_.size(); i++)
          {
            int x = x1 + cos_yaw*left_turn_[i].x - sin_yaw*left_turn_[i].y;
            int y = y1 + sin_yaw*left_turn_[i].x + cos_yaw*left_turn_[i].y;
            if (x < 0 || y < 0 || x > x_max || y > y_max)
            {
              RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoevre out of bounds, looking in other directions...");
              man_is_free = false;
              break;
            }
            else
            {
              man_grids[i].x = x;
              man_grids[i].y = y;
            }
          }
        }
        else if (i == 1) // Relative forward manoeuvre
        {
          man_grids.resize(forward_.size());
          for (uint i = 0; i < forward_.size(); i++)
          {
            int x = x1 + cos_yaw*forward_[i].x - sin_yaw*forward_[i].y;
            int y = y1 + sin_yaw*forward_[i].x + cos_yaw*forward_[i].y;
            if (x < 0 || y < 0 || x > x_max || y > y_max)
            {
              RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoevre out of bounds, looking in other directions...");
              man_is_free = false;
              break;
            }
            else
            {
              man_grids[i].x = x;
              man_grids[i].y = y;
            }
          }
        }
        else if (i == 2) // Relative right turn manoeuvre
        {
          man_grids.resize(right_turn_.size());
          for (uint i = 0; i < right_turn_.size(); i++)
          {
            int x = x1 + cos_yaw*right_turn_[i].x - sin_yaw*right_turn_[i].y;
            int y = y1 + sin_yaw*right_turn_[i].x + cos_yaw*right_turn_[i].y;
            if (x < 0 || y < 0 || x > x_max || y > y_max)
            {
              RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoevre out of bounds, looking in other directions...");
              man_is_free = false;
              break;
            }
            else
            {
              man_grids[i].x = x;
              man_grids[i].y = y;
            }
          }
        }

        int overlap = 0; // Keep track of the overlap with already visited grids
        for (const auto man_grid : man_grids)
        {
          if (grid[man_grid.y][man_grid.x] == eNodeVisited && visited[man_grid.y][man_grid.x] == eNodeVisited)
          {
            man_is_free = false;
            break;
          }
          else if (grid[man_grid.y][man_grid.x] == eNodeOpen && visited[man_grid.y][man_grid.x] == eNodeVisited)
          {
            overlap++;
          }
        }
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "--> with size=%lu of which %d are overlapping", man_grids.size(), overlap);
        if (man_is_free && overlap <= max_overlap_)
        {
          Point_t new_point = {x2, y2};
          gridNode_t new_node = {new_point, 0, 0};
          prev = pathNodes.back();
          pathNodes.push_back(new_node);
          it = --(pathNodes.end());
          for (const auto man_grid : man_grids)
          {
            visited[man_grid.y][man_grid.x] = eNodeVisited;
          }
          done = false;
          break;
        }
        // Try next direction clockwise
        dx_prev = dx;
        dx = dy;
        dy = -dx_prev;
      }
    }
    return pathNodes;
  }

  std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool>> const &grid,
                                           Point_t &init,
                                           double &yaw_start,
                                           int &multiple_pass_counter,
                                           int &visited_counter)
  {
    multiple_pass_counter = 0; // Initial node is initially set as visited so it does not count
    visited_counter = 0; // TODO(Aron): currently these counters don't work properly due to the footprint

    std::vector<std::vector<bool>> visited;
    visited = grid; // Copy grid matrix
    int init_x = init.x;
    int init_y = init.y;
    Point_t new_point = {init_x, init_y};
    gridNode_t new_node =
        {
            new_point, // Point: x,y
            0,         // Cost
            0,         // Heuristic
        };
    std::list<gridNode_t> pathNodes;
    std::list<Point_t> fullPath;
    pathNodes.push_back(new_node);

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "!!!!!! Starting a spiral from (x=%d, y=%d, yaw=%f) !!!!!!", init_x, init_y, yaw_start);
    pathNodes = spiral(grid, pathNodes, yaw_start, visited); // First spiral fill

    visualizeSpirals(pathNodes, "first_spiral", 0.2, 0.5, 0.0, 0.6, 0.0);

    std::list<Point_t> goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goalpoints

    for (const auto pathNode : pathNodes) // Add points to full path
    {
      Point_t newPoint = {pathNode.pos.x, pathNode.pos.y};
      visited_counter++;
      fullPath.push_back(newPoint);
    }

    while (goals.size() != 0)
    {
      // Remove all elements from pathNodes list except last element
      // The last point is the starting point for a new search and A* extends the path from there on
      pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
      visited_counter--; // First point is already counted as visited

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "!!!!!! Starting an A* path from (x=%d, y=%d, yaw=?) !!!!!!", pathNodes.back().pos.x, pathNodes.back().pos.y);
      // Plan to closest open Node using A*. Here, `goals` is essentially the map, so we use `goals`
      // to determine the distance from the end of a potential path to the nearest free space
      bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes); // TODO(Aron): A* plans as a differential drive robot with a 1x1 grid footprint
      if (resign)
      {
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "--> size of A* path to closest open Node is %lu", pathNodes.size());

      // Update visited grid
      for (const auto pathNode : pathNodes)
      {
        if (visited[pathNode.pos.y][pathNode.pos.x])
        {
          multiple_pass_counter++;
        }
        visited[pathNode.pos.y][pathNode.pos.x] = eNodeVisited; // TODO(Aron): not done according to footprint for A* path now
      }
      if (pathNodes.size() > 0)
      {
        multiple_pass_counter--; // First point is already counted as visited
      }

      spiral_counter_++; // Count number of spirals planned
      if (spiral_counter_ > 2)
      {
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "@@@@@@ BREAK INSERTED TO ONLY PLAN CERTAIN AMOUNT OF SPIRALS @@@@@@"); // For debugging purposes
      break;
      }

      gridNode_t lastNodeAstar = pathNodes.back(); // Save the value of the final node of the A star path to compare to later

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "!!!!!! Starting a spiral from (x=%d, y=%d, yaw=?) !!!!!!", pathNodes.back().pos.x, pathNodes.back().pos.y);
      // Spiral fill from current position (added to A* transition path)
      pathNodes = spiral(grid, pathNodes, yaw_start, visited); // It overwrites yaw_start if its not a first spiral

      // Need to extract only the spiral part for visualization
      std::list<gridNode_t> pathNodes_spiral = pathNodes; // Work with a copy so that pathNodes is not affected later
      std::list<gridNode_t>::iterator it = pathNodes_spiral.begin();
      for (const auto node : pathNodes_spiral)
      {
        if (node.pos.x == lastNodeAstar.pos.x && node.pos.y == lastNodeAstar.pos.y)
        {
          break; // Break if iterator is found that matches last node of A* path
        }
        else if (it == pathNodes_spiral.end())
        {
          break; // Break if it is already at last node to prevent it becoming bigger than pathNodes_spiral
        }
        ++it; // Iterate one further to find starting point of new spiral
      }
      pathNodes_spiral.erase(pathNodes_spiral.begin(), it);
      if (pathNodes_spiral.size() > 1)
      {
        SpiralSTC::visualizeSpirals(pathNodes_spiral, "spiral" + std::to_string(goals.size() + 1), 0.2, 0.5, 0.0, 0.6, 0.0);
      }

      goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goalpoints

      for (const auto pathNode : pathNodes)
      {
        Point_t newPoint = {pathNode.pos.x, pathNode.pos.y};
        visited_counter++;
        fullPath.push_back(newPoint);
      }
    }

    visualizeGrid(visited, "visitedCubes", 0.25, 0.0, 0.0, 0.6);

    return fullPath;
  }

  bool SpiralSTC::makePlan(const geometry_msgs::msg::PoseStamped &start, const geometry_msgs::msg::PoseStamped &goal,
                           std::vector<geometry_msgs::msg::PoseStamped> &plan)
  {
    if (!initialized_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Initialized!");
    }

    clock_t begin = clock();
    Point_t start_point;
    double yaw_start;

    std::vector<std::vector<bool>> grid;
    if (!parseGrid(costmap_, grid, (robot_radius_ * 2)/division_factor_, (tool_radius_ * 2)/division_factor_, start, start_point, yaw_start))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Could not parse retrieved grid");
      return false;
    }

    coarse_grid_.resizeMap(ceil(costmap_->getSizeInMetersX()/tile_size_), ceil(costmap_->getSizeInMetersY()/tile_size_), tile_size_, grid_origin_.x, grid_origin_.y);

    // Grid visualization with occupied cells greyed out
    visualizeGridlines();
    visualizeGrid(grid, "gridCubes", 0.6, 0.0, 0.0, 0.0);

    // Find a location on the map so that the manoeuvre will not be out of bounds (not relevant)
    int mid_x = coarse_grid_.getSizeInCellsX()/2;
    int mid_y = coarse_grid_.getSizeInCellsY()/2;
    int mid_x_plus_one = mid_x+1;
    int mid_y_plus_one = mid_y+1;
    int mid_y_minus_one = mid_y-1;
    double yaw = 0.0;
    // Compute the absolute swept path of the 3 basic manoeuvres
    ManoeuvreFootprint(mid_x, mid_y, mid_x, mid_y_plus_one, yaw, left_turn_);
    ManoeuvreFootprint(mid_x, mid_y, mid_x_plus_one, mid_y, yaw, forward_);
    ManoeuvreFootprint(mid_x, mid_y, mid_x, mid_y_minus_one, yaw, right_turn_);

    // Convert absolute cell locations to relative cell locations for each manoeuvre
    // TODO(Aron): make the computation below neater
    for (uint i = 0; i < left_turn_.size(); i++)
    {
      left_turn_[i] = {left_turn_[i].x - mid_x, left_turn_[i].y - mid_y};
    }
    for (uint i = 0; i < forward_.size(); i++)
    {
      forward_[i] = {forward_[i].x - mid_x, forward_[i].y - mid_y};
    }
    for (uint i = 0; i < right_turn_.size(); i++)
    {
      right_turn_[i] = {right_turn_[i].x - mid_x, right_turn_[i].y - mid_y};
    }
    for (auto const cell : left_turn_)
    {
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Test cell: (x=%d,y=%d)", cell.x, cell.y);
    }

    std::list<Point_t> goalPoints = spiral_stc(grid,
                                               start_point,
                                               yaw_start,
                                               spiral_cpp_metrics_.multiple_pass_counter,
                                               spiral_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Naive cpp completed!");
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Converting path to plan");

    parsePointlist2Plan(start, goalPoints, plan);

    // Print some metrics:
    spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter - spiral_cpp_metrics_.multiple_pass_counter;
    spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total visited: %d", spiral_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

    // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
    // (also controlled by planner_frequency parameter in move_base namespace)

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Publishing plan!");
    publishPlan(plan);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published!");
    RCLCPP_DEBUG(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published");

    clock_t end = clock();
    double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Elapsed time: %f", elapsed_secs);

    return true;
  }

  bool SpiralSTC::FootprintCells(int &x_m, int &y_m, double &yaw, std::vector<nav2_costmap_2d::MapLocation> &footprint_cells)
  {
    double x_w, y_w;
    coarse_grid_.mapToWorld(x_m, y_m, x_w, y_w);
    std::vector<geometry_msgs::msg::Point> footprint;
    nav2_costmap_2d::transformFootprint(x_w, y_w, yaw, coarse_grid_ros_->getRobotFootprint(), footprint);
    std::vector<nav2_costmap_2d::MapLocation> footprint_ML;
    for (auto const point : footprint)
    {
      int map_x, map_y;
      coarse_grid_.worldToMapNoBounds(point.x, point.y, map_x, map_y);
      if (map_x < 0 || map_y < 0 || map_x >= (int)coarse_grid_.getSizeInCellsX()|| map_y >= (int)coarse_grid_.getSizeInCellsY())
      {
        return false;
      }
      nav2_costmap_2d::MapLocation mapLoc{(uint)map_x, (uint)map_y};
      footprint_ML.push_back(mapLoc);
    }
    coarse_grid_.convexFillCells(footprint_ML, footprint_cells);
    return true;
  }

  // TODO(Aron): Turn this into multiple functions and call them in initialization procedure
  // also think of a way to save and call up the different to-be-checked grids per manoeuvre
  bool SpiralSTC::ManoeuvreFootprint(int &x1, int &y1, int &x2, int &y2, double &yaw1, std::vector<nav2_costmap_2d::MapLocation> &man_grids)
  {
    // Determine the footprint of the manoeuvre starting pose
    std::vector<nav2_costmap_2d::MapLocation> footprint1_cells;
    if (!FootprintCells(x1, y1, yaw1, footprint1_cells))
    {
      return false;
    }

    // Determine the orientation of the manoeuvre ending pose
    int dx = x2 - x1;
    int dy = y2 - y1;
    double yaw2 = std::atan2(dy, dx);

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoeuvre from (x=%d, y=%d, yaw=%f) to (x=%d, y=%d, yaw=%f)", x1, y1, yaw1, x2, y2, yaw2);

    // Compute the footprints of all intermediate poses
    uint N = 10; // Steps in between footprint 1 and footprint 2
    double yaw_diff, yaw_inter;
    std::vector<nav2_costmap_2d::MapLocation> cells, intermediate_cells;

    // Manually compute the difference in special cases
    if (yaw1 == -0.5*M_PI && yaw2 == M_PI)
    {
      yaw_diff = -0.5*M_PI;
    }
    else if (yaw1 == M_PI && yaw2 == -0.5*M_PI)
    {
      yaw_diff = 0.5*M_PI;
    }
    else
    {
      yaw_diff = yaw2 - yaw1;
    }

    for (uint i = 1; i < N; i++)
    {
      yaw_inter = yaw1 + (i*(yaw_diff))/(N+1); // Determine intermediate orientation between yaw1 and yaw2
      if (yaw_inter > M_PI)
      {
        yaw_inter = yaw_inter - 2*M_PI; // Wrap angle back to (-Pi, Pi]
      }
      if (!FootprintCells(x1, y1, yaw_inter, cells))
      {
        return false;
      }
      intermediate_cells.insert(intermediate_cells.end(), cells.begin(), cells.end());
    }

    // Determine the footprint of the manoeuvre starting pose
    std::vector<nav2_costmap_2d::MapLocation> footprint2_cells;
    if (!FootprintCells(x2, y2, yaw2, footprint2_cells))
    {
      return false;
    }
    footprint2_cells.insert(footprint2_cells.end(), intermediate_cells.begin(), intermediate_cells.end());

    // Save the indexes of the covered cells for footprint 1
    std::vector<int> footprint1_cells_inds;
    for (auto const cell1 : footprint1_cells)
    {
      int index1 = coarse_grid_.getIndex(cell1.x, cell1.y);
      footprint1_cells_inds.push_back(index1);
    }

    // Find the indexes of the covered cells for footprint 2
    std::vector<int> man_grids_inds;
    for (auto const cell2 : footprint2_cells)
    {
      int index2 = coarse_grid_.getIndex(cell2.x, cell2.y);
      bool allow = true;
      // Check if an index of footprint 2 matches with any of the indexes of footprint 1
      for (auto const index1 : footprint1_cells_inds)
      {
        if (index2 == index1)
        {
          allow = false;
          break;
        }
      }
      // If that is not the case, save it into the output vector
      if (allow == true)
      {
        man_grids_inds.push_back(index2);
      }
    }

    // Get rid of duplicates in the to-be-checked grids and convert back to MapLocations
    std::sort(man_grids_inds.begin(), man_grids_inds.end());
    man_grids_inds.erase(unique(man_grids_inds.begin(), man_grids_inds.end()), man_grids_inds.end());
    for (auto const ind : man_grids_inds)
    {
      uint mapgrid_x, mapgrid_y;
      coarse_grid_.indexToCells(ind, mapgrid_x, mapgrid_y);
      nav2_costmap_2d::MapLocation maploc{mapgrid_x, mapgrid_y};
      man_grids.push_back(maploc);
    }

    return true;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////// Visualization Methods /////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////

  void SpiralSTC::visualizeGridlines()
  {
    visualization_msgs::msg::Marker gridlines;
    gridlines.header.frame_id = global_frame_.c_str();
    gridlines.header.stamp = rclcpp::Time();
    gridlines.ns = "gridLines";
    gridlines.id = 0;
    gridlines.type = visualization_msgs::msg::Marker::LINE_LIST;
    gridlines.action = visualization_msgs::msg::Marker::ADD;
    gridlines.pose.orientation.w = 1.0;
    gridlines.scale.x = 0.02;
    gridlines.color.a = 0.5;
    gridlines.color.r = gridlines.color.g = gridlines.color.b = 0.0;

    geometry_msgs::msg::Point p;

    for (uint32_t i = 0; i < costmap_->getSizeInMetersX()/tile_size_ ; i++)
    {
      p.x = (grid_origin_.x);
      p.y = (grid_origin_.y) + i*tile_size_;
      p.z = 0.0;
      gridlines.points.push_back(p);
      p.x = (grid_origin_.x) + costmap_->getSizeInMetersX();
      gridlines.points.push_back(p);
    }

    for (uint32_t i = 0; i < costmap_->getSizeInMetersY()/tile_size_ ; i++)
    {
      p.x = (grid_origin_.x) + i*tile_size_;
      p.y = (grid_origin_.y);
      p.z = 0.0;
      gridlines.points.push_back(p);
      p.y = (grid_origin_.y) + costmap_->getSizeInMetersY();
      gridlines.points.push_back(p);
    }

    vis_pub_grid_->publish(gridlines);
  }

  void SpiralSTC::visualizeGrid(std::vector<std::vector<bool>> const &grid, std::string name_space, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker gridCubes = cubeMarker(global_frame_.c_str(), name_space, 0, tile_size_, a, r, g, b);
    geometry_msgs::msg::Point p;
    int ix, iy;
    int nRows = grid.size();
    int nCols = grid[0].size();

    for (iy = 0; iy < nRows; ++(iy))
    {
      for (ix = 0; ix < nCols; ++(ix))
      {
        if (grid[iy][ix] == true)
        {
          p.x = (ix + 0.5)*tile_size_ + grid_origin_.x;
          p.y = (iy + 0.5)*tile_size_ + grid_origin_.y;
          gridCubes.points.push_back(p);
        }
      }
    }

    vis_pub_grid_->publish(gridCubes);
  }

  void SpiralSTC::visualizeSpirals(std::list<gridNode_t> &spiralNodes, std::string name_space, float w, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker spiral = lineStrip(global_frame_.c_str(), name_space, 0, w, a, r, g, b);
    geometry_msgs::msg::Point p;
    p.z = 0.0;
    for (const auto spiralNode : spiralNodes)
    {
      p.x = (spiralNode.pos.x + 0.5)*tile_size_ + grid_origin_.x;
      p.y = (spiralNode.pos.y + 0.5)*tile_size_ + grid_origin_.y;
      spiral.points.push_back(p);
    }
    vis_pub_spirals_->publish(spiral);
  }

  visualization_msgs::msg::Marker SpiralSTC::cubeMarker(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker cubeList;
    cubeList.header.frame_id = frame_id;
    cubeList.header.stamp = rclcpp::Time();
    cubeList.ns = name_space;
    cubeList.action = visualization_msgs::msg::Marker::ADD;
    cubeList.pose.orientation.w = 1.0;
    cubeList.id = id;
    cubeList.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cubeList.scale.x = size;
    cubeList.scale.y = size;
    cubeList.scale.z = size;
    cubeList.color.a = a;
    cubeList.color.r = r;
    cubeList.color.g = g;
    cubeList.color.b = b;
    return cubeList;
  }

  visualization_msgs::msg::Marker SpiralSTC::lineStrip(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker lineStrip;
    lineStrip.header.frame_id = frame_id;
    lineStrip.header.stamp = rclcpp::Time();
    lineStrip.ns = name_space;
    lineStrip.action = visualization_msgs::msg::Marker::ADD;
    lineStrip.pose.orientation.w = 1.0;
    lineStrip.id = id;
    lineStrip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lineStrip.scale.x = size;
    lineStrip.scale.y = size;
    lineStrip.scale.z = size;
    lineStrip.color.a = a;
    lineStrip.color.r = r;
    lineStrip.color.g = g;
    lineStrip.color.b = b;
    return lineStrip;
  }
} // namespace full_coverage_path_planner

// Register this planner as a nav2_core::GlobalPlanner plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav2_core::GlobalPlanner)
