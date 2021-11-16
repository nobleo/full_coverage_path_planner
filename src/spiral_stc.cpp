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

      // Currently this plugin does not use the costmap, instead request a map from a server (will change in the future)
      costmap_ = costmap_ros->getCostmap();
      global_frame_ = costmap_ros->getGlobalFrameID();
      coarse_grid_ros = costmap_ros.get();

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),
                  "Configuring plugin %s of type NavfnPlanner", name_.c_str());

      // Create publishers to visualize the planner output
      plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("plan", 1);
      grid_pub = node_->create_publisher<visualization_msgs::msg::Marker>("grid", 0);
      spirals_pub = node_->create_publisher<visualization_msgs::msg::Marker>("spirals", 0);

      // TODO(Aron): These parameters are not used anymore, change parseGrid() function?
      // Define  robot radius (radius) parameter
      double robot_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".robot_radius", rclcpp::ParameterValue(robot_radius_default));
      node_->get_parameter(name_ + ".robot_radius", robot_radius_);
      // Define  tool radius (radius) parameter
      double tool_radius_default = 0.5;
      declare_parameter_if_not_declared(node_, name_ + ".tool_radius", rclcpp::ParameterValue(tool_radius_default));
      node_->get_parameter(name_ + ".tool_radius", tool_radius_);

      // TODO(Aron): Define newly introduced parameters here, after moving them from the header to navigations_params.yaml

      initialized_ = true;
    }
  }

  void SpiralSTC::activate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Activating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    grid_pub->on_activate();
    spirals_pub->on_activate();
  }

  void SpiralSTC::deactivate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Deactivating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    grid_pub->on_deactivate();
    spirals_pub->on_deactivate();
  }

  void SpiralSTC::cleanup()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Cleaning up plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    // TODO(CesarLopez): Add proper cleanup
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
    std::list<gridNode_t> path_nodes(init); // Copy incoming list to path_nodes
    std::list<gridNode_t>::iterator it = --(path_nodes.end()); // Create iterator and let it point to the last element of end
    if (path_nodes.size() > 1) // If list is length 1, keep iterator at end
    {
      it--; // Let iterator point to second to last element
    }
    gridNode_t prev = *(it);

    // Initialize spiral direction towards robot's y-axis
    int dx = 0;
    int dy = 1;
    int dx_prev;
    double yaw1 = yaw_start; // This value will change later on

    // Mark initial footprint as visited
    std::vector<nav2_costmap_2d::MapLocation> init_cells;
    if (!computeFootprintCells(path_nodes.back().pos.x, path_nodes.back().pos.y, yaw1, "tool", init_cells))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Starting footprint seems to be out of bounds!");
    }
    for (const auto cell : init_cells)
    {
      visited[cell.y][cell.x] = eNodeVisited;
      visited_copy[cell.y][cell.x] = eNodeVisited;
    }

    std::vector<nav2_costmap_2d::MapLocation> man_grids;
    bool done = false;
    while (!done)
    {
      if (it != path_nodes.begin())
      {
        // Turn counter-clockwise
        dx = path_nodes.back().pos.x - prev.pos.x;
        dy = path_nodes.back().pos.y - prev.pos.y;
        yaw1 = std::atan2(dy,dx); // Keep overwriting the orientation according to the last two nodes
        dx_prev = dx;
        dx = -dy;
        dy = dx_prev;
      }
      done = true; // This condition might change in the loop below
      // Loop over the three possible directions: left, forward, right (directions taken before counter-clockwise turn)
      for (size_t i = 0; i < 3; ++i)
      {
        int x1 = path_nodes.back().pos.x;
        int y1 = path_nodes.back().pos.y;
        int x2 = path_nodes.back().pos.x + dx;
        int y2 = path_nodes.back().pos.y + dy;
        double yaw2 = std::atan2(y2-y1,x2-x1);

        man_grids.clear(); // Clear the cell vector of te manoeuvre before filling it again
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoeuvre from (x=%d, y=%d, yaw=%f) to (x=%d, y=%d, yaw=%f)", x1, y1, yaw1, x2, y2, yaw2);
        bool man_is_free = true; // This condition might change in the loop below

        // Determine map extremes
        int x_max = coarse_grid.getSizeInCellsX() - 1;
        int y_max = coarse_grid.getSizeInCellsY() - 1;

        // Apply a rotation to the relative manoeuvre cells to convert from robot frame to world frame
        if (i == 0) // Relative left turn manoeuvre
        {
          man_grids.resize(left_turn_rel.size());
          for (uint i = 0; i < left_turn_rel.size(); i++)
          {
            Point_t p = rotatePoint(x1 + left_turn_rel[i].x, y1 + left_turn_rel[i].y, x1, y1, yaw1);
            if (!checkMapBounds(p.x, p.y, x_max, y_max))
            {
              RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoevre out of bounds, looking in other directions...");
              man_is_free = false;
              break;
            }
            else
            {
              man_grids[i] = {static_cast<uint>(p.x), static_cast<uint>(p.y)};
            }
          }
          max_overlap = max_overlap_turn;
        }
        else if (i == 1) // Relative forward manoeuvre
        {
          man_grids.resize(forward_rel.size());
          for (uint i = 0; i < forward_rel.size(); i++)
          {
            Point_t p = rotatePoint(x1 + forward_rel[i].x, y1 + forward_rel[i].y, x1, y1, yaw1);
            if (!checkMapBounds(p.x, p.y, x_max, y_max))
            {
              RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Manoevre out of bounds, looking in other directions...");
              man_is_free = false;
              break;
            }
            else
            {
              man_grids[i] = {static_cast<uint>(p.x), static_cast<uint>(p.y)};
            }
          }
          max_overlap = max_overlap_forward;
        }
        else if (i == 2) // Relative right turn manoeuvre
        {
          man_grids.resize(right_turn_rel.size());
          for (uint i = 0; i < right_turn_rel.size(); i++)
          {
            Point_t p = rotatePoint(x1 + right_turn_rel[i].x, y1 + right_turn_rel[i].y, x1, y1, yaw1);
            if (!checkMapBounds(p.x, p.y, x_max, y_max))
            {
              RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "  Manoevre out of bounds, looking in other directions...");
              man_is_free = false;
              break;
            }
            else
            {
              man_grids[i] = {static_cast<uint>(p.x), static_cast<uint>(p.y)};
            }
          }
          max_overlap = max_overlap_turn;
        }

        // Check the manoeuvre cells of the vehicle for collisions
        for (const auto man_grid : man_grids)
        {
          if (grid[man_grid.y][man_grid.x] == eNodeVisited)
          {
            man_is_free = false;
            break;
          }
        }

        // Check the manoeuvre cells of the tool for overlap
        int overlap = 0;
        std::vector<nav2_costmap_2d::MapLocation> visited_cells;
        visited_cells.clear();
        computeManoeuvreFootprint(x1, y1, x2, y2, yaw1, "tool", visited_cells); // TODO(Aron): Precompute and reuse this just like with the vehicle manoeuvres
        for (const auto visited_cell : visited_cells)
        {
          if (grid[visited_cell.y][visited_cell.x] == eNodeOpen && visited[visited_cell.y][visited_cell.x] == eNodeVisited)
          {
            overlap++;
          }
        }
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "  --> with size=%lu (& %lu) of which %d are overlapping", man_grids.size(), visited_cells.size(), overlap);

        // Check if it can still go either right or left at the final orientation
        std::vector<Point_t> future_left, future_right;
        future_left.resize(left_turn_rel.size());
        for (uint i = 0; i < left_turn_rel.size(); i++)
        {
          Point_t p = rotatePoint(x2 + left_turn_rel[i].x, y2 + left_turn_rel[i].y, x2, y2, yaw2);
          future_left[i] = {p.x, p.y};
        }
        future_right.resize(right_turn_rel.size());
        for (uint i = 0; i < right_turn_rel.size(); i++)
        {
          Point_t p = rotatePoint(x2 + right_turn_rel[i].x, y2 + right_turn_rel[i].y, x2, y2, yaw2);
          future_right[i] = {p.x, p.y};
        }
        bool future_left_rejected = false;
        bool future_right_rejected = false;
        for (uint i = 0; i < future_left.size(); i++)
        {
          if (!checkMapBounds(future_left[i].x, future_left[i].y, x_max, y_max) || grid[future_left[i].y][future_left[i].x] == eNodeVisited)
          {
            future_left_rejected = true;
          }
          if (!checkMapBounds(future_right[i].x, future_right[i].y, x_max, y_max) || grid[future_right[i].y][future_right[i].x] == eNodeVisited)
          {
            future_right_rejected = true;
          }
        }
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "  --> causing collision: %d, future left rejected: %d, future right rejected: %d", !man_is_free, future_left_rejected, future_right_rejected);
        if (future_left_rejected && future_right_rejected)
        {
          man_is_free = false; // Do not make the manoeuvre if it is not possible to turn right or left at the next step
        }

        // When all conditions are met, add the point to path_nodes and mark the covered cells as visited
        if (man_is_free && overlap <= max_overlap)
        {
          Point_t new_point = {x2, y2};
          gridNode_t new_node = {new_point, 0, 0};
          prev = path_nodes.back();
          path_nodes.push_back(new_node);
          it = --(path_nodes.end());
          for (const auto cell : visited_cells)
          {
            visited[cell.y][cell.x] = eNodeVisited;
            visited_copy[cell.y][cell.x] = eNodeVisited;
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
    return path_nodes;
  }

  std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool>> const &grid,
                                           Point_t &init,
                                           double &yaw_start,
                                           int &multiple_pass_counter,
                                           int &visited_counter)
  {
    multiple_pass_counter = 0; // Initial node is initially set as visited so it does not count
    visited_counter = 0; // TODO(Aron): Currently these counters don't work properly due to the footprint

    std::vector<std::vector<bool>> visited;
    visited = grid; // Copy grid matrix
    visited_copy.resize(visited[0].size(), std::vector<bool>(visited.size())); // For debugging purposes, to only see the grids that are marked as visited by the spirals
    int init_x = init.x;
    int init_y = init.y;
    Point_t new_point = {init_x, init_y};
    gridNode_t new_node =
        {
            new_point, // Point: x,y
            0,         // Cost
            0,         // Heuristic
        };
    std::list<gridNode_t> path_nodes;
    std::list<Point_t> full_path;
    path_nodes.push_back(new_node);

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "!!!!!!!!!!!! Starting a spiral from (x=%d, y=%d, yaw=%f) !!!!!!!!!!!!", init_x, init_y, yaw_start);
    path_nodes = spiral(grid, path_nodes, yaw_start, visited); // First spiral fill

    visualizeSpirals(path_nodes, "first_spiral", 0.2, 0.5, 0.0, 0.6, 0.0);

    std::list<Point_t> goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goal points

    for (const auto path_node : path_nodes) // Add points to full path
    {
      Point_t new_point = {path_node.pos.x, path_node.pos.y};
      visited_counter++;
      full_path.push_back(new_point);
    }

    while (goals.size() != 0)
    {
      spiral_counter++; // Count number of spirals planned
      if (spiral_counter == 2)
      {
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "@@@@@@@@@ BREAK INSERTED TO ONLY PLAN CERTAIN AMOUNT OF SPIRALS @@@@@@@@@"); // For debugging purposes
        break;
      }

      // Remove all elements from path_nodes list except last element
      // The last point is the starting point for a new search and A* extends the path from there on
      path_nodes.erase(path_nodes.begin(), --(path_nodes.end()));
      visited_counter--; // First point is already counted as visited

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "!!!!!!!!!!!! Starting an A* path from (x=%d, y=%d) !!!!!!!!!!!!", path_nodes.back().pos.x, path_nodes.back().pos.y);
      // Plan to closest open Node using A*. Here, `goals` is essentially the map, so we use `goals`
      // to determine the distance from the end of a potential path to the nearest free space
      bool accept_a_star = false;
      bool resign;
      while (!accept_a_star)
      {
        resign = a_star_to_open_space(grid, path_nodes.back(), 1, visited, goals, path_nodes); // TODO(Aron): A* plans as a differential drive robot with a 1x1 grid footprint
        if (resign)
        {
          break;
        }
        int x_n = path_nodes.back().pos.x;
        int y_n = path_nodes.back().pos.y;
        std::list<gridNode_t>::iterator it = --(path_nodes.end());
        if (path_nodes.size() > 1)
        {
          it--;
        }
        else
        {
          break;
        }
        gridNode_t prev = *(it);
        double yaw = atan2(y_n-prev.pos.y, x_n-prev.pos.x);
        std::vector<nav2_costmap_2d::MapLocation> a_star_end_footprint;
        accept_a_star = computeFootprintCells(x_n, y_n, yaw, "vehicle", a_star_end_footprint);
        int visit_count = 0;
        for (const auto cell : a_star_end_footprint)
        {
          if (grid[cell.y][cell.x] == eNodeVisited)
          {
            accept_a_star = false;
            break;
          }
          else if (visited[cell.y][cell.x] == eNodeVisited)
          {
            visit_count++;
          }
        }
        if (!accept_a_star || visit_count > max_overlap)
        {
          RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "~~~ A* is not accepted, grid considered visited");
          visited[y_n][x_n] = eNodeVisited;
          path_nodes.erase(++(path_nodes.begin()), path_nodes.end());
          accept_a_star = false;
        }
      }
      if (resign)
      {
        break;
      }

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "--> size of A* path to closest open node is %lu", path_nodes.size());

      // Update visited grid
      for (const auto path_node : path_nodes)
      {
        if (visited[path_node.pos.y][path_node.pos.x])
        {
          multiple_pass_counter++;
        }
        visited[path_node.pos.y][path_node.pos.x] = eNodeVisited;
      }
      if (path_nodes.size() > 0)
      {
        multiple_pass_counter--; // First point is already counted as visited
      }

      gridNode_t last_node_a_star = path_nodes.back(); // Save the value of the final node of the A star path to compare to later

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "!!!!!!!!!!!! Starting a spiral from (x=%d, y=%d, yaw=?) !!!!!!!!!!!!", path_nodes.back().pos.x, path_nodes.back().pos.y);
      // Spiral fill from current position (added to A* transition path)
      path_nodes = spiral(grid, path_nodes, yaw_start, visited); // It overwrites yaw_start if its not a first spiral

      // Need to extract only the spiral part for visualization
      std::list<gridNode_t> path_nodes_spiral = path_nodes; // Work with a copy so that path_nodes is not affected later
      std::list<gridNode_t>::iterator it = path_nodes_spiral.begin();
      for (const auto node : path_nodes_spiral)
      {
        if (node.pos.x == last_node_a_star.pos.x && node.pos.y == last_node_a_star.pos.y)
        {
          break; // Break if iterator is found that matches last node of A* path
        }
        else if (it == path_nodes_spiral.end())
        {
          break; // Break if it is already at last node to prevent it becoming bigger than path_nodes_spiral
        }
        ++it; // Iterate one further to find starting point of new spiral
      }
      path_nodes_spiral.erase(path_nodes_spiral.begin(), it);
      if (path_nodes_spiral.size() > 1)
      {
        SpiralSTC::visualizeSpirals(path_nodes_spiral, "spiral" + std::to_string(goals.size() + 1), 0.2, 0.5, 0.0, 0.6, 0.0);
      }

      goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goal points

      for (const auto path_node : path_nodes)
      {
        Point_t new_point = {path_node.pos.x, path_node.pos.y};
        visited_counter++;
        full_path.push_back(new_point);
      }
    }

    visualizeGrid(visited, "visited_cubes", 0.3, 0.0, 0.0, 0.8);
    visualizeGrid(visited_copy, "visited_cubes_copy", 0.3, 0.0, 0.8, 0.0);

    return full_path;
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
    if (!parseGrid(costmap_, grid, (tool_width * 2)/division_factor, (tool_width * 2)/division_factor, start, start_point, yaw_start))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Could not parse retrieved grid");
      return false;
    }

    coarse_grid.resizeMap(ceil(costmap_->getSizeInMetersX()/tile_size_), ceil(costmap_->getSizeInMetersY()/tile_size_), tile_size_, grid_origin_.x, grid_origin_.y);

    // Grid visualization with occupied cells greyed out
    visualizeGridlines();
    visualizeGrid(grid, "grid_cubes", 0.6, 0.0, 0.0, 0.0);

    // Find a location on the map so that the manoeuvre will not be out of bounds (not relevant)
    int mid_x = coarse_grid.getSizeInCellsX()/2;
    int mid_y = coarse_grid.getSizeInCellsY()/2;
    int mid_x_plus_one = mid_x+1;
    int mid_y_plus_one = mid_y+1;
    int mid_y_minus_one = mid_y-1;
    double yaw = 0.0;
    // Compute the absolute swept path of the 3 basic manoeuvres
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Computing standard manoeuvres");
    computeManoeuvreFootprint(mid_x, mid_y, mid_x, mid_y_plus_one, yaw, "vehicle", left_turn);
    computeManoeuvreFootprint(mid_x, mid_y, mid_x_plus_one, mid_y, yaw, "vehicle", forward);
    computeManoeuvreFootprint(mid_x, mid_y, mid_x, mid_y_minus_one, yaw, "vehicle", right_turn);

    // Convert absolute cell locations to relative cell locations for each manoeuvre
    left_turn_rel.resize(left_turn.size());
    forward_rel.resize(forward.size());
    right_turn_rel.resize(right_turn.size());
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Left (relative) turn below:");
    for (uint i = 0; i < left_turn.size(); i++)
    {
      left_turn_rel[i] = {(int)left_turn[i].x - mid_x, (int)left_turn[i].y - mid_y};
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), (" cell: (x=" + std::to_string(left_turn_rel[i].x) + " , y=" + std::to_string(left_turn_rel[i].y) + ")").c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Forward (relative) manoeuvre below:");
    for (uint i = 0; i < forward.size(); i++)
    {
      forward_rel[i] = {(int)forward[i].x - mid_x, (int)forward[i].y - mid_y};
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), (" cell: (x=" + std::to_string(forward_rel[i].x) + " , y=" + std::to_string(forward_rel[i].y) + ")").c_str());
    }
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Right (relative) turn below:");
    for (uint i = 0; i < right_turn.size(); i++)
    {
      right_turn_rel[i] = {(int)right_turn[i].x - mid_x, (int)right_turn[i].y - mid_y};
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), (" cell: (x=" + std::to_string(right_turn_rel[i].x) + " , y=" + std::to_string(right_turn_rel[i].y) + ")").c_str());
    }

    std::list<Point_t> goal_points = spiral_stc(grid,
                                               start_point,
                                               yaw_start,
                                               spiral_cpp_metrics_.multiple_pass_counter,
                                               spiral_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Naive cpp completed!");
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Converting path to plan");

    parsePointlist2Plan(start, goal_points, plan);

    // Print some metrics:
    spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter - spiral_cpp_metrics_.multiple_pass_counter;
    spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total visited: %d", spiral_cpp_metrics_.visited_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

    // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept (also controlled by planner_frequency parameter in move_base namespace)

    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Publishing plan!");
    publishPlan(plan);
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Plan published!");

    clock_t end = clock();
    double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
    RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Elapsed time: %f", elapsed_secs);

    return true;
  }

  bool SpiralSTC::computeFootprintCells(int &x_m, int &y_m, double &yaw, std::string part, std::vector<nav2_costmap_2d::MapLocation> &footprint_cells)
  {
    // Convert input map locations to world coordinates
    double x_w, y_w;
    coarse_grid.mapToWorld(x_m, y_m, x_w, y_w);
    std::vector<geometry_msgs::msg::Point> footprint;

    // Differentiate between a footprint requested for a vehicle and its tool
    if (part == "vehicle")
    {
      nav2_costmap_2d::transformFootprint(x_w, y_w, yaw, coarse_grid_ros->getRobotFootprint(), footprint);
    }
    else if (part == "tool")
    {
      //TODO(Aron): This has to become a node parameter or something of that nature, not declared here
      std::vector<geometry_msgs::msg::Point> tool_footprint;
      geometry_msgs::msg::Point p;
      p.x = 0.2;
      p.y = 0.4;
      tool_footprint.push_back(p);
      p.x = 0.545;
      p.y = 0.4;
      tool_footprint.push_back(p);
      p.x = 0.545;
      p.y = -0.4;
      tool_footprint.push_back(p);
      p.x = 0.2;
      p.y = -0.4;
      tool_footprint.push_back(p);
      nav2_costmap_2d::transformFootprint(x_w, y_w, yaw, tool_footprint, footprint);
    }

    // Convert footprint from Point vector to MapLocation vector
    int x_max = coarse_grid.getSizeInCellsX() - 1;
    int y_max = coarse_grid.getSizeInCellsY() - 1;
    std::vector<nav2_costmap_2d::MapLocation> footprint_ML;
    for (const auto point : footprint)
    {
      int map_x, map_y;
      coarse_grid.worldToMapNoBounds(point.x, point.y, map_x, map_y); // Use worldToMapNoBounds() variant because of bugs in the one that enforces bounds
      if (!checkMapBounds(map_x, map_y, x_max, y_max))
      {
        return false; // When the requested footprint falls out of bounds, the function returns false
      }
      nav2_costmap_2d::MapLocation map_loc{static_cast<uint>(map_x), static_cast<uint>(map_y)};
      footprint_ML.push_back(map_loc);
    }

    // Filter out double corner points that happen due to the grid resolution to prevent problems with convexFillCells() later
    std::vector<int> footprint_inds;
    for (const auto loc : footprint_ML)
    {
      int index = coarse_grid.getIndex(loc.x, loc.y);
      footprint_inds.push_back(index);
    }
    std::sort(footprint_inds.begin(), footprint_inds.end());
    footprint_inds.erase(std::unique(footprint_inds.begin(), footprint_inds.end()), footprint_inds.end());
    footprint_ML.clear();
    for (const auto ind : footprint_inds)
    {
      nav2_costmap_2d::MapLocation map_loc;
      coarse_grid.indexToCells(ind, map_loc.x, map_loc.y);
      footprint_ML.push_back(map_loc);
    }

    // Find the cells below the convex footprint and save them
    if (footprint_ML.size() < 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Footprint does not consists of 3 or more points!");
    }
    coarse_grid.convexFillCells(footprint_ML, footprint_cells);
    return true;
  }

  bool SpiralSTC::computeManoeuvreFootprint(int &x1, int &y1, int &x2, int &y2, double &yaw1, std::string part, std::vector<nav2_costmap_2d::MapLocation> &man_grids)
  {
    // Determine the footprint of the manoeuvre starting pose
    std::vector<nav2_costmap_2d::MapLocation> footprint1_cells;
    if (part == "vehicle" && !computeFootprintCells(x1, y1, yaw1, "vehicle", footprint1_cells))
    {
      return false;
    }
    else if (part == "tool" && !computeFootprintCells(x1, y1, yaw1, "tool", footprint1_cells))
    {
      return false;
    }

    // Determine the orientation of the manoeuvre ending pose
    double yaw2 = std::atan2(y2 - y1, x2 - x1);

    // Compute the footprints of all intermediate poses
    double yaw_diff, yaw_inter;

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

    // Determine the footprint of the intermediate poses of the manoeuvre
    std::vector<nav2_costmap_2d::MapLocation> intermediate_cells;
    for (int i = 1; i < N_footprints; i++)
    {
      std::vector<nav2_costmap_2d::MapLocation> cells;
      yaw_inter = yaw1 + (i*(yaw_diff))/(N_footprints+1);
      if (yaw_inter > M_PI)
      {
        yaw_inter = yaw_inter - 2*M_PI; // Wrap angle back to (-PI, PI]
      }
      if (part == "vehicle" && !computeFootprintCells(x1, y1, yaw_inter, "vehicle", cells))
      {
        return false;
      }
      else if (part == "tool" && !computeFootprintCells(x1, y1, yaw_inter, "tool", cells))
      {
          return false;
      }
      intermediate_cells.insert(intermediate_cells.end(), cells.begin(), cells.end());
    }

    // Determine the footprint of the manoeuvre ending pose
    std::vector<nav2_costmap_2d::MapLocation> footprint2_cells;
    if (part == "vehicle" && !computeFootprintCells(x2, y2, yaw2, "vehicle", footprint2_cells))
    {
      return false;
    }
    else if (part == "tool" && !computeFootprintCells(x2, y2, yaw2, "tool", footprint2_cells))
    {
      return false;
    }
    footprint2_cells.insert(footprint2_cells.end(), intermediate_cells.begin(), intermediate_cells.end());

    // Save the indexes of the covered cells for footprint 1
    std::vector<int> footprint1_cells_inds;
    for (const auto cell1 : footprint1_cells)
    {
      int index1 = coarse_grid.getIndex(cell1.x, cell1.y);
      footprint1_cells_inds.push_back(index1);
    }

    // Find the indexes of the covered cells for footprint 2
    std::vector<int> man_grids_inds;
    for (const auto cell2 : footprint2_cells)
    {
      int index2 = coarse_grid.getIndex(cell2.x, cell2.y);
      bool allow = true;
      // Check if an index of footprint 2 matches with any of the indexes of footprint 1
      for (const auto index1 : footprint1_cells_inds)
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
    for (const auto ind : man_grids_inds)
    {
      nav2_costmap_2d::MapLocation map_loc;
      coarse_grid.indexToCells(ind, map_loc.x, map_loc.y);
      man_grids.push_back(map_loc);
    }

    return true;
  }

  Point_t SpiralSTC::rotatePoint(int poi_x, int poi_y, int &icr_x, int &icr_y, double &yaw)
  {
    Point_t p;
    double poi_x_w, poi_y_w, icr_x_w, icr_y_w, rotated_x_w, rotated_y_w;
    coarse_grid.mapToWorld(poi_x, poi_y, poi_x_w, poi_y_w);
    coarse_grid.mapToWorld(icr_x, icr_y, icr_x_w, icr_y_w);
    rotated_x_w = icr_x_w + (poi_x_w - icr_x_w)*cos(yaw) - (poi_y_w - icr_y_w)*sin(yaw);
    rotated_y_w = icr_y_w + (poi_x_w - icr_x_w)*sin(yaw) + (poi_y_w - icr_y_w)*cos(yaw);
    coarse_grid.worldToMapNoBounds(rotated_x_w, rotated_y_w, p.x, p.y);
    return p;
  }

  bool SpiralSTC::checkMapBounds(int x, int y, int &x_max, int &y_max)
  {
    if (x < 0 || x > x_max || y < 0 || y > y_max)
    {
      return false;
    }
    return true;
  }

  /* @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ Visualization Methods @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

  void SpiralSTC::visualizeGridlines()
  {
    visualization_msgs::msg::Marker grid_lines;
    grid_lines.header.frame_id = global_frame_.c_str();
    grid_lines.header.stamp = rclcpp::Time();
    grid_lines.ns = "grid_lines";
    grid_lines.id = 0;
    grid_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    grid_lines.action = visualization_msgs::msg::Marker::ADD;
    grid_lines.pose.orientation.w = 1.0;
    grid_lines.scale.x = 0.02;
    grid_lines.color.a = 0.5;
    grid_lines.color.r = grid_lines.color.g = grid_lines.color.b = 0.0;

    geometry_msgs::msg::Point p;

    for (uint32_t i = 0; i < costmap_->getSizeInMetersX()/tile_size_ ; i++)
    {
      p.x = (grid_origin_.x);
      p.y = (grid_origin_.y) + i*tile_size_;
      p.z = 0.0;
      grid_lines.points.push_back(p);
      p.x = (grid_origin_.x) + costmap_->getSizeInMetersX();
      grid_lines.points.push_back(p);
    }

    for (uint32_t i = 0; i < costmap_->getSizeInMetersY()/tile_size_ ; i++)
    {
      p.x = (grid_origin_.x) + i*tile_size_;
      p.y = (grid_origin_.y);
      p.z = 0.0;
      grid_lines.points.push_back(p);
      p.y = (grid_origin_.y) + costmap_->getSizeInMetersY();
      grid_lines.points.push_back(p);
    }

    grid_pub->publish(grid_lines);
  }

  void SpiralSTC::visualizeGrid(std::vector<std::vector<bool>> const &grid, std::string name_space, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker grid_cubes = cubeMarker(global_frame_.c_str(), name_space, 0, tile_size_, a, r, g, b);
    geometry_msgs::msg::Point p;
    int ix, iy;
    int n_rows = grid.size();
    int n_cols = grid[0].size();

    for (iy = 0; iy < n_rows; ++(iy))
    {
      for (ix = 0; ix < n_cols; ++(ix))
      {
        if (grid[iy][ix] == true)
        {
          p.x = (ix + 0.5)*tile_size_ + grid_origin_.x;
          p.y = (iy + 0.5)*tile_size_ + grid_origin_.y;
          grid_cubes.points.push_back(p);
        }
      }
    }

    grid_pub->publish(grid_cubes);
  }

  void SpiralSTC::visualizeSpirals(std::list<gridNode_t> &spiral_nodes, std::string name_space, float w, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker spiral = lineStrip(global_frame_.c_str(), name_space, 0, w, a, r, g, b);
    geometry_msgs::msg::Point p;
    p.z = 0.0;
    for (const auto spiral_node : spiral_nodes)
    {
      p.x = (spiral_node.pos.x + 0.5)*tile_size_ + grid_origin_.x;
      p.y = (spiral_node.pos.y + 0.5)*tile_size_ + grid_origin_.y;
      spiral.points.push_back(p);
    }
    spirals_pub->publish(spiral);
  }

  visualization_msgs::msg::Marker SpiralSTC::cubeMarker(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker cube_list;
    cube_list.header.frame_id = frame_id;
    cube_list.header.stamp = rclcpp::Time();
    cube_list.ns = name_space;
    cube_list.action = visualization_msgs::msg::Marker::ADD;
    cube_list.pose.orientation.w = 1.0;
    cube_list.id = id;
    cube_list.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cube_list.scale.x = size;
    cube_list.scale.y = size;
    cube_list.scale.z = size;
    cube_list.color.a = a;
    cube_list.color.r = r;
    cube_list.color.g = g;
    cube_list.color.b = b;
    return cube_list;
  }

  visualization_msgs::msg::Marker SpiralSTC::lineStrip(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.frame_id = frame_id;
    line_strip.header.stamp = rclcpp::Time();
    line_strip.ns = name_space;
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.scale.x = size;
    line_strip.scale.y = size;
    line_strip.scale.z = size;
    line_strip.color.a = a;
    line_strip.color.r = r;
    line_strip.color.g = g;
    line_strip.color.b = b;
    return line_strip;
  }
} // namespace full_coverage_path_planner

// Register this planner as a nav2_core::GlobalPlanner plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav2_core::GlobalPlanner)
