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
      // coarse_grid_ros_= costmap_ros.get();
      coarse_grid_ros_= costmap_ros.get();
      //coarse_grid_ros_= &(costmap_ros.get());
      global_frame_ = costmap_ros->getGlobalFrameID();

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"),
                  "Configuring plugin %s of type NavfnPlanner", name_.c_str());

      // Create a publisher to visualize the plan
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
      vis_pub_grid_->on_activate(); // Aron: necessary??
      vis_pub_spirals_->on_activate(); // Aron: necessary??
  }

  void SpiralSTC::deactivate()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Deactivating plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
      vis_pub_grid_->on_deactivate(); // Aron: necessary??
      vis_pub_spirals_->on_deactivate(); // Aron: necessary??
  }

  void SpiralSTC::cleanup()
  {
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"), "Cleaning up plugin %s of type FullCoveragePathPlanner",
      name_.c_str());
    // TODO(clopez) Add proper cleanup
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
                                          std::vector<std::vector<bool>> &visited)
  {
    int nRows = grid.size();
    int nCols = grid[0].size();
    std::list<gridNode_t> pathNodes(init); // Copy incoming list to pathNodes
    std::list<gridNode_t>::iterator it = --(pathNodes.end()); // Create iterator and let it point to the last element of end
    if (pathNodes.size() > 1) // If list is length 1, keep iterator at end
      it--;                   // Let iterator point to second to last element

    gridNode_t prev = *(it);
    std::vector<nav2_costmap_2d::MapLocation> man_grids, visited_after_man;
    bool done = false;
    while (!done)
    {
      // Initialize spiral direction towards y-axis
      int dx = 0;
      int dy = 1;
      int dx_prev;
      double yaw = -1; // For now indicates no starting orientation is known
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
      // This condition might change in the loop below
      done = true;
      // Loop over the four possible directions: forward, right, backward, left
      for (size_t i = 0; i < 4; ++i)
      {
        int x1 = pathNodes.back().pos.x;
        int y1 = pathNodes.back().pos.y;
        int x2 = pathNodes.back().pos.x + dx;
        int y2 = pathNodes.back().pos.y + dy;
        man_grids = manoeuvreFootprint(x1, y1, x2, y2, yaw);
        if (test_counter < 1)
        {
          RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Showing manoeuvre grids once..........................");
          RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), ("Manoeuvre grids size: " + std::to_string(man_grids.size())).c_str());
          RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), ("x1: " + std::to_string(x1) + ", y1: " + std::to_string(y1) + ", yaw: " + std::to_string(yaw)).c_str());
          for (auto const grid : man_grids)
          {
            RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), ("man_grid: {" + std::to_string(grid.x) + ", " + std::to_string(grid.y) + "}").c_str());
          }
          test_counter++;
        }
        // Check if entire footprint falls inside grid limits
        if (x2 >= 0 + (division_factor_-1)/2 && x2 < nCols - (division_factor_-1)/2 && y2 >= 0 + (division_factor_-1)/2 && y2 < nRows - (division_factor_-1)/2)
        {
          bool man_is_free = true;
          for (const auto man_grid : man_grids)
          {
            if (!(grid[man_grid.y][man_grid.x] == eNodeOpen && visited[man_grid.y][man_grid.x] == eNodeOpen))
            {
              man_is_free = false;
              break;
            }
          }
          if (man_is_free)
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
            // brake for loop
            break;
          }
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
                                           int &multiple_pass_counter,
                                           int &visited_counter)
  {
    // Initial node is initially set as visited so it does not count
    multiple_pass_counter = 0;
    visited_counter = 0; // TODO(Aron): currently these counters don't work due to the footprint

    std::vector<std::vector<bool>> visited;
    visited = grid; // Copy grid matrix
    int x = init.x;
    int y = init.y;

    Point_t new_point = {x, y};
    gridNode_t new_node =
        {
            new_point, // Point: x,y
            0,         // Cost
            0,         // Heuristic
        };
    std::list<gridNode_t> pathNodes;
    std::list<Point_t> fullPath;
    pathNodes.push_back(new_node);
    visited[y][x] = eNodeVisited;  // TODO(Aron): Set initial footprint as visited

    pathNodes = spiral(grid, pathNodes, visited);    // First spiral fill

    visualizeSpirals(pathNodes, "first_spiral", 0.2, 0.5, 0.0, 0.6, 0.0);

    std::list<Point_t> goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goalpoints
    for (const auto pathNode : pathNodes) // Add points to full path
    {
      Point_t newPoint = {pathNode.pos.x, pathNode.pos.y};
      visited_counter++;
      fullPath.push_back(newPoint);
    }
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end())); // Remove all elements from pathNodes list except last element

    while (goals.size() != 0)
    {
      // Remove all elements from pathNodes list except last element.
      // The last point is the starting point for a new search and A* extends the path from there on
      pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
      visited_counter--; // First point is already counted as visited
      // Plan to closest open Node using A*.  `goals` is essentially the map, so we use `goals`
      // to determine the distance from the end of a potential path to the nearest free space
      bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
      if (resign)
      {
        break;
      }

      // Update visited grid
      for (const auto pathNode : pathNodes)
      {
        if (visited[pathNode.pos.y][pathNode.pos.x])
        {
          multiple_pass_counter++;
        }
        visited[pathNode.pos.y][pathNode.pos.x] = eNodeVisited; // TODO(Aron): this is done in both spiral() and here!
      }
      if (pathNodes.size() > 0)
      {
        multiple_pass_counter--; // First point is already counted as visited
      }

      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "NOTE: break inserted to only plan one spiral!!!!!!!!!!!!!!!!!!");
      break;

      gridNode_t lastNodeAstar = pathNodes.back();

      // Spiral fill from current position (added to A* transition path)
      pathNodes = spiral(grid, pathNodes, visited);

      // Need to extract only the spiral part for visualization
      std::list<gridNode_t> pathNodes_copy = pathNodes; // Work with a copy so that splice() does not affect pathNodes
      std::list<gridNode_t>:: iterator it = pathNodes_copy.begin();
      for (const auto node : pathNodes_copy)
      {
        if (node.pos.x == lastNodeAstar.pos.x && node.pos.y == lastNodeAstar.pos.y)
        {
          break; // Break if iterator is found that matches last node of A* path
        }
        ++it; // Iterate one further to find starting point of new spiral
      }
      std::list<gridNode_t> pathNodes_spiral;
      pathNodes_spiral.splice(pathNodes_spiral.begin(), pathNodes_copy, it, pathNodes_copy.end());
      if (pathNodes_spiral.size() > 1)
      {
        SpiralSTC::visualizeSpirals(pathNodes_spiral, "spiral" + std::to_string(goals.size() + 1), 0.2, 0.5, 0.0, 0.6, 0.0);
      }
      if (pathNodes_spiral.size() > 0)
      {
        pathNodes_spiral.clear();
      }

      goals = map_2_goals(visited, eNodeOpen); // Retrieve remaining goalpoints

      for (const auto pathNode : pathNodes)
      {
        Point_t newPoint = {pathNode.pos.x, pathNode.pos.y};
        visited_counter++;
        fullPath.push_back(newPoint);
      }
    }

    visualizeGrid(visited, "visitedCubes", 0.3, 0.0, 0.0, 0.6);

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
    Point_t startPoint;

    std::vector<std::vector<bool>> grid;
    if (!parseGrid(costmap_, grid, (robot_radius_ * 2)/division_factor_, (tool_radius_ * 2)/division_factor_, start, startPoint))
    {
      RCLCPP_ERROR(rclcpp::get_logger("FullCoveragePathPlanner"), "Could not parse retrieved grid");
      return false;
    }

    coarse_grid_.resizeMap(ceil(costmap_->getSizeInMetersX()/tile_size_), ceil(costmap_->getSizeInMetersY()/tile_size_), tile_size_, grid_origin_.x, grid_origin_.y);

    // Grid visualization with occupied cells greyed out
    visualizeGridlines();
    visualizeGrid(grid, "gridCubes", 0.6, 0.0, 0.0, 0.0);

    std::list<Point_t> goalPoints = spiral_stc(grid,
                                               startPoint,
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

  // TODO(Aron): Turn this into multiple functions and call them in initialization procedure
  // also think of a way to save and call up the different to-be-checked grids per manoeuvre
  std::vector<nav2_costmap_2d::MapLocation> SpiralSTC::manoeuvreFootprint(int &x1, int &y1, int &x2, int &y2, double &yaw1)
  {
    std::vector<nav2_costmap_2d::MapLocation> man_grids;
    std::vector<int> man_grids_inds;
    double x, y;

    // Determine footprint of manoeuvre starting position
    if (yaw1 == -1) {yaw1 = 0.0;} // for now assume initial orientation is 0 rad
    coarse_grid_.mapToWorld(x1, y1, x, y);
    std::vector<geometry_msgs::msg::Point> footprint1;
    nav2_costmap_2d::transformFootprint(x, y, yaw1, coarse_grid_ros_->getRobotFootprint(), footprint1);

    if (test_counter < 1)
    {
      for (auto const point : footprint1)
      {
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), ("Footprint point in world coordinates: " + std::to_string(point.x) + ", " + std::to_string(point.y)).c_str());
      }
    }

    // Convert that footprint and find the covered cells
    std::vector<nav2_costmap_2d::MapLocation> footprint1_ML, footprint1_cells;
    for (auto const point : footprint1)
    {
      uint mapx, mapy;
      coarse_grid_.worldToMap(point.x, point.y, mapx, mapy);
      nav2_costmap_2d::MapLocation mapLoc{static_cast<uint>(mapx), static_cast<uint>(mapy)};
      footprint1_ML.push_back(mapLoc);
    }
    coarse_grid_.convexFillCells(footprint1_ML, footprint1_cells);

    // Determine the footprint of the manoeuvre ending position
    int dx = x2 - x1;
    int dy = y2 - y1;
    double yaw2 = std::atan2(dy, dx);
    coarse_grid_.mapToWorld(x2, y2, x, y);
    std::vector<geometry_msgs::msg::Point> footprint2;
    nav2_costmap_2d::transformFootprint(x, y, yaw2, coarse_grid_ros_->getRobotFootprint(), footprint2);

    // Convert that footprint and find the covered cells
    std::vector<nav2_costmap_2d::MapLocation> footprint2_ML, footprint2_cells;
    for (auto const point : footprint2)
    {
      uint mapx, mapy;
      coarse_grid_.worldToMap(point.x, point.y, mapx, mapy);
      nav2_costmap_2d::MapLocation mapLoc{static_cast<uint>(mapx), static_cast<uint>(mapy)};
      footprint2_ML.push_back(mapLoc);
    }
    coarse_grid_.convexFillCells(footprint2_ML, footprint2_cells);

    // Temporary debugging output (only printed once)
    if (test_counter < 1)
    {
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Footprint 1 below ============");
      for (auto const cell : footprint1_ML)
      {
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), ("Footprint corner: " + std::to_string(cell.x) + ", " + std::to_string(cell.y)).c_str());
      }
      RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Footprint 2 below ============");
      for (auto const cell : footprint2_ML)
      {
        RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), ("Footprint corner: " + std::to_string(cell.x) + ", " + std::to_string(cell.y)).c_str());
      }
    }

    // Save the indexes of the covered cells for footprint 1
    std::vector<int> footprint1_cells_inds;
    for (auto const cell1 : footprint1_cells)
    {
      int index1 = coarse_grid_.getIndex(cell1.x, cell1.y);
      footprint1_cells_inds.push_back(index1);
    }

    // Find the indexes of the covered cells for footprint 2
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

    return man_grids;
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

  visualization_msgs::msg::Marker SpiralSTC::sphereMarker(std::string frame_id, std::string name_space, int id, float size, float a, float r, float g, float b)
  {
    visualization_msgs::msg::Marker sphereList;
    sphereList.header.frame_id = frame_id;
    sphereList.header.stamp = rclcpp::Time();
    sphereList.ns = name_space;
    sphereList.action = visualization_msgs::msg::Marker::ADD;
    sphereList.pose.orientation.w = 1.0;
    sphereList.id = id;
    sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphereList.scale.x = size;
    sphereList.scale.y = size;
    sphereList.scale.z = size;
    sphereList.color.a = a;
    sphereList.color.r = r;
    sphereList.color.g = g;
    sphereList.color.b = b;
    return sphereList;
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
