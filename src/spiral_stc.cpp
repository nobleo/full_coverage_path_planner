//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/spiral_stc.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav_core::BaseGlobalPlanner)

namespace full_coverage_path_planner
{
void SpiralSTC::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    // Create a publisher to visualize the plan
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh, private_named_nh("~/" + name);

    plan_pub_ = private_named_nh.advertise<nav_msgs::Path>("plan", 1);
    // Try to request the cpp-grid from the cpp_grid map_server
    cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

    // Define  robot radius (radius) parameter
    float robot_radius_default = 0.5f;
    private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
    // Define  tool radius (radius) parameter
    float tool_radius_default = 0.5f;
    private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);
    initialized_ = true;
  }
}

std::list<gridNode_t> SpiralSTC::spiral(std::vector<std::vector<bool> > const& grid, std::list<gridNode_t>& init,
                                        std::vector<std::vector<bool> >& visited)
{
  int dx, dy, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
  // Mountain pattern filling of the open space
  // Copy incoming list to 'end'
  std::list<gridNode_t> pathNodes(init);  

  // this array stores how far the robot can travel in a straight line for each direction
  int free_space_in_dir[5] = {0};
  // for each direction
  for (int i = 1; i < 5; i++) {
    // start from starting pos
    x2 = pathNodes.back().pos.x;
    y2 = pathNodes.back().pos.y;
    // loop until hits wall
    while (validMove(x2, y2, nCols, nRows, grid, visited)) {
      switch (i) {
        case east:
          x2++;
          break;
        case west:
          x2--;
          break;
        case north:
          y2++;
          break;
        case south:
          y2--;
          break;
        default:
          break;
      }
      free_space_in_dir[i]++;
    }
  }
  // set initial direction towards direction with most travel possible
  int robot_dir = 0;
  int pattern_dir = point;
  int indexValue = 0;
  for (int i = 1; i <= 4; i++) {
      if (free_space_in_dir[i] > indexValue) {
          robot_dir = i;
          indexValue = free_space_in_dir[i];
      }
  }
  // set dx and dy based on robot_dir
  switch(robot_dir) {
    case east: // 1
      dx = +1;
      dy = 0;
      break;
    case west: // 2
      dx = -1;
      dy = 0;
      break;
    case north: // 3
      dx = 0;
      dy = +1;
      break;
    case south: // 4
      dx = 0;
      dy = -1;
      break;
    default:
      break;
  }

  bool done = false;
  while (!done)
  {
    // 1. drive straight until hit wall
    bool hitWall = false;
    while(!hitWall) {
      x2 = pathNodes.back().pos.x + dx;
      y2 = pathNodes.back().pos.y + dy;
      if (!validMove(x2, y2, nCols, nRows, grid, visited))        
      {
        hitWall = true;
        x2 = pathNodes.back().pos.x;
        y2 = pathNodes.back().pos.y;
        break;
      }
      if (!hitWall) {
          Point_t new_point = { x2, y2 };
          gridNode_t new_node =
          {
            new_point,  // Point: x,y
            0,          // Cost
            0,          // Heuristic
          };
          pathNodes.push_back(new_node);
          visited[y2][x2] = eNodeVisited;  // Close node
      }
    }

    // 2. check left and right after hitting wall, then change direction
    if (robot_dir == north || robot_dir == south)
    {
      if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited)
        && !validMove(x2 - 1, y2, nCols, nRows, grid, visited)) {
        // dead end, exit
        done = true;
        break;
      } else if (!validMove(x2 + 1, y2, nCols, nRows, grid, visited)) {
        // east is occupied, travel towards west
        x2--;
        pattern_dir = west;
      } else if (!validMove(x2 - 1, y2, nCols, nRows, grid, visited)) {
        // west is occupied, travel towards east
        x2++;
        pattern_dir = east;
      } else {
        // both sides are opened. If don't have a prefered turn direction, travel towards farthest edge
        if (!(pattern_dir == east || pattern_dir == west)) {
          if (x2 < nCols / 2) { // east
            pattern_dir = east;
          } else { // west
            pattern_dir = west;
          }
        }
        if (pattern_dir = east) {
            x2++;
        } else if (pattern_dir = west) {
            x2--;
        }
      }

      // add Nodes to List
      Point_t new_point = { x2, y2 };
      gridNode_t new_node =
      {
        new_point,  // Point: x,y
        0,          // Cost
        0,          // Heuristic
      };
      pathNodes.push_back(new_node);
      visited[y2][x2] = eNodeVisited;  // Close node

      // change direction 180 deg
      if (robot_dir == north) {
        robot_dir = south;
        dy = -1;
      } else if (robot_dir == south) {
        robot_dir = north;
        dy = 1;
      }
    }
    else if (robot_dir == east || robot_dir == west)
    {
      if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited)
        && !validMove(x2, y2 - 1, nCols, nRows, grid, visited)) {
        // dead end, exit
        done = true;
        break;
      } else if (!validMove(x2, y2 + 1, nCols, nRows, grid, visited)) {
        // north is occupied, travel towards south
        y2--;
        pattern_dir = south;
      } else if (!validMove(x2, y2 - 1, nCols, nRows, grid, visited)) {
        // south is occupied, travel towards north
        y2++;
        pattern_dir = north;
      } else {
        // both sides are opened. If don't have a prefered turn direction, travel towards farthest edge
        if (!(pattern_dir == north || pattern_dir == south)) {
          if (y2 < nRows / 2) { // north
            pattern_dir = north;
          } else { // south
            pattern_dir = south;
          }
        }
        if (pattern_dir = north) {
            y2++;
        } else if (pattern_dir = south) {
            y2--;
        }
      }

      // add Nodes to List
      Point_t new_point = { x2, y2 };
      gridNode_t new_node =
      {
        new_point,  // Point: x,y
        0,          // Cost
        0,          // Heuristic
      };
      pathNodes.push_back(new_node);
      visited[y2][x2] = eNodeVisited;  // Close node
      
      // change direction 180 deg
      if (robot_dir == east) {
        robot_dir = west;
        dx = -1;
      } else if (robot_dir == west) {
        robot_dir = east;
        dx = 1;
      }
    }
    // Log
    printPathNodes(pathNodes);
  }
  return pathNodes;
}

std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool> > const& grid,
                                          Point_t& init,
                                          int &multiple_pass_counter,
                                          int &visited_counter)
{
  int x, y, nRows = grid.size(), nCols = grid[0].size();
  // Initial node is initially set as visited so it does not count
  multiple_pass_counter = 0;
  visited_counter = 0;

  std::vector<std::vector<bool> > visited;
  visited = grid;  // Copy grid matrix
  x = init.x;
  y = init.y;

  // add initial point to pathNodes
  Point_t new_point = { x, y };
  gridNode_t new_node =
  {
    new_point,  // Point: x,y
    0,          // Cost
    0,          // Heuristic
  };
  std::list<gridNode_t> pathNodes;
  std::list<Point_t> fullPath;
  pathNodes.push_back(new_node);
  visited[y][x] = eNodeVisited;

  std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);  // Retrieve all goalpoints (Cells not visited)
  std::list<gridNode_t>::iterator it;

#ifdef DEBUG_PLOT
  ROS_INFO("Grid before walking is: ");
  printGrid(grid, visited, fullPath);
#endif

  while (goals.size() != 0)
  {
    // Spiral fill from current position
    // TODO: Convert to U-turn pattern
    pathNodes = spiral(grid, pathNodes, visited);

#ifdef DEBUG_PLOT
    ROS_INFO("Visited grid updated after spiral:");
    printGrid(grid, visited, pathNodes, SpiralStart, pathNodes.back());
#endif

    goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints

    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      Point_t newPoint = { it->pos.x, it->pos.y };
      visited_counter++;
      fullPath.push_back(newPoint);
    }

    // Remove all elements from pathNodes list except last element.
    // The last point is the starting point for a new search and A* extends the path from there on
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
    visited_counter--;  // First point is already counted as visited
    // Plan to closest open Node using A*
    // `goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
    //    to the nearest free space
    bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
    if (resign)
    {
#ifdef DEBUG_PLOT
      ROS_INFO("A_star_to_open_space is resigning", goals.size());
#endif
      break;
    }

    // Update visited grid
    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      if (visited[it->pos.y][it->pos.x])
      {
        multiple_pass_counter++;
      }
      visited[it->pos.y][it->pos.x] = eNodeVisited;
    }
    if (pathNodes.size() > 0)
    {
      multiple_pass_counter--;  // First point is already counted as visited
    }

#ifdef DEBUG_PLOT
    ROS_INFO("Grid with path marked as visited is:");
    gridNode_t SpiralStart = pathNodes.back();
    printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif
    
  }

  return fullPath;
}

bool SpiralSTC::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  else
  {
    ROS_INFO("Initialized!");
  }

  clock_t begin = clock();
  Point_t startPoint;

  /********************** Get grid from server **********************/
  std::vector<std::vector<bool> > grid;
  nav_msgs::GetMap grid_req_srv;
  ROS_INFO("Requesting grid!!");
  if (!cpp_grid_client_.call(grid_req_srv))
  {
    ROS_ERROR("Could not retrieve grid from map_server");
    return false;
  }

  if (!parseGrid(grid_req_srv.response.map, grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
  {
    ROS_ERROR("Could not parse retrieved grid");
    return false;
  }

#ifdef DEBUG_PLOT
  ROS_INFO("Start grid is:");
  std::list<Point_t> printPath;
  printPath.push_back(startPoint);
  printGrid(grid, grid, printPath);
#endif

  std::list<Point_t> goalPoints = spiral_stc(grid,
                                              startPoint,
                                              spiral_cpp_metrics_.multiple_pass_counter,
                                              spiral_cpp_metrics_.visited_counter);
  ROS_INFO("naive cpp completed!");
  ROS_INFO("Converting path to plan");

  parsePointlist2Plan(start, goalPoints, plan);
  // Print some metrics:
  spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter
                                            - spiral_cpp_metrics_.multiple_pass_counter;
  spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
  ROS_INFO("Total visited: %d", spiral_cpp_metrics_.visited_counter);
  ROS_INFO("Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
  ROS_INFO("Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
  ROS_INFO("Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

  // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
  // (also controlled by planner_frequency parameter in move_base namespace)

  ROS_INFO("Publishing plan!");
  publishPlan(plan);
  ROS_INFO("Plan published!");
  ROS_DEBUG("Plan published");

  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  std::cout << "elapsed time: " << elapsed_secs << "\n";

  return true;
}
}  // namespace full_coverage_path_planner
