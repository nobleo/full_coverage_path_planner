//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <limits>
#include <list>
#include <vector>

#include <full_coverage_path_planner/common.hpp>

#define DEBUG_PLOT

int distanceToClosestPoint(Point_t poi, std::list<Point_t> const & goals)
{
  // Return minimum distance from goals-list
  int min_dist = INT_MAX;
  std::list<Point_t>::const_iterator it;
  for (it = goals.begin(); it != goals.end(); ++it) {
    int cur_dist = distanceSquared((*it), poi);
    if (cur_dist < min_dist) {
      min_dist = cur_dist;
    }
  }
  return min_dist;
}

int distanceSquared(const Point_t & p1, const Point_t & p2)
{
  int dx = p2.x - p1.x;
  int dy = p2.y - p1.y;

  int dx2 = dx * dx;
  if (dx2 != 0 && dx2 / dx != dx) {
    throw std::range_error("Integer overflow error for the given points");
  }

  int dy2 = dy * dy;
  if (dy2 != 0 && dy2 / dy != dy) {
    throw std::range_error("Integer overflow error for the given points");
  }

  if (dx2 > std::numeric_limits<int>::max() - dy2) {
    throw std::range_error("Integer overflow error for the given points");
  }
  int d2 = dx2 + dy2;

  return d2;
}

/**
 * Sort vector<gridNode> by the heuristic value of the last element
 * @return whether last elem. of first has a larger heuristic value than last elem of second
 */
bool sort_gridNodePath_heuristic_desc(
  const std::vector<gridNode_t> & first, const std::vector<gridNode_t> & second)
{
  return first.back().he > second.back().he;
}

bool planAStarToOpenSpace(
  std::vector<std::vector<bool>> const & grid, gridNode_t init, int cost,
  std::vector<std::vector<bool>> & visited, std::list<Point_t> const & open_space,
  std::list<gridNode_t> & path_nodes)
{
  int dx, dy, dx_prev, n_rows = grid.size(), n_cols = grid[0].size();

  std::vector<std::vector<bool>> closed(n_rows, std::vector<bool>(n_cols, eNodeOpen));
  // All nodes in the closed list are currently still open

  closed[init.pos.y][init.pos.x] = eNodeVisited;  // Of course we have visited the current/initial location

  #ifdef DEBUG_PLOT
  std::cout << "A*: Marked init " << init << " as eNodeVisited (true)" << std::endl;
  #endif

  std::vector<std::vector<gridNode_t>> open1(1, std::vector<gridNode_t>(1, init));  // open1 is a *vector* of paths

  while (true) { // Keep searching until either a path is found or no path can be found

    #ifdef DEBUG_PLOT
    std::cout << "A*: open1.size() = " << open1.size() << std::endl;
    #endif

    if (open1.size() == 0) {  // If there are no open paths, there's no place to go and we must resign
      // Empty end_node list and add init as only element
      path_nodes.erase(path_nodes.begin(), --(path_nodes.end()));
      path_nodes.push_back(init);
      return true;  // We resign, cannot find a path
    } else {
      // Sort elements from high to low (because sort_gridNodePath_heuristic_desc uses a > b)
      std::sort(open1.begin(), open1.end(), sort_gridNodePath_heuristic_desc);  // Sort bases on heuristic costs
      std::vector<gridNode_t> nn = open1.back();  // Get the *path* with currently the lowest heuristic cost
      open1.pop_back();  // The last element is no longer open because we use it here, so remove from open list

      #ifdef DEBUG_PLOT
      std::cout << "A*: Check out path from " << nn.front().pos << " to " << nn.back().pos <<
        " of length " << nn.size() << std::endl;
      #endif

      // Does the path nn end in open space?
      if (visited[nn.back().pos.y][nn.back().pos.x] == eNodeOpen) {
        // If so, we found a path to open space
        // Copy the path nn to path_nodes so we can report that path (to get to open space)
        std::vector<gridNode_t>::iterator iter;
        for (iter = nn.begin(); iter != nn.end(); ++iter) {
          path_nodes.push_back((*iter));
        }
        return false;  // We do not resign, we found a path
      } else {
        if (nn.size() > 1) {
          // Create iterator for gridNode_t list and let it point to the last element of nn
          std::vector<gridNode_t>::iterator it = --(nn.end());
          dx = it->pos.x - (it - 1)->pos.x;
          dy = it->pos.y - (it - 1)->pos.y;
          // TODO(CesarLopez) docs: this seems to cycle through directions
          // (notice the shift-by-one between both sides of the =)
          dx_prev = dx;
          dx = -dy;
          dy = dx_prev;
        } else {
          dx = 0;
          dy = 1;
        }

        // For all nodes surrounding the end of the path nn
        for (int i = 0; i < 4; ++i) {
          Point_t p2 = {nn.back().pos.x + dx, nn.back().pos.y + dy};

          #ifdef DEBUG_PLOT
          std::cout << "A*: Look around in direction " << i << " at p2=" << p2 << std::endl;
          #endif

          if (p2.x >= 0 && p2.x < n_cols && p2.y >= 0 && p2.y < n_rows) { // Bounds check, do not step out of map
            // If the new node (a neighbor of the end of the path nn) is open, append it to new_path ( = nn)
            // and add that to the open1-list of paths.
            // Because of the pop_back on open1, what happens is that the path is temporarily 'checked out',
            // modified here, and then added back (if the condition above and below holds)
            if (closed[p2.y][p2.x] == eNodeOpen && grid[p2.y][p2.x] == eNodeOpen) {

              #ifdef DEBUG_PLOT
              std::cout << "A*: p2=" << p2 << " is OPEN" << std::endl;
              #endif

              std::vector<gridNode_t> new_path = nn;
              // The heuristic has to be designed to prefer a CCW (counter-clockwise) turn
              Point_t new_point = {p2.x, p2.y};
              gridNode_t new_node =
              {
                new_point,  // Point: x, y
                cost + nn.back().cost,  // Cost
                cost + nn.back().cost + distanceToClosestPoint(p2, open_space) + i  // Heuristic (+i so CCW turns are cheaper)
              };
              new_path.push_back(new_node);
              closed[new_node.pos.y][new_node.pos.x] = eNodeVisited;  // New node is now used in a path and thus visited

              #ifdef DEBUG_PLOT
              std::cout << "A*: Marked new_node " << new_node << " as eNodeVisited (true)" <<
                std::endl;
              std::cout << "A*: Add path from " << new_path.front().pos << " to " <<
                new_path.back().pos << " of length " << new_path.size() << " to open1" << std::endl;
              #endif

              open1.push_back(new_path);
            }

            #ifdef DEBUG_PLOT
            else {
              std::cout << "A*: p2=" << p2 << " is not open: "
                "closed[" << p2.y << "][" << p2.x << "]=" << closed[p2.y][p2.x] << ", "
                "grid[" << p2.y << "][" << p2.x << "]=" << grid[p2.y][p2.x] << std::endl;
            }
            #endif
          }

          #ifdef DEBUG_PLOT
          else {
            std::cout << "A*: p2=(" << p2.x << ", " << p2.y << ") is out of bounds" << std::endl;
          }
          #endif

          // Cycle around to next neighbor, CCW
          dx_prev = dx;
          dx = dy;
          dy = -dx_prev;
        }
      }
    }
  }
}

std::list<Point_t> retrieveGoalsFromMap(std::vector<std::vector<bool>> const & grid, bool value_to_search)
{
  std::list<Point_t> goals;
  int ix, iy;
  int n_rows = grid.size();
  int n_cols = grid[0].size();
  for (iy = 0; iy < n_rows; ++(iy)) {
    for (ix = 0; ix < n_cols; ++(ix)) {
      if (grid[iy][ix] == value_to_search) {
        Point_t p = {ix, iy};  // x, y
        goals.push_back(p);
      }
    }
  }
  return goals;
}
