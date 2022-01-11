//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <limits>
#include <list>
#include <vector>

#include <full_coverage_path_planner/common.h>

int distanceToClosestPoint(Point_t poi, std::list<Point_t> const& goals)
{
  // Return minimum distance from goals-list
  int min_dist = INT_MAX;
  std::list<Point_t>::const_iterator it;
  for (it = goals.begin(); it != goals.end(); ++it)
  {
    int cur_dist = distanceSquared((*it), poi);
    if (cur_dist < min_dist)
    {
      min_dist = cur_dist;
    }
  }
  return min_dist;
}

int distanceSquared(const Point_t& p1, const Point_t& p2)
{
  int dx = p2.x - p1.x;
  int dy = p2.y - p1.y;

  int dx2 = dx * dx;
  if (dx2 != 0 && dx2 / dx != dx)
  {
    throw std::range_error("Integer overflow error for the given points");
  }

  int dy2 = dy * dy;
  if (dy2 != 0 && dy2 / dy != dy)
  {
    throw std::range_error("Integer overflow error for the given points");
  }

  if (dx2 > std::numeric_limits< int >::max() - dy2)
    throw std::range_error("Integer overflow error for the given points");
  int d2 = dx2 + dy2;

  return d2;
}

/**
 * Sort vector<gridNode> by the heuristic value of the last element
 * @return whether last elem. of first has a larger heuristic value than last elem of second
 */
bool sort_gridNodePath_heuristic_desc(const std::vector<gridNode_t> &first, const std::vector<gridNode_t> &second)
{
  return (first.back().he > second.back().he);
}

bool a_star_to_open_space(std::vector<std::vector<bool> > const &grid, gridNode_t init, int cost,
                          std::vector<std::vector<bool> > &visited, std::list<Point_t> const &open_space,
                          std::list<gridNode_t> &pathNodes)
{
  uint dx, dy, dx_prev, nRows = grid.size(), nCols = grid[0].size();

  std::vector<std::vector<bool> > closed(nRows, std::vector<bool>(nCols, eNodeOpen));
  // All nodes in the closest list are currently still open

  closed[init.pos.y][init.pos.x] = eNodeVisited;  // Of course we have visited the current/initial location
#ifdef DEBUG_PLOT
  std::cout << "A*: Marked init " << init << " as eNodeVisited (true)" << std::endl;
  printGrid(closed);
#endif

  std::vector<std::vector<gridNode_t> > open1(1, std::vector<gridNode_t>(1, init));  // open1 is a *vector* of paths

  while (true)
  {
#ifdef DEBUG_PLOT
    std::cout << "A*: open1.size() = " << open1.size() << std::endl;
#endif
    if (open1.size() == 0)  // If there are no open paths, there's no place to go and we must resign
    {
      // Empty end_node list and add init as only element
      pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));
      pathNodes.push_back(init);
      return true;  // We resign, cannot find a path
    }
    else
    {
      // Sort elements from high to low (because sort_gridNodePath_heuristic_desc uses a > b)
      std::sort(open1.begin(), open1.end(), sort_gridNodePath_heuristic_desc);

      std::vector<gridNode_t> nn = open1.back();  // Get the *path* with the lowest heuristic cost
      open1.pop_back();  // The last element is no longer open because we use it here, so remove from open list
#ifdef DEBUG_PLOT
      std::cout << "A*: Check out path from" << nn.front().pos << " to " << nn.back().pos
      << " of length " << nn.size() << std::endl;
#endif

      // Does the path nn end in open space?
      if (visited[nn.back().pos.y][nn.back().pos.x] == eNodeOpen)
      {
        // If so, we found a path to open space
        // Copy the path nn to pathNodes so we can report that path (to get to open space)
        std::vector<gridNode_t>::iterator iter;
        for (iter = nn.begin(); iter != nn.end(); ++iter)
        {
          pathNodes.push_back((*iter));
        }

        return false;  // We do not resign, we found a path
      }
      else  // Path nn does not lead to open space
      {
        if (nn.size() > 1)
        {
          // Create iterator for gridNode_t list and let it point to the last element of nn
          std::vector<gridNode_t>::iterator it = --(nn.end());
          dx = it->pos.x - (it - 1)->pos.x;
          dy = it->pos.y - (it - 1)->pos.y;
          // TODO(CesarLopez) docs: this seems to cycle through directions
          // (notice the shift-by-one between both sides of the =)
          dx_prev = dx;
          dx = -dy;
          dy = dx_prev;
        }
        else
        {
          dx = 0;
          dy = 1;
        }

        // For all nodes surrounding the end of the end of the path nn
        for (uint i = 0; i < 4; ++i)
        {
          Point_t p2 =
          {
            static_cast<int>(nn.back().pos.x + dx),
            static_cast<int>(nn.back().pos.y + dy),
          };

#ifdef DEBUG_PLOT
          std::cout << "A*: Look around " << i << " at p2=(" << p2 << std::endl;
#endif

          if (p2.x >= 0 && p2.x < nCols && p2.y >= 0 && p2.y < nRows)  // Bounds check, do not sep out of map
          {
            // If the new node (a neighbor of the end of the path nn) is open, append it to newPath ( = nn)
            // and add that to the open1-list of paths.
            // Because of the pop_back on open1, what happens is that the path is temporarily 'checked out',
            // modified here, and then added back (if the condition above and below holds)
            if (closed[p2.y][p2.x] == eNodeOpen && grid[p2.y][p2.x] == eNodeOpen)
            {
#ifdef DEBUG_PLOT
              std::cout << "A*: p2=" << p2 << " is OPEN" << std::endl;
#endif
              std::vector<gridNode_t> newPath = nn;
              // # heuristic  has to be designed to prefer a CCW turn
              Point_t new_point = { p2.x, p2.y };
              gridNode_t new_node =
              {
                new_point,                                                                             // Point: x,y
                static_cast<int>(cost + nn.back().cost),                                               // Cost
                static_cast<int>(cost + nn.back().cost + distanceToClosestPoint(p2, open_space) + i),
                // Heuristic (+i so CCW turns are cheaper)
              };
              newPath.push_back(new_node);
              closed[new_node.pos.y][new_node.pos.x] = eNodeVisited;  // New node is now used in a path and thus visited

#ifdef DEBUG_PLOT
              std::cout << "A*: Marked new_node " << new_node << " as eNodeVisited (true)" << std::endl;
              std::cout << "A*: Add path from " << newPath.front().pos << " to " << newPath.back().pos
              << " of length " << newPath.size() << " to open1" << std::endl;
#endif
              open1.push_back(newPath);
            }
#ifdef DEBUG_PLOT
            else
            {
              std::cout << "A*: p2=" << p2 << " is not open: "
                        "closed[" << p2.y << "][" << p2.x << "]=" << closed[p2.y][p2.x] << ", "
                        "grid["  << p2.y << "][" << p2.x << "]=" << grid[p2.y][p2.x] << std::endl;
            }
#endif
          }
#ifdef DEBUG_PLOT
          else
          {
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

void printGrid(std::vector<std::vector<bool> > const& grid, std::vector<std::vector<bool> > const& visited,
               std::list<Point_t> const& path)
{
  for (uint iy = grid.size() - 1; iy >= 0; --iy)
  {
    for (uint ix = 0; ix < grid[0].size(); ++ix)
    {
      if (visited[iy][ix])
      {
        if (ix == path.front().x && iy == path.front().y)
        {
          std::cout << "\033[1;32m▓\033[0m";  // Show starting position in green color
        }
        else if (ix == path.back().x && iy == path.back().y)
        {
          std::cout << "\033[1;31m▓\033[0m";  // Show stopping position in red color
        }
        else if (visited[iy][ix] && grid[iy][ix])
        {
          std::cout << "\033[1;33m▓\033[0m";  // Show walls in yellow color
        }
        else
        {
          std::cout << "\033[1;36m▓\033[0m";
        }
      }
      else
      {
        std::cout << "\033[1;37m▓\033[0m";
      }
    }
    std::cout << "\n";
  }
}

void printGrid(std::vector<std::vector<bool> > const& grid, std::vector<std::vector<bool> > const& visited,
               std::list<gridNode_t> const& path, gridNode_t start, gridNode_t end)
{
  for (uint iy = grid.size() - 1; iy >= 0; --iy)
  {
    for (uint ix = 0; ix < grid[0].size(); ++ix)
    {
      if (visited[iy][ix])
      {
        if (ix == start.pos.x && iy == start.pos.y)
        {
          std::cout << "\033[1;32m▓\033[0m";  // Show starting position in green color
        }
        else if (ix == end.pos.x && iy == end.pos.y)
        {
          std::cout << "\033[1;31m▓\033[0m";  // Show stopping position in red color
        }
        else if (visited[iy][ix] && grid[iy][ix])
        {
          std::cout << "\033[1;33m▓\033[0m";  // Show walls in yellow color
        }
        else
        {
          std::cout << "\033[1;36m▓\033[0m";
        }
      }
      else
      {
        std::cout << "\033[1;37m▓\033[0m";
      }
    }
    std::cout << "\n";
  }
}

void printGrid(std::vector<std::vector<bool> > const& grid)
{
  for (uint iy = grid.size() - 1; iy >= 0; --iy)
  {
    for (uint ix = 0; ix < grid[0].size(); ++ix)
    {
      if (grid[iy][ix])
      {
        std::cout << "\033[1;36m▓\033[0m";
      }
      else
      {
        std::cout << "\033[1;37m▓\033[0m";
      }
    }
    std::cout << "\n";
  }
}

std::list<Point_t> map_2_goals(std::vector<std::vector<bool> > const& grid, bool value_to_search)
{
  std::list<Point_t> goals;
  int ix, iy;
  uint nRows = grid.size();
  uint nCols = grid[0].size();
  for (iy = 0; iy < nRows; ++(iy))
  {
    for (ix = 0; ix < nCols; ++(ix))
    {
      if (grid[iy][ix] == value_to_search)
      {
        Point_t p = { ix, iy };  // x, y
        goals.push_back(p);
      }
    }
  }
  return goals;
}
