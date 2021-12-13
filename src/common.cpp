//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <limits>
#include <list>
#include <vector>

#include <full_coverage_path_planner/common.hpp>

int distanceToClosestPoint(Point_t poi, std::list<Point_t> const & goals)
{
  int min_dist = INT_MAX;
  std::list<Point_t>::const_iterator it;
  for (it = goals.begin(); it != goals.end(); it++) {
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

std::list<Point_t> retrieveGoalsFromMap(
  std::vector<std::vector<bool>> const & grid, bool value_to_search)
{
  std::list<Point_t> goals;
  int ix, iy, n_rows = grid.size(), n_cols = grid[0].size();
  for (iy = 0; iy < n_rows; iy++) {
    for (ix = 0; ix < n_cols; ix++) {
      if (grid[iy][ix] == value_to_search) {
        Point_t p = {ix, iy};
        goals.push_back(p);
      }
    }
  }
  return goals;
}
