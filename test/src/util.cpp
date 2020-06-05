//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 27-9-18.
//

#include <vector>
#include <full_coverage_path_planner/util.h>

std::vector<std::vector<bool> > makeTestGrid(int x, int y, bool fill)
{
  std::vector<std::vector<bool> > grid;
  for (int i = 0; i < y; ++i)
  {
    std::vector<bool> row;
    for (int j = 0; j < x; ++j)
    {
      row.push_back(fill);
    }
    grid.push_back(row);
  }
  return grid;
}

bool operator==(const Point_t &lhs, const Point_t &rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool randomFillTestGrid(std::vector<std::vector<bool> > &grid, float obstacle_fraction)
{
  unsigned int seed = time(NULL);
  int max_y = grid.size();
  if (max_y < 1)
  {
    // Cannot work on less than a row
    return false;
  }
  int max_x = grid.front().size();
  if (max_x < 1)
  {
    // Cannot work on less than a column
    return false;
  }

  int total_cells = max_y * max_x;
  int total_obstacles = total_cells * (obstacle_fraction / 100);
//  std::cout << "total_cells: " << total_cells << ", total_obstacles: " << total_obstacles << std::endl;

  // For the amount of obstacles we need to create, generate random coordinates and insert an obstacle
  for (int i = 0; i < total_obstacles; ++i)
  {
    int x = rand_r(&seed) % max_x;
    int y = rand_r(&seed) % max_y;
//    std::cout << "Obstacle at (" << x << ", " << y << ")" << std::endl;
    grid[y][x] = true;
  }
}
