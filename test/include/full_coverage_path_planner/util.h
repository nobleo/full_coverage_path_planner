//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 27-9-18.
//

#include <stdlib.h>
#include <time.h>
#include <vector>
#include <full_coverage_path_planner/common.h>

#ifndef FULL_COVERAGE_PATH_PLANNER_UTIL_H
#define FULL_COVERAGE_PATH_PLANNER_UTIL_H
/**
 * Create a X*Y grid
 *
 * @param x number of elements in horizontal direction (columns)
 * @param y number of elements in vertical direction (rows)
 * @param fill what to fill the rows with?
 * @return a vector of vectors. The inner vector has size x, the outer vector contains y x-sized vectors
 */
std::vector<std::vector<bool> > makeTestGrid(int x, int y, bool fill = false);
/**
 * Fill a test grid with a fraction of random obstacles
 * @param grid a vector of vectors that will be modified to have obstacles (true elements) in random places
 * @param obstacle_fraction between 0 and 100, what is the percentage of cells that must be marked as obstacle
 * @return bool indicating success
 */
bool randomFillTestGrid(std::vector<std::vector<bool> > &grid, float obstacle_fraction);

bool operator==(const Point_t &lhs, const Point_t &rhs);

struct CompareByPosition
{
  bool operator()(const Point_t &lhs, const Point_t &rhs)
  {
    if (lhs.x != rhs.x)
    {
      return lhs.x < rhs.x;
    }
    return lhs.y < rhs.y;
  }
};

#endif  // FULL_COVERAGE_PATH_PLANNER_UTIL_H
