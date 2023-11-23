//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 6-9-18.
//
#include <climits>
#include <fstream>
#include <list>
#include <vector>

#ifndef FULL_COVERAGE_PATH_PLANNER_COMMON_H
#define FULL_COVERAGE_PATH_PLANNER_COMMON_H

typedef struct
{
  int x, y;
}
Point_t;

inline std::ostream &operator << (std::ostream &os, Point_t &p)
{
  return os << "(" << p.x << ", " << p.y << ")";
}

typedef struct
{
  Point_t pos;

  /** Path cost
   * cost of the path from the start node to gridNode_t
   */
  int cost;

  /** Heuristic cost
   * cost of the cheapest path from this gridNode_t to the goal
   */
  int he;
}
gridNode_t;

inline std::ostream &operator << (std::ostream &os, gridNode_t &g)
{
  return os << "gridNode_t(" << g.cost << ", " << g.he << ", " << g.pos  << ")";
}


typedef struct
{
  float x, y;
}
fPoint_t;

inline std::ostream &operator << (std::ostream &os, fPoint_t &p)
{
  return os << "(" << p.x << ", " << p.y << ")";
}

enum
{
  eNodeOpen = false,
  eNodeVisited = true
};

enum
{
  point = 0,
  east = 1,
  west = 2,
  north = 3,
  south = 4
};

/**
 * Find the distance from poi to the closest point in goals
 * @param poi Starting point
 * @param goals Potential next points to find the closest of
 * @return Distance to the closest point (out of 'goals') to 'poi'
 */
int distanceToClosestPoint(Point_t poi, std::list<Point_t> const &goals);

/**
 * Calculate the distance between two points, squared
 */
int distanceSquared(const Point_t &p1, const Point_t &p2);

/**
 * Perform A* shorted path finding from init to one of the points in heuristic_goals
 * @param grid 2D grid of bools. true == occupied/blocked/obstacle
 * @param init start position
 * @param cost cost of traversing a free node
 * @param visited grid 2D grid of bools. true == visited
 * @param open_space Open space that A* need to find a path towards. Only used for the heuristic and directing search
 * @param pathNodes nodes that form the path from init to the closest point in heuristic_goals
 * @return whether we resign from finding a path or not. true is we resign and false if we found a path
 */
bool a_star_to_open_space(std::vector<std::vector<bool> > const &grid, gridNode_t init, int cost,
                          std::vector<std::vector<bool> > &visited, std::list<Point_t> const &open_space,
                          std::list<gridNode_t> &pathNodes);

/**
 * Print a grid according to the internal representation
 * @param grid
 * @param visited
 * @param fullPath
 */
void printGrid(std::vector<std::vector<bool> > const& grid,
               std::vector<std::vector<bool> > const& visited,
               std::list<Point_t> const& path);

/**
 * Print a grid according to the internal representation
 * @param grid
 * @param visited
 * @param fullPath
 * @param start
 * @param end
 */
void printGrid(std::vector<std::vector<bool> > const& grid,
               std::vector<std::vector<bool> > const& visited,
               std::list<gridNode_t> const& path,
               gridNode_t start,
               gridNode_t end);

/**
 * Print a 2D array of bools to stdout
 */
void printGrid(std::vector<std::vector<bool> > const& grid);

/**
 * Convert 2D grid of bools to a list of Point_t
 * @param grid 2D grid representing a map
 * @param value_to_search points matching this value will be returned
 * @return a list of points that have the given value_to_search
 */
std::list<Point_t> map_2_goals(std::vector<std::vector<bool> > const& grid, bool value_to_search);

/**
 * Prints pathNodes in the terminal
 * @param pathNodes pathNodes to be printed in the terminal
 */
void printPathNodes(std::list<gridNode_t> pathNodes);

/**
 * returns true only if the desired move is valid
 * @param x2 x coordinate of desired position
 * @param y2 y coordinate of desired position
 * @param nCols
 * @param nRows 
 * @param grid internal map representation - 2D grid of bools. true == occupied/blocked/obstacle
 * @param visited 2D grid of bools. true == visited
 */
bool validMove(int x2, int y2, int nCols, int nRows,
               std::vector<std::vector<bool> > const& grid,
               std::vector<std::vector<bool> > const& visited);

/**
 * Adds node in (x2, y2) into the list of pathNodes, and marks the node as visited
 */
void addNodeToList(int x2, int y2, std::list<gridNode_t>& pathNodes,
                   std::vector<std::vector<bool>>& visited);

/**
 * Returns direction in which most free space is visible when given the robot's current location
 * @param ignoreDir ignores a single direction specified. Pass 0 (point) to consider all four directions.
 */
int dirWithMostSpace(int x2, int y2, int nCols, int nRows,
                     std::vector<std::vector<bool> > const& grid,
                     std::vector<std::vector<bool> > const& visited,
                     int ignoreDir);

#endif  // FULL_COVERAGE_PATH_PLANNER_COMMON_H
