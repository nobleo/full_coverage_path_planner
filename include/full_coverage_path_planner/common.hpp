//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 6-9-18.
//
#pragma once

#include <climits>
#include <fstream>
#include <list>
#include <vector>

typedef struct
{
  int x, y;
}
Point_t;

inline std::ostream & operator<<(std::ostream & os, Point_t & p)
{
  return os << "(" << p.x << ", " << p.y << ")";
}

typedef struct
{
  Point_t pos;
  int cost;  // Path cost: cost of the path from the start node to gridNode_t
  int he;  // Heuristic cost: cost of the cheapest path from this gridNode_t to the goal
}
gridNode_t;

inline std::ostream & operator<<(std::ostream & os, gridNode_t & g)
{
  return os << "gridNode_t(" << g.cost << ", " << g.he << ", " << g.pos << ")";
}

typedef struct
{
  double x, y;
}
dPoint_t;

inline std::ostream & operator<<(std::ostream & os, dPoint_t & p)
{
  return os << "(" << p.x << ", " << p.y << ")";
}

enum
{
  eNodeOpen = false,
  eNodeVisited = true
};

/**
 * Find the distance from poi to the closest point in goals
 * @param poi Starting point
 * @param goals Potential next points to find the closest of
 * @return Distance to the closest point (out of 'goals') to 'poi'
 */
int distanceToClosestPoint(Point_t poi, std::list<Point_t> const & goals);

/**
 * Calculate the distance between two points, squared
 */
int distanceSquared(const Point_t & p1, const Point_t & p2);

/**
 * Convert 2D grid of bools to a list of Point_t
 * @param grid 2D grid representing a map
 * @param value_to_search points matching this value will be returned
 * @return a list of points that have the given value_to_search
 */
std::list<Point_t> retrieveGoalsFromMap(std::vector<std::vector<bool>> const & grid, bool value_to_search);
