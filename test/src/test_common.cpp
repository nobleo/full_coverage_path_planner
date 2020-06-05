//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
// Created by nobleo on 25-9-18.
//

/*
 * Run tests for all of the common function except the print* functions that do not return anything testable
 * Most important here is the conversion function and a variant of A*. Each test is explained below
 *
 */
#include <list>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <full_coverage_path_planner/common.h>
#include <full_coverage_path_planner/util.h>

/**
 * DistanceSquared uses euclidian distance except for the expensive sqrt-call: returns dx^2+dy^2.
 */
TEST(TestDistanceSquared, testDistanceSquared)
{
  // 1^2 + 1^2 = 1 + 1 = 2
  ASSERT_EQ(2, distanceSquared({0, 0}, {1, 1}));  // NOLINT

  // 10^2 + 10^2 = 100 + 100 = 200
  ASSERT_EQ(200, distanceSquared({0, 0}, {10, 10}));  // NOLINT

  /* The function uses plain integers. Squaring and then adding them can go out of range of int.
   * In that case: throw an exception, don't crash
   */
  ASSERT_THROW(distanceSquared({0, 0}, {100000, 100000}), std::range_error);  // NOLINT

  /* Points at equal distance in direct directions should have same value
   */
  ASSERT_EQ(distanceSquared({0, 0}, {11, 10}),  // NOLINT
            distanceSquared({0, 0}, {10, 11}));  // NOLINT

  /* The function is used mostly to order points by distance, so the actual value doesn't matter.
   * The only property that is important is that points further away should have a larger value than those close by
   */
  ASSERT_LE(distanceSquared({0, 0}, {10, 10}),  // NOLINT
            distanceSquared({0, 0}, {11, 11}));  // NOLINT
}

/*
 * Test basics of distanceToClosestPoint:
 * - does it return the only point if there is one only
 * - if we add another further away, is the first point still returned
 */
TEST(TestDistanceToClosestPoint, testDistanceToOnlyPoint)
{
  Point_t poi = {0, 0};  // NOLINT
  std::list<Point_t> goals;
  goals.push_back({1, 1});  // NOLINT

  // There is only 1 point, at 1,1, so the (squared) distance is 2
  ASSERT_EQ(2, distanceToClosestPoint(poi, goals));

  goals.push_back({2, 2});  // NOLINT // We add a point that is further away, so the first point is still closest
  ASSERT_EQ(2, distanceToClosestPoint(poi, goals));
}

/*
 * Add several points, 2 at same distance
 */
TEST(TestDistanceToClosestPoint, testDistanceToEqualDistancePoints)
{
  Point_t poi = {0, 0};  // NOLINT
  std::list<Point_t> goals;
  goals.push_back({0, 1});  // NOLINT // closest, d=1
  goals.push_back({1, 0});  // NOLINT // closest, d=1
  goals.push_back({1, 1});  // NOLINT
  goals.push_back({2, 2});  // NOLINT

  ASSERT_EQ(1, distanceToClosestPoint(poi, goals));

  goals.push_back({0, 0});  // NOLINT // new closest, d=0
  ASSERT_EQ(0, distanceToClosestPoint(poi, goals));
}

/*
 * distanceToClosestPoint cannot deal with dimensions larger than 2^16 because it does a square of that value
 * Function should not crash if one distance is <2^16 but raise an exception if we go over
 */
TEST(TestDistanceToClosestPoint, testDistanceAtIntLimits)
{
  Point_t poi = {0, 0};  // NOLINT
  std::list<Point_t> goals;
  for (int i = 0; i < 32768; ++i)
  {
    goals.push_back({i, i});  // NOLINT
  }

  ASSERT_NO_THROW(distanceToClosestPoint(poi, goals));  // OK for small enough dimensions
  ASSERT_EQ(0, distanceToClosestPoint(poi, goals));

  for (int i = 32768; i < 100000; ++i)
  {
    goals.push_back({i, i});  // NOLINT
  }

  // Squaring and adding 100000 is too much for an int
  ASSERT_THROW(distanceToClosestPoint(poi, goals), std::range_error);  // Must throw for large enough dimensions
}

/*
 * Same as testDistanceAtIntLimits but with negative numbers
 */
TEST(TestDistanceToClosestPoint, testDistanceAtIntNegativeLimits)
{
  Point_t poi = {0, 0};  // NOLINT
  std::list<Point_t> goals;
  for (int i = 0; i < 32768; ++i)
  {
    goals.push_back({-i, -i});  // NOLINT
  }

  ASSERT_NO_THROW(distanceToClosestPoint(poi, goals));
  ASSERT_EQ(0, distanceToClosestPoint(poi, goals));

  for (int i = 32768; i < 100000; ++i)
  {
    goals.push_back({-i, -i});  // NOLINT
  }

  // Squaring and adding 100000 is too much for an int
  ASSERT_THROW(distanceToClosestPoint(poi, goals), std::range_error);
}

/*
 * Test points in a block a bit further away
 */
TEST(TestDistanceToClosestPoint, testDistanceToBlock)
{
  Point_t poi = {0, 0};  // NOLINT
  std::list<Point_t> goals;
  goals.push_back({10, 10});  // NOLINT // closest, d^2=200
  goals.push_back({11, 10});  // NOLINT
  goals.push_back({10, 11});  // NOLINT
  goals.push_back({12, 11});  // NOLINT

  ASSERT_EQ(200, distanceToClosestPoint(poi, goals));

  goals.push_back({0, 0});  // NOLINT // new closest, d=0
  ASSERT_EQ(0, distanceToClosestPoint(poi, goals));
}

/*
 * Test points in a stroke a bit further away
 */
TEST(TestDistanceToClosestPoint, testDistanceToDiagonal)
{
  Point_t poi = {100, 100};  // NOLINT
  std::list<Point_t> goals;
  for (int i = 0; i < 10; ++i)
  {
    for (int j = 0; j < 10; ++j)
    {
      goals.push_back({i, j});  // NOLINT
    }
  }

  // Closest point is 9,9, so distance is 91^2 + 91^2
  ASSERT_EQ(16562, distanceToClosestPoint(poi, goals));
}

/*
 * Test a test utility function to make grids.
 * Grids must all have specified size but now allow access beyond those limits.
 * Users of the resulting test grid are also not allowed to access beyond the limits
 *  and should crash if they do for proper testing.
 */
TEST(TestMakeTestGrid, testDimensions)
{
  std::vector<std::vector<bool> > grid = makeTestGrid(3, 4);
  ASSERT_EQ(4, grid.size());
  ASSERT_EQ(3, grid.at(0).size());
  ASSERT_EQ(3, grid.at(1).size());
  ASSERT_EQ(3, grid.at(2).size());
  ASSERT_EQ(3, grid.at(3).size());
  ASSERT_ANY_THROW(grid.at(3).at(3));  // Only 3 items in X direction (horizontal) so no index 3
  ASSERT_ANY_THROW(grid.at(4).size());  // Only 4 items in Y direction (vertical) so no index 4
}

/*
 * Test that if there is a NxN map with only a single element, only that single element is returned
 */
TEST(TestMap_2_goals, testFindSingle)
{
/* Map will be 3x3 with only the middle value set to true, the others being false
 *
 * [ 0 0 0 ]
 * [ 0 1 0 ]
 * [ 0 0 0 ]
 */
  std::vector<std::vector<bool> > grid = makeTestGrid(3, 3);
  grid[1][1] = true;
  std::list<Point_t> goals;
  goals = map_2_goals(grid, true);
  ASSERT_EQ(1, goals.size());

  Point_t center = {1, 1};  // NOLINT
  ASSERT_EQ(center.x, goals.front().x);
  ASSERT_EQ(center.y, goals.front().y);
}

/*
 * Test that if there is a 3x3 map with 3 elements, that those elements and no more are returned
 */
TEST(TestMap_2_goals, testNumberOfGoals)
{
/* Map:
 * [ 1 0 0 ]
 * [ 0 1 0 ]
 * [ 0 0 1 ]
 */
  std::vector<std::vector<bool> > grid = makeTestGrid(3, 3);
  grid[0][0] = true;
  grid[1][1] = true;
  grid[2][2] = true;
  std::list<Point_t> goals;
  goals = map_2_goals(grid, true);
  std::vector<Point_t> goalVector = std::vector<Point_t>(goals.begin(), goals.end());

  // We only set 3 points to 1, so we should only find those 3
  ASSERT_EQ(3, goals.size());

  // And specifically those 3, not anything else
  Point_t corner0 = {0, 0};  // NOLINT
  Point_t center = {1, 1};  // NOLINT
  Point_t corner2 = {2, 2};  // NOLINT
  ASSERT_EQ(corner0, goalVector.at(0));
  ASSERT_EQ(center, goalVector.at(1));
  ASSERT_EQ(corner2, goalVector.at(2));
}

/*
 * map_2_goals can look for both true and false cells.
 * In either case, we should return the correct number of goals
 */
TEST(TestMap_2_goals, testInvertedMap)
{
/* Map:
 * [ 1 1 1 ]
 * [ 1 0 1 ]
 * [ 1 1 1 ]
 */
  std::vector<std::vector<bool> > grid = makeTestGrid(3, 3, true);
  grid[1][1] = false;

  ASSERT_EQ(8, map_2_goals(grid, true).size());  // There are 8 true values
  ASSERT_EQ(1, map_2_goals(grid, false).size());  // There is only 1 false value
}

/*
 * If we use a non-square grid, do we use the correct order of indexing?
 * Coordinates use x,y and y indexes over rows, x over indices in that row
 */
TEST(TestMap_2_goals, testCoordinateOrder)
{
/* Map is 3x4
 * [ 0 0 0 ]
 * [ 0 0 0 ]
 * [ 0 0 0 ]
 * [ 0 0 1 ]
 *
 */
  std::vector<std::vector<bool> > grid = makeTestGrid(3, 4, false);
  grid.at(3).at(2) = true;
  std::list<Point_t> goals;

  goals = map_2_goals(grid, true);
  ASSERT_EQ(1, goals.size());

  Point_t corner0 = {2, 3};  // NOLINT
  ASSERT_EQ(corner0.x, goals.front().x);
  ASSERT_EQ(corner0.y, goals.front().y);
}

/* LEGENDA
 * Note: in tests for the A* path finding algorithm, use this legend for the maps:
 * s: start
 * p: path node
 * v: visited
 * 1: obstacle / occupied
 * 0: open node / unoccupied
 */

/*
 * On an empty map, find a route to open space should be as simple as stepping to the next position
 * (when the current position is already visited and thus not OK)
 */
TEST(TestAStarToOpenSpace, testEmptyMap)
{
  /*
   * [s] We start from here, so this is visited
   * [0] And this is still open, so we should go here to find open space
   * [0]
   * [0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(1, 4, false);
  std::vector<std::vector<bool> > visited = makeTestGrid(1, 4, false);
  std::list<Point_t> goals;
  goals.push_back({0, 3});  // NOLINT

  visited[0][0] = true;
  gridNode_t start;
  start.pos = {0, 0};  // NOLINT
  start.cost = 1;
  start.he = 0;

  std::list<gridNode_t> pathNodes;

  bool resign = a_star_to_open_space(grid,  // map to traverse, all empty
                                     start,  // Start
                                     1,  // Cost of traversing a node
                                     visited,  // Visited nodes, of which there are none yet
                                     goals,
                                     pathNodes);
  /*
   * [p] We came from here
   * [p] and this is the first unvisited, non-obstacle cell so we step here
   * [0]
   * [0]
   */

  std::vector<gridNode_t> pathNodesVector = std::vector<gridNode_t>(pathNodes.begin(), pathNodes.end());
  ASSERT_EQ(false, resign);

  // First element is that initial node
  ASSERT_EQ(0, pathNodesVector.at(0).pos.x);
  ASSERT_EQ(0, pathNodesVector.at(0).pos.y);

  // First step we take reaches free space
  ASSERT_EQ(0, pathNodesVector.at(1).pos.x);
  ASSERT_EQ(1, pathNodesVector.at(1).pos.y);

  // 2 nodes: start and end
  ASSERT_EQ(2, pathNodes.size());
}

/*
 * Small extension of testEmptyMap: there is one more cell already visited
 * That mean the path is thus from start, over the one visited cell to a new unvisited cell, so length 3
 */
TEST(TestAStarToOpenSpace, testSingleVisitedCellMap)
{
  /*
   * [s]
   * [v]
   * [0]
   * [0]
   */

  std::vector<std::vector<bool> > grid = makeTestGrid(1, 4, false);
  std::vector<std::vector<bool> > visited = makeTestGrid(1, 4, false);
  std::list<Point_t> goals = map_2_goals(grid, true);

  visited[0][0] = true;
  visited[1][0] = true;
  gridNode_t start;
  start.pos = {0, 0};  // NOLINT
  start.cost = 1;
  start.he = 0;

  std::list<gridNode_t> pathNodes;

  bool resign = a_star_to_open_space(grid,  // map to traverse, all empty
                                     start,  // Start
                                     1,  // Cost of traversing a node
                                     visited,  // Visited nodes, of which there are none yet
                                     goals,
                                     pathNodes);
  /*
   * [p]
   * [p]
   * [p]
   * [0]
   */
  std::vector<gridNode_t> pathNodesVector = std::vector<gridNode_t>(pathNodes.begin(), pathNodes.end());
  ASSERT_EQ(false, resign);

  ASSERT_EQ(0, pathNodesVector.at(0).pos.x);
  ASSERT_EQ(0, pathNodesVector.at(0).pos.y);

  ASSERT_EQ(0, pathNodesVector.at(1).pos.x);
  ASSERT_EQ(1, pathNodesVector.at(1).pos.y);

  ASSERT_EQ(0, pathNodesVector.at(2).pos.x);
  ASSERT_EQ(2, pathNodesVector.at(2).pos.y);

  ASSERT_EQ(3, pathNodes.size());
}

/*
 * When we start in the corner and have somehow visited the direct neighbors, the shortest path should involve 3 points
 */
TEST(TestAStarToOpenSpace, testSingleMulticellVisitedMap)
{
  /*
   * [s v 0 0]
   * [v v 0 0]
   * [0 0 0 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  std::vector<std::vector<bool> > visited = makeTestGrid(4, 4, false);
  std::list<Point_t> goals = map_2_goals(grid, true);

  visited[0][0] = true;
  visited[1][0] = true;
  visited[0][1] = true;
  visited[1][1] = true;

  gridNode_t start;
  start.pos = {0, 0};  // NOLINT
  start.cost = 1;
  start.he = 0;

  std::list<gridNode_t> pathNodes;

  bool resign = a_star_to_open_space(grid,  // map to traverse, all empty
                                     start,  // Start
                                     1,  // Cost of traversing a node
                                     visited,  // Visited nodes, of which there are none yet
                                     goals,
                                     pathNodes);
  /* Several paths possible, but each covers 3 nodes, e.g.
   * [p p p 0]
   * [v v 0 0]
   * [0 0 0 0]
   * [0 0 0 0]
   */
  std::vector<gridNode_t> pathNodesVector = std::vector<gridNode_t>(pathNodes.begin(), pathNodes.end());
  ASSERT_EQ(false, resign);
  ASSERT_EQ(3, pathNodes.size());
}

/*
 * In a map with 2 walls and everything between those wall already visited, test that the path has the expected length
 */
TEST(TestAStarToOpenSpace, testMazeMap)
{
  /*
   * [s v v v]
   * [1 1 1 v]
   * [0 v v v] goal is the single 0 in this map
   * [1 1 1 1]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[1][0] = 1;
  grid[1][1] = 1;
  grid[1][2] = 1;
  grid[3][0] = 1;
  grid[3][1] = 1;
  grid[3][2] = 1;
  grid[3][3] = 1;
  std::vector<std::vector<bool> > visited = makeTestGrid(4, 4, false);
  std::list<Point_t> goals = map_2_goals(grid, true);

  visited[0][0] = 1;
  visited[0][1] = 1;
  visited[0][2] = 1;
  visited[0][3] = 1;
  visited[1][3] = 1;
  visited[2][3] = 1;
  visited[2][2] = 1;
  visited[2][1] = 1;

  gridNode_t start;
  start.pos = {0, 0};  // NOLINT
  start.cost = 1;
  start.he = 0;

  std::list<gridNode_t> pathNodes;

  bool resign = a_star_to_open_space(grid,  // map to traverse, all empty
                                     start,  // Start
                                     1,  // Cost of traversing a node
                                     visited,  // Visited nodes, of which there are none yet
                                     goals,
                                     pathNodes);
  /*
   * [p p p p]
   * [1 1 1 p]
   * [p p p p]
   * [1 1 1 1]
   */
  ASSERT_EQ(false, resign);
  ASSERT_EQ(9, pathNodes.size());
}

/*
 * Extension of testMazeMap, but the bottom wall has a hole that is also not yet visited.
 * That is the closest open space, so check that the found path goes there.
 */
TEST(TestAStarToOpenSpace, testMazeWithHoleMap)
{
  /* There are 2 open spaces in this test, the one at 3,3 is closest
   * [s v v v]
   * [1 1 1 v]
   * [0 v v v]
   * [1 1 1 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[1][0] = 1;
  grid[1][1] = 1;
  grid[1][2] = 1;
  grid[3][0] = 1;
  grid[3][1] = 1;
  grid[3][2] = 1;
  std::vector<std::vector<bool> > visited = makeTestGrid(4, 4, false);
  std::list<Point_t> goals = map_2_goals(grid, true);

  visited[0][0] = 1;
  visited[0][1] = 1;
  visited[0][2] = 1;
  visited[0][3] = 1;
  visited[1][3] = 1;
  visited[2][3] = 1;
  visited[2][2] = 1;
  visited[2][1] = 1;

  gridNode_t start;
  start.pos = {0, 0};  // NOLINT
  start.cost = 1;
  start.he = 0;

  std::list<gridNode_t> pathNodes;

  bool resign = a_star_to_open_space(grid,  // map to traverse, all empty
                                     start,  // Start
                                     1,  // Cost of traversing a node
                                     visited,  // Visited nodes, of which there are none yet
                                     goals,
                                     pathNodes);
  /*
   * [p p p p]
   * [1 1 1 p]
   * [0 v v p] // These 2 visited nodes but unconnected nodes cannot happen in reality but fine for this test
   * [1 1 1 p]
   */
  ASSERT_EQ(false, resign);
  ASSERT_EQ(7, pathNodes.size());
}

/*
 * When we start in the corner and have obstacles around us, check that we can indeed not find a path
 */
TEST(TestAStarToOpenSpace, testBlockedMap)
{
  /*
   * [s 1 0 0]
   * [1 1 0 0]
   * [0 0 0 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  std::vector<std::vector<bool> > visited = makeTestGrid(4, 4, false);

  visited[0][0] = true;
  grid[1][0] = true;
  grid[0][1] = true;
  grid[1][1] = true;
  std::list<Point_t> goals = map_2_goals(grid, true);

  gridNode_t start;
  start.pos = {0, 0};  // NOLINT
  start.cost = 1;
  start.he = 0;

  std::list<gridNode_t> pathNodes;

  bool resign = a_star_to_open_space(grid,  // map to traverse, all empty
                                     start,  // Start
                                     1,  // Cost of traversing a node
                                     visited,  // Visited nodes, of which there are none yet
                                     goals,
                                     pathNodes);
  // No path possible so we should resign
  ASSERT_EQ(true, resign);
  ASSERT_EQ(1, pathNodes.size());  // Only the cell we start at:
  ASSERT_EQ(start.pos, pathNodes.front().pos);
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
