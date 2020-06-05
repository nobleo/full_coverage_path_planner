//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
// Created by nobleo on 27-9-18.
//

/*
 * Full coverage simply requires all reachable nodes to be visited.
 * We check this by counting the number of unique elements in the path.
 * By putting the path nodes in a set, we are left with only the unique elements
 *  and then we can count how big that set is (i.e. the cardinality of the set of path nodes)
 */
#include <list>
#include <set>
#include <vector>

#include <time.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <full_coverage_path_planner/common.h>
#include <full_coverage_path_planner/spiral_stc.h>
#include <full_coverage_path_planner/util.h>

cv::Mat drawMap(std::vector<std::vector<bool> > const& grid);

cv::Mat drawPath(const cv::Mat &mapImg,
                 const cv::Mat &pathImg,
                 const Point_t &start,
                 std::list<Point_t> &path);

/*
 * On a map with nothing on it, spiral_stc should cover all the nodes of the map
 */
TEST(TestSpiralStc, testFillEmptyMap)
{
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);

  Point_t start = {0, 0};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  ASSERT_EQ(4 * 4, path.size());  // All nodes of the 4x4 map are covered
}

/*
 * On a map with a single obstacle, all the other nodes should still be visited.
 */
TEST(TestSpiralStc, testFillMapWithOneObstacle)
{
  /*
   * [s 0 0 0]
   * [0 0 0 0]
   * [0 0 1 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[2][2] = 1;

  Point_t start = {0, 0};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 but with 1 obstacle, coverage all reachable nodes should over 4*4 - 1 = 15 nodes
  ASSERT_EQ((4 * 4) - 1, pathSet.size());
}

/*
 * In a map with 2 obstacles, still the complete map should be covered except for those 2 obstacles
 */
TEST(TestSpiralStc, testFillMapWith2Obstacles)
{
  /*
   * [s 0 0 0]
   * [0 0 0 0]
   * [0 0 1 0]
   * [0 0 0 1]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[2][2] = 1;
  grid[3][3] = 1;

  Point_t start = {0, 0};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 but with 1 obstacle, coverage all reachable nodes should over 4*4 - 2 = 14 nodes
  ASSERT_EQ((4 * 4) - 2, pathSet.size());
}

/*
 * On a 4x4 map where the opposite right half of the map is blocked, we can cover only the 4x2 reachable nodes
 */
TEST(TestSpiralStc, testFillMapWithHalfBlocked)
{
  /*
   * [s 0 1 0]
   * [0 0 1 0]
   * [0 0 1 0]
   * [0 0 1 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[0][2] = 1;
  grid[1][2] = 1;
  grid[2][2] = 1;
  grid[3][2] = 1;

  Point_t start = {0, 0};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 but can only visit half the map, half the 4x4 area should be covered
  ASSERT_EQ((4 * 4) / 2, pathSet.size());
}

/*
 * On a map with a wall almost blocking off a half of the map, but leaving a gap to the other side,
 *  spiral_stc should still cover all reachable nodes
 */
TEST(TestSpiralStc, testFillMapWithWall)
{
  /*
   * [s 0 1 0]
   * [0 0 1 0]
   * [0 0 1 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[0][2] = 1;
  grid[1][2] = 1;
  grid[2][2] = 1;

  Point_t start = {0, 0};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 there is a 3-length wall
  ASSERT_EQ((4 * 4) - 3, pathSet.size());
}

/*
 * This test case features a very short dead-end
 */
TEST(TestSpiralStc, testDeadEnd1)
{
  /*
   * [0 0 1 0]
   * [0 1 0 0]
   * [0 s 0 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[0][2] = 1;
  grid[1][1] = 1;

  cv::Mat mapImg = drawMap(grid);

  Point_t start = {1, 2};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  cv::Mat pathImg = mapImg.clone();
  cv::Mat pathViz = drawPath(mapImg, pathImg, start, path);
  cv::imwrite("/tmp/testDeadEnd1.png", pathViz);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 and there are2 obstacle cells
  ASSERT_EQ((4 * 4) - 2, pathSet.size());
}

/*
 * This test case is an extension of testDeadEnd1, where the top row is also covered as an obstacle.
 * The top row is covered but the obstacle from testDeadEnd1 is shifted downwards
 *  in an attempt to see if SpiralSTC also fails when a dead-end is not on the edge of the map
 *  (but below a row of obstacles)
 */
TEST(TestSpiralStc, testDeadEnd1WithTopRow)
{
  /*
   * [1 1 1 1]
   * [0 0 1 0]
   * [0 1 0 0]
   * [0 s 0 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 5, false);
  grid[0][0] = 1;
  grid[0][1] = 1;
  grid[0][2] = 1;
  grid[0][3] = 1;

  grid[1][2] = 1;
  grid[2][1] = 1;

  cv::Mat mapImg = drawMap(grid);

  Point_t start = {1, 3};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  cv::Mat pathImg = mapImg.clone();
  cv::Mat pathViz = drawPath(mapImg, pathImg, start, path);
  cv::imwrite("/tmp/testDeadEnd1WithTopRow.png", pathViz);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*5 and there are 6 cells blocked in total
  ASSERT_EQ((4 * 5) - 6, pathSet.size());
}

/*
 * This test case features a very short dead-end
 */
TEST(TestSpiralStc, testDeadEnd2)
{
  /*
   * [1 0 0 0]
   * [0 1 0 0]
   * [0 s 0 0]
   * [0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(4, 4, false);
  grid[0][0] = 1;
  grid[1][1] = 1;

  cv::Mat mapImg = drawMap(grid);

  Point_t start = {3, 0};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  cv::Mat pathImg = mapImg.clone();
  cv::Mat pathViz = drawPath(mapImg, pathImg, start, path);
  cv::imwrite("/tmp/testDeadEnd2.png", pathViz);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 and there are2 obstacle cells
  ASSERT_EQ((4 * 4) - 2, pathSet.size());
}

/*
 * This test case features a very short dead-end
 */
TEST(TestSpiralStc, testDeadEnd3)
{
  /*
   * [0 0 0 0 0 0 1 0 0]
   * [1 0 1 0 1 1 0 0 0]
   * [0 0 0 0 1 2 0 0 0]
   * [0 0 0 0 0 0 0 0 0]
   * [0 0 0 0 0 0 0 0 0]
   * [0 0 0 0 0 0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(9, 6, false);
  grid[0][6] = 1;
  grid[1][0] = 1;
  grid[1][2] = 1;
  grid[1][4] = 1;
  grid[1][5] = 1;
  grid[2][4] = 1;

  cv::Mat mapImg = drawMap(grid);

  Point_t start = {5, 2};
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  cv::Mat pathImg = mapImg.clone();
  cv::Mat pathViz = drawPath(mapImg, pathImg, start, path);
  cv::imwrite("/tmp/testDeadEnd3.png", pathViz);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 and there are2 obstacle cells
  ASSERT_EQ((6 * 9) - 6, pathSet.size());
}

/*
 * This test case features a very short dead-end
 */
TEST(TestSpiralStc, testDeadEnd3WithTopRow)
{
  /*
   * [1 1 1 1 1 1 1 1 1]
   * [0 0 0 0 0 0 1 0 0]
   * [1 0 1 0 1 1 0 0 0]
   * [0 0 0 0 1 2 0 0 0]
   * [0 0 0 0 0 0 0 0 0]
   * [0 0 0 0 0 0 0 0 0]
   * [0 0 0 0 0 0 0 0 0]
   */
  std::vector<std::vector<bool> > grid = makeTestGrid(9, 7, false);
  grid[0][0] = 1;
  grid[0][1] = 1;
  grid[0][2] = 1;
  grid[0][3] = 1;
  grid[0][4] = 1;
  grid[0][5] = 1;
  grid[0][6] = 1;

  grid[1][6] = 1;
  grid[2][0] = 1;
  grid[2][2] = 1;
  grid[2][4] = 1;
  grid[2][5] = 1;
  grid[3][4] = 1;

  cv::Mat mapImg = drawMap(grid);

  Point_t start = {5, 3};  // NOLINT
  int multiple_pass_counter, visited_counter;
  std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                              start,
                                                                              multiple_pass_counter,
                                                                              visited_counter);

  cv::Mat pathImg = mapImg.clone();
  cv::Mat pathViz = drawPath(mapImg, pathImg, start, path);
  cv::imwrite("/tmp/testDeadEnd3WithTopRow.png", pathViz);

  // By Adding the nodes of the path to the set, we only retain the unique elements
  std::set<Point_t, CompareByPosition> pathSet(path.begin(), path.end());

  // Because the area is 4*4 and there are2 obstacle cells
  ASSERT_EQ((7 * 9) - 6 - 7, pathSet.size());
}

/**
 * Draw a nested vector of bools into an openCV image
 * @param grid
 * @return 2D 8-bit single-channel image
 */
cv::Mat drawMap(std::vector<std::vector<bool> > const& grid)
{
  int y_size = static_cast<int>(grid.size());
  int x_size = static_cast<int>(grid[0].size());

  cv::Mat mapImg = cv::Mat::zeros(y_size, x_size, CV_8U);  // CV_8U 8bit unsigned int 1 channel
  for (int k = 0; k < y_size; k++)
  {
    for (int l = 0; l < x_size; l++)
    {
      if (grid[k][l])
      {
        cv::rectangle(mapImg, {l, k}, {l, k}, 255);  // NOLINT
      }
    }
  }
  return mapImg;
}

/**
 * Draw path on a copy of the map
 * This is done twice: one to serve as input for calcDifference and another is returned for visualisation purposes
 * @param mapImg original map with just obstacles
 * @param pathImg Image that will feed into calcDifference
 * @param start Where does the path start?
 * @param path the actual path to be drawn
 * @return 2D RGB image for visualisation purposes
 */
cv::Mat drawPath(const cv::Mat &mapImg,
                 const cv::Mat &pathImg,
                 const Point_t &start,
                 std::list<Point_t> &path)
{
  cv::Mat pathViz = cv::Mat::zeros(mapImg.cols, mapImg.rows, CV_8UC3);
  std::vector<cv::Mat> channels;
  channels.push_back(mapImg.clone());
  channels.push_back(mapImg.clone());
  channels.push_back(mapImg.clone());
  cv::merge(channels, pathViz);

  int step = 0;
  for (std::list<Point_t>::iterator it = path.begin(); it != path.end(); ++it)
  {
//      std::cout << "Path at (" << it->x << ", " << it->y << ")" << std::endl;
    cv::rectangle(pathImg, {it->x, it->y}, {it->x, it->y}, 255);  // NOLINT

    // Color the path in lighter and lighter color towards the end
    step++;
    int value = ((step * 200) / static_cast<int>(path.size())) + 50;
    cv::Scalar color(value, 128, 128);
    cv::rectangle(pathViz, {it->x, it->y}, {it->x, it->y}, color);  // NOLINT
  }

  // Draw the start and end in green and red, resp.
  cv::Scalar green(0, 255, 0);
  cv::Scalar red(0, 0, 255);
  cv::rectangle(pathViz,
  {start.x, start.y},
  {start.x, start.y},
  green);
  cv::rectangle(pathViz,
  {path.back().x, path.back().y},
  {path.back().x, path.back().y},
  red);
  return pathViz;
}

/**
 * Determine whether the drawn path covers all that can be covered
 * @param mapImg original map with obstacles
 * @param pathImg map with the path drawn into it, from drawPath
 * @param start where does the path start?
 * @return
 */
int calcDifference(const cv::Mat &mapImg, const cv::Mat &pathImg, const Point_t& start)
{
  cv::Mat floodfilledImg = mapImg.clone();
  cv::floodFill(floodfilledImg, {start.x, start.y}, 255);  // NOLINT
  cv::Mat difference;
  cv::subtract(floodfilledImg, pathImg, difference);

  return cv::countNonZero(difference);
}

/**
 * Find a proper starting point in the map, i.e. any place that is not an obstacle
 * @param grid
 * @return
 */
Point_t findStart(std::vector<std::vector<bool> > const& grid)
{
  unsigned int seed = time(NULL);
  int y_size = grid.size();
  int x_size = grid[0].size();

  Point_t start = {rand_r(&seed) % x_size, rand_r(&seed) % y_size};  // Start in some random place
  while (grid[start.y][start.x])
  {
    // Try to find a better starting point that is not inside an obstacle
    start = {rand_r(&seed) % x_size, rand_r(&seed) % y_size};  // Start in some random place
  }
  return start;
}

/*
 * Create a NxM map, spawn X random obstacles (so that some percentage of the map is covered by obstacles)
 Run coverage planning on that map and check that all reachable cells are covered (by using OpenCV Floodfill)

 1. Create map with obstacles
 2. Convert this to an image called mapImg
 3. Copy mapImg as floodfilledImg
 3. Copy mapImg as pathImg
 4. Perform floodfill from start position on floodfilledImg
 5. On each coordinate in path, fill pixel at that coordinate in pathImg
 6. pathImg and floodfilledImg should be identical
 */
TEST(TestSpiralStc, testRandomMap)
{
  // Seed pseudorandom sequence to create *reproducible test
  unsigned int seed = 12345;
  int failures = 0;
  int success = 0;
  int tests = 0;
  for (int i = 0; i < 5; ++i)
  // Or use https://github.com/google/googletest/blob/master/googletest/docs/advanced.md#repeating-the-tests
  {
    tests++;
    int x_size = rand_r(&seed) % 100 + 1;
    int y_size = rand_r(&seed) % 100 + 1;
    std::vector<std::vector<bool> > grid = makeTestGrid(x_size, y_size, false);
    randomFillTestGrid(grid, 20);  // ...% fill of obstacles

    cv::Mat mapImg = drawMap(grid);
    Point_t start = findStart(grid);
    int multiple_pass_counter, visited_counter;
    std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid,
                                                                                start,
                                                                                multiple_pass_counter,
                                                                                visited_counter);

    cv::Mat pathImg = mapImg.clone();
    cv::Mat pathViz = drawPath(mapImg, pathImg, start, path);
    int differentPixelCount = calcDifference(mapImg, pathImg, start);
    if (differentPixelCount)
    {
      cv::imwrite("/tmp/" + std::to_string(i) + "_path_viz.png", pathViz);
      failures++;
    }
    else
    {
      success++;
    }
    EXPECT_EQ(0, differentPixelCount);
  }

  ASSERT_EQ(0, failures);
  ASSERT_EQ(tests, success);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
