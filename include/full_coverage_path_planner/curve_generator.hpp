//
// Copyright [2020] Nobleo Technology [legal/copyright]
//
#ifndef FULL_COVERAGE_PATH_PLANNER__CURVE_GENERATOR_HPP_
#define FULL_COVERAGE_PATH_PLANNER__CURVE_GENERATOR_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <vector>

namespace curve_generator
{

class CubicBezier
{
public:
  CubicBezier() = default;

  ~CubicBezier() = default;

  /**
   * Generate cubic Bezier curve from four control points
   * @param p0top4 control points
   * @param normalized_step scale value to do interpolation
   * @param path output path
   */
  void generateCubicBezierCurve(
    const tf2::Vector3 p0, const tf2::Vector3 p1,
    const tf2::Vector3 p2, const tf2::Vector3 p3,
    const double max_path_resolution,
    nav_msgs::msg::Path & path);
};
}  // namespace curve_generator

#endif  // FULL_COVERAGE_PATH_PLANNER__CURVE_GENERATOR_HPP_
