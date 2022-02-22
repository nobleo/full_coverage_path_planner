//
// Copyright [2020] Nobleo Technology [legal/copyright]
//
#include "full_coverage_path_planner/curve_generator.hpp"

namespace curve_generator
{

// Generate Cubic BezierCurve using the logic in these sources
// Link 1:
// https://en.wikipedia.org/wiki/B%C3%A9zier_curve#/media/File:B%C3%A9zier_3_big.svg
// Link 2:
// https://en.wikipedia.org/wiki/B%C3%A9zier_curve#
// Link 3:
// https://stackoverflow.com/questions/785097/how-do-i-implement-a-b%C3%A9zier-curve-in-c
// Link 4:
// https://stackoverflow.com/questions/37642168/how-to-convert-quadratic-bezier-curve-code-into-cubic-bezier-curve/37642695#37642695
// We add the calculation of the angle given by the blue lines in the gifs.

void CubicBezier::generateCubicBezierCurve(
  const tf2::Vector3 p0,
  const tf2::Vector3 p1,
  const tf2::Vector3 p2,
  const tf2::Vector3 p3,
  const double max_path_resolution,
  nav_msgs::msg::Path & path)
{
  // Approximate normalized step by computing a bound on length by summing the
  // control segments lengths
  double normalized_step =
    max_path_resolution /
    ((p1 - p0).length() + (p2 - p1).length() + (p3 - p2).length());
  for (double scale = 0.0; scale < 1.0; scale += normalized_step) {
    // The Green Lines in Link 1
    auto q0 = tf2::lerp(p0, p1, scale);
    auto q1 = tf2::lerp(p1, p2, scale);
    auto q2 = tf2::lerp(p2, p3, scale);

    // The Blue Line in Link 1
    auto r0 = tf2::lerp(q0, q1, scale);
    auto r1 = tf2::lerp(q1, q2, scale);

    // The Black Dot in Link 1
    auto bezier_pose = tf2::lerp(r0, r1, scale);

    // Get pose
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = bezier_pose.x();
    pose.pose.position.y = bezier_pose.y();
    pose.pose.position.z = bezier_pose.z();
    // Vector representing the tangent of the blue line
    auto axis_vector = r1 - r0;
    // For now only valid for the xy plane (z=0)
    // Search a way to do it for 3D. Tries this but did noy work
    // https://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/
    double rz = atan2(axis_vector.y(), axis_vector.x());
    auto bezier_quaternion = tf2::Quaternion();
    bezier_quaternion.setRPY(0.0, 0.0, rz);
    pose.pose.orientation.x = bezier_quaternion.x();
    pose.pose.orientation.y = bezier_quaternion.y();
    pose.pose.orientation.z = bezier_quaternion.z();
    pose.pose.orientation.w = bezier_quaternion.w();

    path.poses.push_back(pose);
  }
}

}  // namespace curve_generator
