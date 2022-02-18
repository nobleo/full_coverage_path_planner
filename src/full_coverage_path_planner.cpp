//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <list>
#include <vector>

#include "full_coverage_path_planner/full_coverage_path_planner.hpp"

/*  *** Note the coordinate system ***
 *  grid[][] is a 2D-vector:
 *  where ix is column-index and x-coordinate in map,
 *  iy is row-index and y-coordinate in map.
 *
 *            Cols  [ix]
 *        _______________________
 *       |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 * Rows  |__|__|__|__|__|__|__|__|
 * [iy]  |__|__|__|__|__|__|__|__|
 *       |__|__|__|__|__|__|__|__|
 *y-axis |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   ^   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *   |   |__|__|__|__|__|__|__|__|
 *
 *   O   --->> x-axis
 */

// Default Constructor
namespace full_coverage_path_planner
{

FullCoveragePathPlanner::FullCoveragePathPlanner()
: initialized_(false) {}

void FullCoveragePathPlanner::publishPlan(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan)
{
  if (!initialized_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("FullCoveragePathPlanner"),
      "This planner has not been initialized yet, but it is being "
      "used, please call initialize() before use");
    return;
  }

  //  Create a message for the plan
  nav_msgs::msg::Path gui_path;
  gui_path.header.frame_id = plan[0].header.frame_id;
  gui_path.header.stamp = plan[0].header.stamp;

  for (const auto pose : plan) {
    gui_path.poses.push_back(pose);
  }

  plan_pub_->publish(gui_path);
}

// void FullCoveragePathPlanner::publishPlan(const
// std::vector<nav_msgs::msg::Path> & path_vector)
// {
// }

void FullCoveragePathPlanner::convertPlanToPathVector(
  const std::vector<geometry_msgs::msg::PoseStamped> & plan,
  const bool enable_smoothing, const double max_path_resolution,
  const double grid_size, std::vector<nav_msgs::msg::Path> & path_vector)
{
  //  Create a message for the plan

  // If no smoothing is enabled, simply gather sections of two waypoints
  if (!enable_smoothing) {
    for (auto it = plan.begin(); it != plan.end(); it++) {
      nav_msgs::msg::Path path;
      path.header.frame_id = plan[0].header.frame_id;
      path.header.stamp = plan[0].header.stamp;
      auto it_next = it;
      it_next++;
      path.poses.push_back(*it);
      path.poses.push_back(*it_next);
      path_vector.push_back(path);
    }
  } else {
    auto bezier_generator = curve_generator::CubicBezier();
    for (auto it = plan.begin(); it != (plan.end() - 2); it++) {
      nav_msgs::msg::Path path;
      path.header.frame_id = plan[0].header.frame_id;
      path.header.stamp = plan[0].header.stamp;
      auto it_next = it;
      it_next++;
      auto it_second_next = it_next;
      it_second_next++;

      // For now we do not smooth section of the the first two poses to avoid an
      // overshooting path
      if (it == plan.begin()) {
        path.poses.push_back(*it);
        path.poses.push_back(*it_next);
        path_vector.push_back(path);
      } else {
        auto it_previous = it;
        it_previous--;
        // Check if there is a pure rotation
        tf2::Transform tf_current_pose;
        tf2::convert(*it, tf_current_pose);
        tf2::Transform tf_next_pose;
        tf2::convert(*it_next, tf_next_pose);
        double turning_angle = tf_next_pose.getRotation().angleShortestPath(
          tf_current_pose.getRotation());
        if (abs(turning_angle) > 0.1) {
          // Compute Bezier control points
          tf2::Transform tf_previous_pose;
          tf2::convert(*it_previous, tf_previous_pose);
          auto vector_pose_diff =
            tf_current_pose.getOrigin() - tf_previous_pose.getOrigin();
          auto length_current_straight_segment = vector_pose_diff.length();
          tf2::Transform tf_second_next_pose;
          tf2::convert(*it_second_next, tf_second_next_pose);
          vector_pose_diff =
            tf_second_next_pose.getOrigin() - tf_next_pose.getOrigin();
          auto length_next_straight_segment = vector_pose_diff.length();

          double clip_distance;
          if (length_current_straight_segment > grid_size &&
            length_next_straight_segment > grid_size)
          {
            // default clip_distance
            clip_distance = grid_size;
          } else {
            // short clip_distance for two consecutive turns
            clip_distance = grid_size / 2.0;
          }
          // p0 is located at a clip_distance inside the corner point along the
          // current straight path
          auto p0 = tf_previous_pose.getOrigin().lerp(
            tf_current_pose.getOrigin(),
            1 - clip_distance / length_current_straight_segment);
          // p1 is located at a clip_distance beyond the corner point along the
          // current straight path
          auto p1 = tf_previous_pose.getOrigin().lerp(
            tf_current_pose.getOrigin(),
            1 + clip_distance / length_current_straight_segment);
          // p2 is located at a clip_distance before the corner point along the
          // next straight path
          auto p2 = tf_next_pose.getOrigin().lerp(
            tf_second_next_pose.getOrigin(),
            -clip_distance / length_current_straight_segment);
          // p3 is located at a clip_distance inside the corner point along the
          // next straight path
          auto p3 = tf_next_pose.getOrigin().lerp(
            tf_second_next_pose.getOrigin(),
            clip_distance / length_current_straight_segment);
          bezier_generator.generateCubicBezierCurve(
            p0, p1, p2, p3,
            max_path_resolution, path);
          path_vector.push_back(path);
        }

        // If so, compute start, end and control points
        // For simplicity, both control points are located at the original
        // corner
      }
    }
  }
}

void FullCoveragePathPlanner::parsePointlist2Plan(
  const geometry_msgs::msg::PoseStamped & start,
  std::list<Point_t> const & goalpoints,
  std::vector<geometry_msgs::msg::PoseStamped> & plan)
{
  geometry_msgs::msg::PoseStamped new_goal;
  std::list<Point_t>::const_iterator it, it_next, it_prev;
  bool do_publish = false;
  double orientation = dir::none;
  RCLCPP_INFO(
    rclcpp::get_logger("FullCoveragePathPlanner"),
    "Received goalpoints with length: %lu", goalpoints.size());

  // TODO(AronTiemessen): Eventually remove this, printing goalpoint list
  tf2::Quaternion quat;
  quat.setW(start.pose.orientation.w);
  quat.setX(start.pose.orientation.x);
  quat.setY(start.pose.orientation.y);
  quat.setZ(start.pose.orientation.z);
  double current_angle = quat.getAngle();
  if (current_angle > M_PI) {
    current_angle -= 2 * M_PI;
  }
  int prevpoint_x = goalpoints.front().x, prevpoint_y = goalpoints.front().y;
  int goalpoint_counter = 1;
  for (const auto point : goalpoints) {
    if (goalpoint_counter > 1) {
      current_angle = std::atan2(point.y - prevpoint_y, point.x - prevpoint_x);
    }
    RCLCPP_INFO(
      rclcpp::get_logger("FullCoveragePathPlanner"),
      "Goalpoint %d: (x=%d, y=%d, o=%f)", goalpoint_counter, point.x,
      point.y, current_angle);
    prevpoint_x = point.x;
    prevpoint_y = point.y;
    goalpoint_counter++;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("FullCoveragePathPlanner"),
    "Full turn arounds in the goals: %lu",
    turn_around_directions_.size());

  if (goalpoints.size() < 1) {
    RCLCPP_WARN(
      rclcpp::get_logger("FullCoveragePathPlanner"),
      "Empty point list");
    return;
  } else if (goalpoints.size() == 1) {
    new_goal.header.frame_id = "map";
    new_goal.pose.position.x = (goalpoints.begin()->x) * tile_size_ +
      grid_origin_.x + tile_size_ * 0.5;
    new_goal.pose.position.y = (goalpoints.begin()->y) * tile_size_ +
      grid_origin_.y + tile_size_ * 0.5;
    new_goal.pose.orientation = createQuaternionMsgFromYaw(0);
    plan.push_back(new_goal);
  } else {
    for (it = goalpoints.begin(); it != goalpoints.end(); it++) {
      it_next = it;
      it_next++;
      it_prev = it;
      it_prev--;

      int dx_now;
      int dy_now;
      int dx_next;
      int dy_next;
      // Check for the direction of movement
      if (it == goalpoints.begin()) {
        dx_now = it_next->x - it->x;
        dy_now = it_next->y - it->y;
        dx_next = 0;
        dy_next = 0;
      } else {
        dx_now = it->x - it_prev->x;
        dy_now = it->y - it_prev->y;
        dx_next = it_next->x - it->x;
        dy_next = it_next->y - it->y;
      }

      // Calculate direction enum: dx + dy*2 will give a unique number for each
      // of the four possible directions because of their signs:
      //  1 +  0*2 =  1
      //  0 +  1*2 =  2
      // -1 +  0*2 = -1
      //  0 + -1*2 = -2
      int move_dir_now = dx_now + dy_now * 2;
      int move_dir_next = dx_next + dy_next * 2;

      // Check if this points needs to be published (i.e. a change of direction
      // or first or last point in list)
      do_publish = (move_dir_next != move_dir_now || it == goalpoints.begin() ||
        (it != goalpoints.end() && it == --goalpoints.end()));

      // Add to vector if required
      if (do_publish) {
        new_goal.header.frame_id = "map";
        new_goal.pose.position.x =
          (it->x) * tile_size_ + grid_origin_.x + tile_size_ * 0.5;
        new_goal.pose.position.y =
          (it->y) * tile_size_ + grid_origin_.y + tile_size_ * 0.5;
        // Calculate desired orientation to be in line with movement direction
        switch (move_dir_now) {
          case dir::none:
            // Keep orientation
            break;
          case dir::right:
            orientation = 0;
            break;
          case dir::up:
            orientation = M_PI / 2;
            break;
          case dir::left:
            orientation = M_PI;
            break;
          case dir::down:
            orientation = M_PI * 1.5;
            break;
        }
        new_goal.pose.orientation = createQuaternionMsgFromYaw(orientation);
        if (it != goalpoints.begin()) {
          // Republish previous goal but with new orientation to indicate change
          // of direction Useful when the plan is strictly followed with
          // base_link Also add an intermediate orientation to dictate the
          // rotation direction
          if (((cos(orientation) == -cos(previous_orientation_) &&
            cos(orientation) != 0) ||
            (sin(orientation) == -sin(previous_orientation_) &&
            sin(orientation) != 0)) &&
            turn_around_directions_.size() > 0)
          {
            RCLCPP_INFO(
              rclcpp::get_logger("FullCoveragePathPlanner"),
              "Adding intermediate orientation");
            if (turn_around_directions_.back() == eClockwise) {
              previous_goal_.pose.orientation =
                createQuaternionMsgFromYaw(orientation - M_PI / 2);
              plan.push_back(previous_goal_);
            } else {
              previous_goal_.pose.orientation =
                createQuaternionMsgFromYaw(orientation + M_PI / 2);
              plan.push_back(previous_goal_);
            }
            turn_around_directions_.pop_back();
          }
          previous_goal_.pose.orientation = new_goal.pose.orientation;
          plan.push_back(previous_goal_);
        }
        plan.push_back(new_goal);
        previous_goal_ = new_goal;
        previous_orientation_ = orientation;
      }
    }
  }

  // // Add poses from current position to start of plan
  // // Compute angle between current pose and first plan point
  // double dy = plan.begin()->pose.position.y - start.pose.position.y;
  // double dx = plan.begin()->pose.position.x - start.pose.position.x;
  // RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "From start (%f,
  // %f) to plan (%f, %f)", start.pose.position.x, start.pose.position.y,
  // plan.begin()->pose.position.x, plan.begin()->pose.position.y);
  // // Arbitrary choice of 100.0*FLT_EPSILON to determine minimum angle
  // precision of 1% if (!(fabs(dy) < 100.0 * FLT_EPSILON && fabs(dx) < 100.0 *
  // FLT_EPSILON)) {
  //   // Add extra translation waypoint
  //   double yaw = std::atan2(dy, dx);
  //   RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "Angle from
  //   starting section: %f", yaw); geometry_msgs::msg::Quaternion quat_temp =
  //   createQuaternionMsgFromYaw(yaw); geometry_msgs::msg::PoseStamped
  //   extra_pose; extra_pose = *plan.begin(); extra_pose.pose.orientation =
  //   quat_temp; plan.insert(plan.begin(), extra_pose); extra_pose = start;
  //   extra_pose.pose.orientation = quat_temp;
  //   plan.insert(plan.begin(), extra_pose);
  // }

  // // Insert current pose
  // plan.insert(plan.begin(), start);
  // // RCLCPP_INFO(rclcpp::get_logger("FullCoveragePathPlanner"), "REMOVING
  // FIRST WAYPOINT IN THE PARSED GOAL LIST, JUST TO TEST, REMOVE LATER")
  // // TODO(aron): remove the above

  RCLCPP_INFO(
    rclcpp::get_logger("FullCoveragePathPlanner"),
    "Plan ready containing %lu goals!", plan.size());
}

bool FullCoveragePathPlanner::parseGrid(
  nav2_costmap_2d::Costmap2D const * cpp_costmap,
  std::vector<std::vector<bool>> & grid, double grid_size,
  geometry_msgs::msg::PoseStamped const & real_start, Point_t & scaled_start,
  double & yaw_start)
{
  size_t node_row;
  size_t node_col;
  size_t node_size = dmax(
    ceil(grid_size / cpp_costmap->getResolution()),
    1);                        // Size of node in pixels/units
  size_t n_rows = cpp_costmap->getSizeInCellsY();
  size_t n_cols = cpp_costmap->getSizeInCellsX();
  unsigned char * cpp_costmap_data = cpp_costmap->getCharMap();
  RCLCPP_INFO(
    rclcpp::get_logger("FullCoveragePathPlanner"),
    "n_rows: %lu, n_cols: %lu, node_size: %lu", n_rows, n_cols,
    node_size);

  if (n_rows == 0 || n_cols == 0) {
    return false;
  }

  // Save map origin and scaling
  cpp_costmap->mapToWorld(0, 0, grid_origin_.x, grid_origin_.y);
  tile_size_ =
    node_size * cpp_costmap->getResolution();    // Size of a tile in meters

  // Scale starting point
  scaled_start.x = static_cast<unsigned int>(
    clamp(
      (real_start.pose.position.x - grid_origin_.x) / tile_size_, 0.0,
      floor(cpp_costmap->getSizeInCellsX() / tile_size_)));
  scaled_start.y = static_cast<unsigned int>(
    clamp(
      (real_start.pose.position.y - grid_origin_.y) / tile_size_, 0.0,
      floor(cpp_costmap->getSizeInCellsY() / tile_size_)));

  // Determine initial orientation
  tf2::Quaternion q;
  q.setW(real_start.pose.orientation.w);
  q.setX(real_start.pose.orientation.x);
  q.setY(real_start.pose.orientation.y);
  q.setZ(real_start.pose.orientation.z);
  yaw_start = q.getAngle();
  if (yaw_start > M_PI) { // q.getAngle() gives angle in [0, 2*PI], wrap back to
                          // [-PI, PI] if necessary
    yaw_start -= 2 * M_PI;
  }

  // Scale grid
  for (size_t iy = 0; iy < n_rows; iy = iy + node_size) {
    std::vector<bool> grid_row;
    for (size_t ix = 0; ix < n_cols; ix = ix + node_size) {
      bool node_occupied = false;
      for (node_row = 0; (node_row < node_size) && ((iy + node_row) < n_rows) &&
        (node_occupied == false);
        ++node_row)
      {
        for (node_col = 0; (node_col < node_size) && ((ix + node_col) < n_cols);
          ++node_col)
        {
          int index_grid = (iy + node_row) * n_cols + (ix + node_col);
          if (cpp_costmap_data[index_grid] > COVERAGE_COST) {
            node_occupied = true;
            break;
          }
        }
      }
      grid_row.push_back(node_occupied);
    }
    grid.push_back(grid_row);
  }
  return true;
}

}  // namespace full_coverage_path_planner
