// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// romea
#include "romea_path_matching/path_matching.hpp"

#include <romea_common_utils/conversions/pose_and_twist3d_conversions.hpp>
#include <romea_common_utils/conversions/twist2d_conversions.hpp>
#include <romea_common_utils/params/ros_param.hpp>
#include <romea_core_common/geometry/PoseAndTwist3D.hpp>
#include <romea_core_path/PathFile.hpp>
#include <romea_core_path/PathMatching2D.hpp>
#include <romea_path_utils/path_matching_info_conversions.hpp>

#include "romea_path_utils/path_builder.hpp"

// #include "uturn_generator.hpp"
// #include <romea_path_msgs/PathAnnotations.h>

namespace romea
{
namespace ros1
{

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
PathMatching::PathMatching(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
: PathMatchingBase(nh, private_nh)
{
}

//-----------------------------------------------------------------------------
bool PathMatching::on_configure()
try {
  PathMatchingBase::on_configure();

  auto path = load_param<std::string>(private_nh_, "path");
  std::string path_dir = private_nh_.param("path_dir", std::string{});
  path_frame_id_ = load_param<std::string>(private_nh_, "path_frame_id");
  // auto wgs84_anchor = load_geodetic_coordinates(private_nh_, "wgs84_anchor");
  display_activated_ = load_param<bool>(private_nh_, "display");

  if (!path.empty() && path[0] != '/') {
    if (!path_dir.empty()) {
      path = path_dir + '/' + path;
    } else {
      throw std::runtime_error(
        "The parameter 'path_dir' is required if the 'path' is not an absolute file path");
    }
  }

  // annotation_dist_max_ = get_parameter_or(node_, "annotation_dist_max", 5.);
  // annotation_dist_min_ = get_parameter_or(node_, "annotation_dist_min", -0.5);
  path_matching_ = std::make_unique<core::PathMatching>(
    path, maximal_research_radius_, interpolation_window_length_);

  display_.init(private_nh_, path_frame_id_);
  display_.load_path(path_matching_->getPath());

  // comparator_.init();
  // uturn_generator_.init();
  // reset_sub_ = private_nh.subscribe<std_msgs::Bool>(
  //    "reset", 1, &PathMatching::resetCallback, this);
  // annotations_pub_ = private_nh.advertise<romea_path_msgs::PathAnnotations>("annotations", 1);

  // loadPath(path);

  timer_ = private_nh_.createTimer(ros::Duration(0.1), &PathMatching::timer_callback_, this);

  path_sub_ = private_nh_.subscribe("path", 1, &PathMatching::process_path_, this);

  ROS_INFO("configured");
  return true;

} catch (const std::runtime_error & e) {
  ROS_ERROR_STREAM("configuration failed: " << e.what());
  return false;
}

//-----------------------------------------------------------------------------
bool PathMatching::on_activate()
{
  bool res = PathMatchingBase::on_activate();
  if (res) ROS_INFO("activated");
  return res;
}

//-----------------------------------------------------------------------------
bool PathMatching::on_deactivate()
{
  bool res = PathMatchingBase::on_deactivate();
  if (res) ROS_INFO("deactivated");
  return res;
}

//-----------------------------------------------------------------------------
void PathMatching::reset()
{
  display_.clear();
  if (path_matching_) {
    path_matching_->reset();
    display_.load_path(path_matching_->getPath());
  }
}

//-----------------------------------------------------------------------------
void PathMatching::process_odom_(const Odometry & msg)
{
  if (!is_active_) {
    return;
  }

  auto stamp = to_romea_duration(msg.header.stamp);
  core::PoseAndTwist3D enuPoseAndBodyTwist3D;
  to_romea(msg.pose, enuPoseAndBodyTwist3D.pose);
  to_romea(msg.twist, enuPoseAndBodyTwist3D.twist);

  const auto & path = path_matching_->getPath();
  auto vehicle_pose = core::toPose2D(enuPoseAndBodyTwist3D.pose);
  auto vehicle_twist = core::toTwist2D(enuPoseAndBodyTwist3D.twist);

  auto matched_points =
    path_matching_->match(stamp, vehicle_pose, vehicle_twist, prediction_time_horizon_);

  if (!matched_points.empty()) {
    match_pub_.publish(
      to_ros_msg(msg.header.stamp, matched_points, 0, path.getLength(), vehicle_twist));

    // publishNearAnnotations(matched_point, msg.header.stamp);

    if (display_activated_) {
      const auto & section = path.getSection(matched_points[0].sectionIndex);
      const auto & curve = section.getCurve(matched_points[0].curveIndex);
      display_.load_curve(curve);
    }
  }
  display_.publish();
}

//-----------------------------------------------------------------------------
void PathMatching::timer_callback_(const ros::TimerEvent &)
{
  auto stamp = ros::Time::now();
  auto report = path_matching_->getReport(to_romea_duration(stamp));
  diagnostics_pub_->publish(stamp, report);
}

void PathMatching::process_path_(const nav_msgs::Path & msg)
{
  path_matching_->setPath(create_path(msg, 1.0));
  reset();
}

}  // namespace ros1
}  // namespace romea
