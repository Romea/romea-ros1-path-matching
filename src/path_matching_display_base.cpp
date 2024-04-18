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

// std
#include <utility>

// local
#include "romea_path_matching/path_matching_display_base.hpp"

namespace romea
{
namespace ros1
{

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::init(ros::NodeHandle & nh, const std::string & path_frame_id)
{
  initMarkers(path_frame_id);
  marker_pub_ = nh.advertise<MarkerArray>("markers", 1);
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::initMarkers(const std::string & path_frame_id)
{
  path_marker_.header.frame_id = path_frame_id;
  path_marker_.ns = "path";
  path_marker_.id = 1;
  path_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  path_marker_.action = visualization_msgs::Marker::ADD;
  path_marker_.pose.orientation.w = 1;
  path_marker_.color.r = .5;
  path_marker_.color.g = .5;
  path_marker_.color.b = .5;
  path_marker_.color.a = 1.;

  path_marker_.scale.x = .1;
  path_marker_.scale.y = .1;
  path_marker_.scale.z = .1;

  curve_marker_.header.frame_id = path_frame_id;
  curve_marker_.ns = "curve";
  curve_marker_.id = 2;
  curve_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  curve_marker_.action = visualization_msgs::Marker::ADD;
  curve_marker_.pose.orientation.w = 1;
  curve_marker_.color.r = 1.;
  curve_marker_.color.g = .1;
  curve_marker_.color.b = .05;
  curve_marker_.color.a = 1.;

  curve_marker_.scale.x = .12;
  curve_marker_.scale.y = .12;
  curve_marker_.scale.z = .12;

  clear_marker_.header.frame_id = path_frame_id;
  clear_marker_.action = visualization_msgs::Marker::DELETEALL;
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::clear()
{
  path_marker_.points.clear();
  curve_marker_.points.clear();
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::load_curve(const core::PathCurve2D & path_curve)
{
  double ss = path_curve.getCurvilinearAbscissaInterval().lower();
  double ds = (path_curve.getCurvilinearAbscissaInterval().upper() - ss) / 30.;

  curve_marker_.points.clear();

  for (size_t n = 0; n < 30; n++) {
    double s = ss + n * ds;
    auto & pt = curve_marker_.points.emplace_back();
    pt.x = path_curve.computeX(s);
    pt.y = path_curve.computeY(s);
    pt.z = 0.1;
  }
}

//-----------------------------------------------------------------------------
void PathMatchingDisplayBase::publish()
{
  visualization_msgs::MarkerArray markers;
  markers.markers.push_back(clear_marker_);
  markers.markers.push_back(path_marker_);
  markers.markers.push_back(curve_marker_);
  marker_pub_.publish(std::move(markers));
}

}  // namespace ros1
}  // namespace romea
