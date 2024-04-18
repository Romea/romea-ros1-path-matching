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

// local
#include "romea_path_matching/path_matching_base.hpp"


namespace
{
const double MAXIMAL_REASEARCH_RADIUS = 10;
const double INTERPOLATION_WINDOW_LENGTH = 3;
const double PREDICTION_TIME_HORIZON = 0.5;
}  // namespace

namespace romea
{
namespace ros1
{

//-----------------------------------------------------------------------------
PathMatchingBase::PathMatchingBase(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
  : nh_(nh), private_nh_(private_nh)
{
}


//-----------------------------------------------------------------------------
void PathMatchingBase::on_configure()
{
  private_nh_.param("maximal_researh_radius", maximal_research_radius_, MAXIMAL_REASEARCH_RADIUS);
  private_nh_.param("prediction_time_horizon", prediction_time_horizon_, PREDICTION_TIME_HORIZON);
  private_nh_.param(
    "interpolation_window_length", interpolation_window_length_, INTERPOLATION_WINDOW_LENGTH);

  odom_sub_ = nh_.subscribe("filtered_odom", 1, &PathMatchingBase::process_odom_, this);

  // diagnostics_pub_ = romea::ros2::make_diagnostic_publisher<romea::core::DiagnosticReport>(
  //   node_, std::string(node_->get_namespace()) + "/" + std::string(node_->get_name()), 1.0);

  reset_srv_ = private_nh_.advertiseService("reset", &PathMatchingBase::reset_srv_callback_, this);
}

//-----------------------------------------------------------------------------
void PathMatchingBase::on_activate()
{
  match_pub_ = private_nh_.advertise<PathMatchingInfo2D>("info", 1);
  // match_pub_->on_activate();
  // diagnostics_pub_->activate();
  is_active_ = true;
}

//-----------------------------------------------------------------------------
void PathMatchingBase::on_deactivate()
{
  match_pub_.shutdown();
  // diagnostics_pub_->deactivate();
  is_active_ = false;
}

//-----------------------------------------------------------------------------
bool PathMatchingBase::reset_srv_callback_(ResetSrv::Request &, ResetSrv::Response &)
{
  reset();
  return true;
}

}  // namespace ros1
}  // namespace romea
