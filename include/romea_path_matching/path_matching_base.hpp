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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_BASE_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_BASE_HPP_

// std
#include <memory>

// ros
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

// romea
#include <romea_common_utils/publishers/diagnostic_publisher.hpp>
#include <romea_path_msgs/PathMatchingInfo2D.h>

namespace romea
{
namespace ros1
{

class PathMatchingBase
{
public:
  using Odometry = nav_msgs::Odometry;
  using PathMatchingInfo2D = romea_path_msgs::PathMatchingInfo2D;
  using ResetSrv = std_srvs::Empty;
  using ReportPublisher = DiagnosticPublisher<core::DiagnosticReport>;

public:
  PathMatchingBase(ros::NodeHandle & nh, ros::NodeHandle & private_nh);
  virtual ~PathMatchingBase() = default;

  void on_configure();
  void on_activate();
  void on_deactivate();

  virtual void reset() = 0;

protected:
  virtual void timer_callback_(const ros::TimerEvent & event) = 0;

  virtual void process_odom_(const Odometry & msg) = 0;

  bool reset_srv_callback_(ResetSrv::Request &, ResetSrv::Response &);

protected:
  ros::NodeHandle & nh_;
  ros::NodeHandle & private_nh_;

  double prediction_time_horizon_ = 0;
  double maximal_research_radius_ = 0;
  double interpolation_window_length_ = 0;

  bool is_active_ = false;

  ros::Subscriber odom_sub_;
  ros::Publisher match_pub_;
  std::shared_ptr<ReportPublisher> diagnostics_pub_;
  ros::Timer timer_;
  ros::ServiceServer reset_srv_;
};

}  // namespace ros2
}  // namespace romea

#endif  // ROMEA_PATH_MATCHING__PATH_MATCHING_BASE_HPP_
