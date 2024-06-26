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

// ros
#include <pluginlib/class_list_macros.h>

// local
#include "romea_path_matching/path_matching_nodelet.hpp"

namespace romea
{
namespace ros1
{

PathMatchingNodelet::PathMatchingNodelet()
{
}

void PathMatchingNodelet::onInit()
{
  path_matching_ = std::make_unique<PathMatching>(getNodeHandle(), getPrivateNodeHandle());
  if (path_matching_->on_configure()) {
    path_matching_->on_activate();
  }
}

}  // namespace ros1
}  // namespace romea

PLUGINLIB_EXPORT_CLASS(romea::ros1::PathMatchingNodelet, nodelet::Nodelet)
