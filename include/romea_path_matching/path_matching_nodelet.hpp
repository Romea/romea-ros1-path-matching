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

#ifndef ROMEA_PATH_MATCHING__PATH_MATCHING_NODELET_HPP_
#define ROMEA_PATH_MATCHING__PATH_MATCHING_NODELET_HPP_

// ros
#include <nodelet/nodelet.h>

// romea
#include <romea_path_matching/path_matching.hpp>

namespace romea
{
namespace ros1
{

class PathMatchingNodelet : public nodelet::Nodelet
{
public:
  void onInit() override;

private:
  std::unique_ptr<PathMatching> path_matching_;
};

}  // namespace ros1
}  // namespace romea

#endif
