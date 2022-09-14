//  Copyright 2022 Sakai Hibiki
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "prox2f_contact_analysis/robotiq_2f_85_fingertip.hpp"

#include <vector>

namespace prox
{
namespace contact
{
Robotiq2F85Fingertip::Robotiq2F85Fingertip() {}

std::vector<Kernel::Ray_3> Robotiq2F85Fingertip::get_rays(uint16_t dpi) const
{
  const float dpm = dpi / .0254;
  const uint16_t rows = height * dpm;
  const uint16_t cols = width * dpm;
  const float pitch = 1 / dpm;

  const Kernel::Point_3 top_left(-width / 2, height / 2, 0);
  const Kernel::Vector_3 direction(0, 0, 1);

  std::vector<Kernel::Ray_3> rays;
  rays.reserve(cols * rows);

  for (uint16_t i = 0; i < rows; ++i) {
    for (uint16_t j = 0; j < cols; ++j) {
      const auto point = top_left + Kernel::Vector_3(pitch * j, -pitch * i, 0);
      rays.emplace_back(point, direction);
    }
  }

  return rays;
}

}  // namespace contact
}  // namespace prox
