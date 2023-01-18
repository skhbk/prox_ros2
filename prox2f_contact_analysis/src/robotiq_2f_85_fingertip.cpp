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
  auto organized_rays = this->get_organized_rays(dpi);
  const std::size_t size = organized_rays.size() * organized_rays.front().size();

  std::vector<Kernel::Ray_3> rays;
  rays.reserve(size);

  for (auto & row : organized_rays) {
    rays.insert(rays.end(), row.begin(), row.end());
  }
  assert(rays.size() == size);

  return rays;
}

std::vector<std::vector<Kernel::Ray_3>> Robotiq2F85Fingertip::get_organized_rays(uint16_t dpi) const
{
  const float dpm = dpi / .0254;
  const uint16_t rows = width * dpm;
  const uint16_t cols = height * dpm;
  const float pitch = 1 / dpm;

  const Kernel::Point_3 origin(-height / 2, -width / 2, 0);
  const Kernel::Vector_3 direction(0, 0, 1);

  std::vector<std::vector<Kernel::Ray_3>> rays;
  rays.resize(rows);

  for (uint16_t y = 0; y < rows; ++y) {
    rays.at(y).reserve(cols);
    for (uint16_t x = 0; x < cols; ++x) {
      const auto point = origin + Kernel::Vector_3(pitch * x, pitch * y, 0);
      rays.at(y).emplace_back(point, direction);
    }
  }

  return rays;
}

}  // namespace contact
}  // namespace prox
