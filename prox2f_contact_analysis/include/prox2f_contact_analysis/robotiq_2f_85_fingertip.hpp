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

#ifndef PROX2F_CONTACT_ANALYSIS__ROBOTIQ_2F_85_FINGERTIP_HPP_
#define PROX2F_CONTACT_ANALYSIS__ROBOTIQ_2F_85_FINGERTIP_HPP_

#include "prox2f_contact_analysis/contact_surface.hpp"

#include <vector>

namespace prox
{
namespace contact
{
class Robotiq2F85Fingertip : public ContactSurface
{
public:
  static constexpr float width = .022, height = .038;

public:
  Robotiq2F85Fingertip();
  std::vector<Kernel::Ray_3> get_rays(uint16_t dpi) const override;
};
}  // namespace contact
}  // namespace prox

#endif  // PROX2F_CONTACT_ANALYSIS__ROBOTIQ_2F_85_FINGERTIP_HPP_
