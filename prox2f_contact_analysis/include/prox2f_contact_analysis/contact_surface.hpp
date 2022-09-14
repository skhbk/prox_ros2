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

#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <vector>

namespace prox
{
namespace contact
{
using Kernel = CGAL::Epeck;

class ContactSurface
{
public:
  virtual std::vector<Kernel::Ray_3> get_rays(uint16_t dpi) const = 0;
};
}  // namespace contact
}  // namespace prox
