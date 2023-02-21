//  Copyright 2023 Sakai Hibiki
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

#ifndef PROX_MESH__RAYCASTER_HPP_
#define PROX_MESH__RAYCASTER_HPP_

#include <optional>
#include <vector>

#include "shape_msgs/msg/mesh.hpp"

namespace prox::mesh
{

using Vertex = std::array<float, 3>;
using Polygon = std::array<uint32_t, 3>;
struct Ray
{
  Vertex origin, direction;
};
using Hit = std::optional<Vertex>;

class Raycaster
{
public:
  Raycaster() = default;
  virtual ~Raycaster() = default;
  virtual void load_mesh(const shape_msgs::msg::Mesh & mesh_msg) = 0;
  virtual std::vector<Hit> raycast(const std::vector<Ray> & rays) = 0;
};

}  // namespace prox::mesh

#endif  // PROX_MESH__RAYCASTER_HPP_
