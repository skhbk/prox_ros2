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

#ifndef PROX_MESH__RAYCASTER_EMBREE_HPP_
#define PROX_MESH__RAYCASTER_EMBREE_HPP_

#include <cmath>
#include <vector>

#include "prox_mesh/raycaster.hpp"

#include "Eigen/Core"
#include "embree3/rtcore.h"

namespace prox::mesh
{
class RaycasterEmbree : public Raycaster
{
  RTCDevice device_;
  RTCScene scene_;

public:
  RaycasterEmbree() : device_(rtcNewDevice(nullptr)), scene_(nullptr) {}

  ~RaycasterEmbree()
  {
    rtcReleaseScene(scene_);
    rtcReleaseDevice(device_);
  };

  void load_mesh(const shape_msgs::msg::Mesh & mesh_msg) override
  {
    rtcReleaseScene(scene_);
    scene_ = rtcNewScene(device_);

    std::vector<Vertex> vertices;
    std::vector<Polygon> polygons;
    vertices.reserve(mesh_msg.vertices.size());
    polygons.reserve(mesh_msg.triangles.size());
    for (const auto & e : mesh_msg.vertices) {
      vertices.push_back(
        {static_cast<float>(e.x), static_cast<float>(e.y), static_cast<float>(e.z)});
    }
    for (const auto & e : mesh_msg.triangles) {
      polygons.emplace_back(e.vertex_indices);
    }

    auto geometry = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

    rtcSetSharedGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertices.data(), 0, sizeof(Vertex),
      vertices.size());
    rtcSetSharedGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, polygons.data(), 0, sizeof(Polygon),
      polygons.size());

    rtcCommitGeometry(geometry);
    rtcAttachGeometry(scene_, geometry);
    rtcReleaseGeometry(geometry);
    rtcCommitScene(scene_);
  }

  std::vector<Hit> raycast(const std::vector<Ray> & rays) override
  {
    std::vector<Hit> hits;
    hits.reserve(rays.size());

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    RTCRayHit ray_hit;
    for (const auto & ray : rays) {
      ray_hit.ray.org_x = ray.origin[0];
      ray_hit.ray.org_y = ray.origin[1];
      ray_hit.ray.org_z = ray.origin[2];
      ray_hit.ray.dir_x = ray.direction[0];
      ray_hit.ray.dir_y = ray.direction[1];
      ray_hit.ray.dir_z = ray.direction[2];
      ray_hit.ray.tfar = INFINITY;

      rtcIntersect1(scene_, &context, &ray_hit);
      if (ray_hit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        const auto & e = ray_hit.ray;
        const Eigen::Vector3f o{e.org_x, e.org_y, e.org_z};
        const Eigen::Vector3f d{e.dir_x, e.dir_y, e.dir_z};
        const Eigen::Vector3f p = o + ray_hit.ray.tfar * d;
        hits.emplace_back(Vertex{p.x(), p.y(), p.z()});
      } else {
        hits.emplace_back(std::nullopt);
      }
    }

    assert(hits.size() == rays.size());

    return hits;
  }
};

}  // namespace prox::mesh

#endif  // PROX_MESH__RAYCASTER_EMBREE_HPP_
