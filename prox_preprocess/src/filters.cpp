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

#include "prox_preprocess/filters.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <string>

namespace prox
{
namespace filters
{
pcl::PointCloud<pcl::PointXYZ> pass_through(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::string & field_name, double limit_min,
  double limit_max)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = cloud;
  pcl::PointCloud<pcl::PointXYZ> output_cloud;

  pcl::PassThrough<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud_ptr);
  filter.setFilterFieldName(field_name);
  filter.setFilterLimits(limit_min, limit_max);
  filter.filter(output_cloud);

  return output_cloud;
}

pcl::PointCloud<pcl::PointXYZ> remove_outliers(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, double radius, int min_neighbors)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = cloud;
  pcl::PointCloud<pcl::PointXYZ> output_cloud;

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
  filter.setInputCloud(cloud_ptr);
  filter.setRadiusSearch(radius);
  filter.setMinNeighborsInRadius(min_neighbors);
  filter.filter(output_cloud);

  return output_cloud;
}

}  // namespace filters
}  // namespace prox
