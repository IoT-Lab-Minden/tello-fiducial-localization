/* BSD 3-Clause License
* 
* Copyright (c) 2020, Multi-robot Systems (MRS) group at Czech Technical University in Prague
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* Source: https://github.com/tiiuae/navigation
*/

#pragma once

#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <queue>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <rclcpp/rclcpp.hpp>
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"

enum TreeValue
{
  OCCUPIED = 1,
  FREE     = -1
};

enum PlanningResult
{
  COMPLETE = 0,
  GOAL_REACHED,
  INCOMPLETE,
  GOAL_IN_OBSTACLE,
  FAILURE
};


struct Node
{
  octomap::OcTreeKey key;
  double             total_cost;
  double             cum_dist;
  double             goal_dist;

  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  bool operator<(const Node &other) const;
  bool operator<=(const Node &other) const;
};

struct CostComparator
{
  bool operator()(const Node &n1, const Node &n2) const;
};

struct LeafComparator
{
  bool operator()(const std::pair<octomap::ColorOcTree::iterator, double> &l1, const std::pair<octomap::ColorOcTree::iterator, double> &l2) const;
};

struct HashFunction
{
  bool operator()(const Node &n) const;
};

class AstarPlanner {

public:
  AstarPlanner(rclcpp::Node* node, double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty, double greedy_penalty,
               double min_altitude, double max_altitude, double timeout_threshold, double max_waypoint_distance, bool unknown_is_occupied);

private:
  rclcpp::Node* node_;

  double safe_obstacle_distance;
  double euclidean_distance_cutoff;
  double planning_tree_resolution;
  double distance_penalty;
  double greedy_penalty;
  double timeout_threshold;
  double max_waypoint_distance;
  double min_altitude;
  double max_altitude;
  bool   unknown_is_occupied;

public:
  std::pair<std::vector<octomap::point3d>, PlanningResult> findPath(
      const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::shared_ptr<octomap::ColorOcTree> mapping_tree, double timeout);

private:
  const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                              {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                              {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                              {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};
  double                              getNodeDepth(const octomap::OcTreeKey &key, octomap::ColorOcTree &tree);

  std::vector<octomap::OcTreeKey> getNeighborhood(const octomap::OcTreeKey &key, octomap::ColorOcTree &tree);

  octomap::OcTreeKey expand(const octomap::OcTreeKey &key, const std::vector<int> &direction);

  double distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2);

  double distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::ColorOcTree &tree);

  bool freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::ColorOcTree &tree);

  std::vector<octomap::OcTreeKey> backtrackPathKeys(const Node &start, const Node &end, std::unordered_map<Node, Node, HashFunction> &parent_map);

  std::vector<octomap::point3d> keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::ColorOcTree &tree);

  DynamicEDTOctomapBase<octomap::ColorOcTree> euclideanDistanceTransform(std::shared_ptr<octomap::ColorOcTree> tree);

  std::optional<std::pair<octomap::ColorOcTree, std::vector<octomap::point3d>>> createPlanningTunnel(std::shared_ptr<octomap::ColorOcTree> tree,
                                                                                              const octomap::point3d &start);

  std::pair<octomap::point3d, bool> generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal, octomap::ColorOcTree &tree);

  std::vector<octomap::point3d> filterPath(const std::vector<octomap::point3d> &waypoints, octomap::ColorOcTree &tree);

  std::vector<octomap::point3d> prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::ColorOcTree &tree);

  /* geometry_msgs::msg::Quaternion yawToQuaternionMsg(const double &yaw); */
};
