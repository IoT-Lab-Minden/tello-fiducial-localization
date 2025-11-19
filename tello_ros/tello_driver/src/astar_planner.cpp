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

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tello_driver/astar_planner.hpp>

bool Node::operator==(const Node &other) const {
  return key == other.key;
}

bool Node::operator!=(const Node &other) const {
  return key != other.key;
}

bool Node::operator<(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist < other.goal_dist;
  }

  return total_cost < other.total_cost;
}

bool Node::operator<=(const Node &other) const {

  if (total_cost == other.total_cost) {
    return goal_dist <= other.goal_dist;
  }

  return total_cost <= other.total_cost;
}

bool CostComparator::operator()(const Node &n1, const Node &n2) const {

  if (n1.total_cost == n2.total_cost) {
    return n1.goal_dist > n2.goal_dist;
  }

  return n1.total_cost > n2.total_cost;
}

bool HashFunction::operator()(const Node &n) const {
  using std::hash;
  return ((hash<int>()(n.key.k[0]) ^ (hash<int>()(n.key.k[1]) << 1)) >> 1) ^ (hash<int>()(n.key.k[2]) << 1);
}

bool LeafComparator::operator()(const std::pair<octomap::ColorOcTree::iterator, double> &l1, const std::pair<octomap::ColorOcTree::iterator, double> &l2) const {
  return l1.second < l2.second;
}

/* AstarPlanner constructor //{ */
AstarPlanner::AstarPlanner(rclcpp::Node* node, double safe_obstacle_distance, double euclidean_distance_cutoff, double planning_tree_resolution, double distance_penalty,
                           double greedy_penalty, double min_altitude, double max_altitude, double timeout_threshold, double max_waypoint_distance,
                           bool unknown_is_occupied) {

  this->node_ = node;
  this->safe_obstacle_distance    = safe_obstacle_distance;
  this->euclidean_distance_cutoff = euclidean_distance_cutoff;
  this->planning_tree_resolution  = planning_tree_resolution;
  this->distance_penalty          = distance_penalty;
  this->greedy_penalty            = greedy_penalty;
  this->min_altitude              = min_altitude;
  this->max_altitude              = max_altitude;
  this->timeout_threshold         = timeout_threshold;
  this->max_waypoint_distance     = max_waypoint_distance;
  this->unknown_is_occupied       = unknown_is_occupied;
}
//}

/* findPath //{ */

std::pair<std::vector<octomap::point3d>, PlanningResult> AstarPlanner::findPath(
    const octomap::point3d &start_coord, const octomap::point3d &goal_coord, std::shared_ptr<octomap::ColorOcTree> mapping_tree, double timeout) {

  RCLCPP_INFO(node_->get_logger(), "[Astar]: start [%.2f, %.2f, %.2f]", start_coord.x(), start_coord.y(), start_coord.z());
  RCLCPP_INFO(node_->get_logger(), "[Astar]: goal [%.2f, %.2f, %.2f]", goal_coord.x(), goal_coord.y(), goal_coord.z());

  auto time_start = std::chrono::high_resolution_clock::now();

  this->timeout_threshold = timeout;

  auto tree_with_tunnel         = createPlanningTunnel(mapping_tree, start_coord);

  if (!tree_with_tunnel) {
    RCLCPP_INFO(node_->get_logger(), "[Astar]: could not create a tunnel");
    return {std::vector<octomap::point3d>(), FAILURE};
  }

  auto tree   = tree_with_tunnel.value().first;
  auto tunnel = tree_with_tunnel.value().second;

  auto map_goal      = goal_coord;
  auto map_query     = tree_with_tunnel->first.search(goal_coord);
  bool original_goal = true;

  if (map_query == NULL) {
    RCLCPP_INFO(node_->get_logger(), "[Astar]: Goal is outside of map");
    /*auto temp_goal = generateTemporaryGoal(start_coord, goal_coord, tree);
    RCLCPP_INFO(node_->get_logger(), "[Astar]: Generated a temporary goal: [%.2f, %.2f, %.2f]", temp_goal.first.x(), temp_goal.first.y(), temp_goal.first.z());
    if (temp_goal.second) {
      std::vector<octomap::point3d> vertical_path;
      vertical_path.push_back(start_coord);
      vertical_path.push_back(temp_goal.first);
      return {vertical_path, INCOMPLETE};
    } else {
      map_goal      = temp_goal.first;
      original_goal = false;
    }*/
  } else if (map_query && map_query->getValue() == TreeValue::OCCUPIED) {
    RCLCPP_INFO(node_->get_logger(), "[Astar]: Goal is inside an inflated obstacle");
    if (distEuclidean(map_goal, start_coord) <= 1 * safe_obstacle_distance) {
      RCLCPP_INFO(node_->get_logger(), "[Astar]: Path special case, we cannot get closer");
      return {std::vector<octomap::point3d>(), GOAL_REACHED};
    }
  }

  std::priority_queue<Node, std::vector<Node>, CostComparator> open_heap;
  std::unordered_set<Node, HashFunction>                       open;
  std::unordered_set<Node, HashFunction>                       closed;
  std::unordered_map<Node, Node, HashFunction>                 parent_map;  // first = child, second = parent

  octomap::OcTreeKey start;
  if (tunnel.empty()) {
    start = tree.coordToKey(start_coord);
  } else {
    start = tree.coordToKey(tunnel.back());
  }

  auto planning_start = tree.keyToCoord(start);
  auto goal           = tree.coordToKey(map_goal);

  if (distEuclidean(planning_start, map_goal) <= 2 * planning_tree_resolution) {

    RCLCPP_INFO(node_->get_logger(), "[Astar]: Path special case, we are there");

    return {std::vector<octomap::point3d>(), GOAL_REACHED};
  }

  RCLCPP_INFO(node_->get_logger(), "[Astar]: Planning from: %.2f, %.2f, %.2f", planning_start.x(), planning_start.y(), planning_start.z());
  RCLCPP_INFO(node_->get_logger(), "[Astar]: Planning to: %.2f, %.2f, %.2f", map_goal.x(), map_goal.y(), map_goal.z());

  Node first;
  first.key        = start;
  first.cum_dist   = 0;
  first.goal_dist  = distEuclidean(start, goal, tree);
  first.total_cost = first.cum_dist + first.goal_dist;
  open_heap.push(first);
  open.insert(first);

  Node best_node        = first;
  Node best_node_greedy = first;

  int neighbor_count = 0;

  while (!open.empty() && rclcpp::ok()) {

    Node current = open_heap.top();
    open_heap.pop();
    open.erase(current);
    closed.insert(current);

    auto time_now = std::chrono::high_resolution_clock::now();

    if (std::chrono::duration<double>(time_now - time_start).count() > timeout_threshold) {

      RCLCPP_INFO(node_->get_logger(), "[Astar]: Planning timeout! Using current best node as goal.");
      auto path_keys = backtrackPathKeys(best_node == first ? best_node_greedy : best_node, first, parent_map);
      RCLCPP_INFO(node_->get_logger(), "[Astar]: Path found. Length: %ld", path_keys.size());

      return {prepareOutputPath(path_keys, tree), INCOMPLETE};
    }

    auto current_coord = tree.keyToCoord(current.key);

    if (distEuclidean(current_coord, map_goal) <= 2 * planning_tree_resolution) {

      auto path_keys = backtrackPathKeys(current, first, parent_map);
      path_keys.push_back(tree.coordToKey(map_goal));
      RCLCPP_INFO(node_->get_logger(), "[Astar]: Path found. Length: %ld", path_keys.size());

    RCLCPP_INFO(node_->get_logger(), "Searched %d neighbors", neighbor_count);
      if (original_goal) {
        return {prepareOutputPath(path_keys, tree), COMPLETE};
      }
      return {prepareOutputPath(path_keys, tree), INCOMPLETE};
    }

    // expand
    auto neighbors = getNeighborhood(current.key, tree);

    neighbor_count += neighbors.size();

    for (auto &nkey : neighbors) {

      Node n;
      n.key = nkey;

      auto closed_query = closed.find(n);
      auto open_query   = open.find(n);

      // in open map
      n.goal_dist  = distEuclidean(nkey, goal, tree);
      n.cum_dist   = current.cum_dist + distEuclidean(current.key, nkey, tree);
      n.total_cost = greedy_penalty * n.goal_dist + distance_penalty * n.cum_dist;

      if (closed_query == closed.end() && open_query == open.end()) {

        if (n <= best_node) {
          best_node = n;
        }

        if (n.goal_dist <= best_node_greedy.goal_dist) {
          best_node_greedy = n;
        }

        open_heap.push(n);
        open.insert(n);
        parent_map[n] = current;
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Searched %d neighbors", neighbor_count);

  if (best_node != first) {

    auto path_keys = backtrackPathKeys(best_node, first, parent_map);

    RCLCPP_INFO(node_->get_logger(), "[Astar]: direct path does not exist, going to the 'best_node'");

    return {prepareOutputPath(path_keys, tree), INCOMPLETE};
  }

  if (best_node_greedy != first) {

    auto path_keys = backtrackPathKeys(best_node_greedy, first, parent_map);

    RCLCPP_INFO(node_->get_logger(), "[Astar]: direct path does not exist, going to the best_node_greedy'");

    return {prepareOutputPath(path_keys, tree), INCOMPLETE};
  }

  if (!tunnel.empty()) {
    std::vector<octomap::point3d> path_to_safety;
    path_to_safety.push_back(start_coord);
    path_to_safety.push_back(tunnel.back());
    RCLCPP_INFO(node_->get_logger(), "[Astar]: path does not exist, escaping no-go zone'");
    return {path_to_safety, INCOMPLETE};
  }

  RCLCPP_INFO(node_->get_logger(), "[Astar]: PATH DOES NOT EXIST!");

  return {std::vector<octomap::point3d>(), FAILURE};
}
//}

/* getNeighborhood() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::getNeighborhood(const octomap::OcTreeKey &key, octomap::ColorOcTree &tree) {

  std::vector<octomap::OcTreeKey> neighbors;

  for (auto &d : EXPANSION_DIRECTIONS) {

    auto newkey    = expand(key, d);
    auto tree_node = tree.search(newkey);

    if (tree_node != NULL) {
      // free cell?
      if (tree_node->getValue() == TreeValue::FREE && tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude) {
        neighbors.push_back(newkey);
      }
    } else if (!unknown_is_occupied) {
      if (tree.keyToCoord(newkey).z() >= min_altitude && tree.keyToCoord(newkey).z() <= max_altitude) {
        neighbors.push_back(newkey);
      }
    }
  }

  return neighbors;
}

//}

/* expand() //{ */

octomap::OcTreeKey AstarPlanner::expand(const octomap::OcTreeKey &key, const std::vector<int> &direction) {

  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + direction[0];
  k.k[1] = key.k[1] + direction[1];
  k.k[2] = key.k[2] + direction[2];

  return k;
}

//}

/* distEuclidean() //{ */

double AstarPlanner::distEuclidean(const octomap::point3d &p1, const octomap::point3d &p2) {

  return (p1 - p2).norm();
}

double AstarPlanner::distEuclidean(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, octomap::ColorOcTree &tree) {

  double voxel_dist = sqrt(pow(k1.k[0] - k2.k[0], 2) + pow(k1.k[1] - k2.k[1], 2) + pow(k1.k[2] - k2.k[2], 2));

  return voxel_dist * tree.getResolution();
}

//}

/* freeStraightPath() //{ */

bool AstarPlanner::freeStraightPath(const octomap::point3d p1, const octomap::point3d p2, octomap::ColorOcTree &tree) {

  octomap::KeyRay ray;
  tree.computeRayKeys(p1, p2, ray);

  for (auto &k : ray) {

    auto tree_node = tree.search(k);

    if (tree_node == NULL) {
      // Path may exist, but goes through unknown cells
      return false;
    }

    if (tree_node->getValue() == TreeValue::OCCUPIED) {
      // Path goes through occupied cells
      return false;
    }
  }

  return true;
}

//}

/* backtrackPathKeys() //{ */

std::vector<octomap::OcTreeKey> AstarPlanner::backtrackPathKeys(const Node &from, const Node &to, std::unordered_map<Node, Node, HashFunction> &parent_map) {

  std::vector<octomap::OcTreeKey> keys;

  Node current = from;
  keys.push_back(current.key);

  while (current.key != to.key) {
    current = parent_map.find(current)->second;
    keys.push_back(current.key);
  };

  keys.push_back(to.key);

  // reverse order
  std::reverse(keys.begin(), keys.end());
  return keys;
}

//}

/* keysToCoords() //{ */

std::vector<octomap::point3d> AstarPlanner::keysToCoords(std::vector<octomap::OcTreeKey> keys, octomap::ColorOcTree &tree) {

  std::vector<octomap::point3d> coords;

  for (auto &k : keys) {
    coords.push_back(tree.keyToCoord(k));
  }

  return coords;
}

//}

/* euclideanDistanceTransform() //{ */

DynamicEDTOctomapBase<octomap::ColorOcTree> AstarPlanner::euclideanDistanceTransform(std::shared_ptr<octomap::ColorOcTree> tree) {

  double x, y, z;

  tree->getMetricMin(x, y, z);
  octomap::point3d metric_min(x, y, z);

  tree->getMetricMax(x, y, z);
  octomap::point3d metric_max(x, y, z);

  DynamicEDTOctomapBase<octomap::ColorOcTree> edf(euclidean_distance_cutoff, tree.get(), metric_min, metric_max, unknown_is_occupied);
  edf.update();

  return edf;
}
//}

/* createPlanningTree() //{ */

std::optional<std::pair<octomap::ColorOcTree, std::vector<octomap::point3d>>> AstarPlanner::createPlanningTunnel(std::shared_ptr<octomap::ColorOcTree> tree,
                                                                                                          const octomap::point3d &start) {

  octomap::ColorOcTree tree_copy = octomap::ColorOcTree(*tree);

  std::vector<octomap::point3d> tunnel;

  octomap::point3d current_coords    = start;
  auto             tree_query = tree_copy.search(current_coords);

  if (tree_query != NULL && tree_query->getValue() != TreeValue::FREE) {

    RCLCPP_INFO(node_->get_logger(), "[Astar]: start is inside of an inflated obstacle, tunneling out");

    // tunnel out of expanded walls

    auto edf = euclideanDistanceTransform(tree);
    int iter1 = 0;

    while (rclcpp::ok() && tree_query != NULL && iter1++ <= 100) {

      if (iter1++ > 100) {
        return {};
      }

      tunnel.push_back(current_coords);
      tree_copy.setNodeValue(current_coords, TreeValue::FREE);

      float            obstacle_dist;
      octomap::point3d closest_obstacle;

      edf.getDistanceAndClosestObstacle(current_coords, obstacle_dist, closest_obstacle);
      octomap::point3d dir_away_from_obstacle = current_coords - closest_obstacle;

      if (obstacle_dist >= safe_obstacle_distance) {
        RCLCPP_INFO(node_->get_logger(), "[Astar]: tunnel created with %d", int(tunnel.size()));
        break;
      }

      current_coords += dir_away_from_obstacle.normalized() * float(tree_copy.getResolution());

      int iter2 = 0;

      while (tree_copy.search(current_coords) == tree_query) {

        if (iter2++ > 100) {
          return {};
        }

        current_coords += dir_away_from_obstacle.normalized() * float(tree_copy.getResolution());
      }

      tree_query = tree_copy.search(current_coords);
    }
  }

  std::pair<octomap::ColorOcTree, std::vector<octomap::point3d>> result = {tree_copy, tunnel};

  return result;
}

//}

/* filterPath() //{ */

std::vector<octomap::point3d> AstarPlanner::filterPath(const std::vector<octomap::point3d> &waypoints, octomap::ColorOcTree &tree) {

  if (waypoints.size() < 3) {
    RCLCPP_INFO(node_->get_logger(), "[Astar]: Not enough points for filtering!");
    return waypoints;
  }

  /* removing obsolete points //{ */

  std::vector<octomap::point3d> filtered;

  filtered.push_back(waypoints.front());

  size_t k = 2;

  while (k < waypoints.size()) {

    if (!freeStraightPath(filtered.back(), waypoints[k], tree)) {
      filtered.push_back(waypoints[k - 1]);
    }

    k++;
  }

  filtered.push_back(waypoints.back());
  //}

  return filtered;
}
//}

/* prepareOutputPath() //{ */

std::vector<octomap::point3d> AstarPlanner::prepareOutputPath(const std::vector<octomap::OcTreeKey> &keys, octomap::ColorOcTree &tree) {
  auto waypoints = keysToCoords(keys, tree);
  auto processed = filterPath(waypoints, tree);

  return processed;
}  // namespace navigation
//}

/* generateTemporaryGoal() //{ */

std::pair<octomap::point3d, bool> AstarPlanner::generateTemporaryGoal(const octomap::point3d &start, const octomap::point3d &goal,
                                                                      octomap::ColorOcTree &tree) {

  bool             vertical_priority = false;
  octomap::point3d temp_goal;

  // check if it is necessary to change altitude and acquire more of map
  if (std::abs(goal.z() - start.z()) > planning_tree_resolution) {
    vertical_priority = true;
    RCLCPP_INFO(node_->get_logger(), "[Astar]: give priority to vertical motion");
    temp_goal.x() = start.x();
    temp_goal.y() = start.y();
    // temp_goal.z() = goal.z();  // scan new layers of octomap if needed
    
    double extra_motion = goal.z() - start.z();
    extra_motion        = (extra_motion / std::abs(extra_motion)) * planning_tree_resolution;

    temp_goal.z()   = goal.z() + extra_motion;      // scan new layers of octomap in that case

    if (temp_goal.z() > max_altitude) {
      RCLCPP_INFO(node_->get_logger(), "[Astar]: capping at max altitude");
      temp_goal.z() = max_altitude;
    }
    if (temp_goal.z() < min_altitude) {
      RCLCPP_INFO(node_->get_logger(), "[Astar]: capping at min altitude");
      temp_goal.z() = min_altitude;
    }
    return {temp_goal, vertical_priority};
  }

  // try to explore unknown cells
  std::set<std::pair<octomap::ColorOcTree::iterator, double>, LeafComparator> leafs;

  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); it++) {

    if (it->getValue() == TreeValue::OCCUPIED) {
      continue;
    }

    auto k = it.getKey();
    k.k[2] += 1;

    if (tree.search(k) == NULL) {
      continue;
    }

    k.k[2] -= 2;

    if (tree.search(k) == NULL) {
      continue;
    }

    leafs.insert({it, distEuclidean(it.getCoordinate(), goal)});
  }

  // sort free nodes on the map edge by their distance from goal
  if (!leafs.empty()) {
    // select the closest point
    return {leafs.begin()->first.getCoordinate(), vertical_priority};
  }

  // solution that is only good for smaller obstacles
  octomap::KeyRay ray;
  tree.computeRayKeys(start, goal, ray);

  for (auto &k : ray) {
    auto coords = tree.keyToCoord(k);
    if (tree.search(coords) != NULL && tree.search(coords)->getValue() == TreeValue::FREE) {
      temp_goal = coords;
    }
  }

  return {temp_goal, vertical_priority};
}
//}
