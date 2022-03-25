#include "route_planner.h"
#include <algorithm>

/**
 * @brief Construct a new Route Planner:: Route Planner object.
 *
 * @param model RouteModel object
 * @param start_x start node x coordinate
 * @param start_y start node y coordinate
 * @param end_x end node x coordinate
 * @param end_y end node y coordinate
 */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // convert inputs to percentage
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  this->start_node = &m_Model.FindClosestNode(start_x, start_y);
  this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

/**
 * @brief Return the Euclidean distance between end node and current node.
 *
 * @param node current node
 * @return float
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*(this->end_node));
}

/**
 * @brief Add current node's neighboring nodes to the search vector with
 * updated values.
 *
 * @param current_node
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();

  for (auto &neighbor_node : current_node->neighbors) {
    if (neighbor_node->visited != true) {
      neighbor_node->visited = true;
      neighbor_node->parent = current_node;
      neighbor_node->h_value = CalculateHValue(neighbor_node);
      neighbor_node->g_value =
          current_node->g_value + current_node->distance(*neighbor_node);
      this->open_list.emplace_back(neighbor_node);
    }
  }
}

/**
 * @brief Find and return the node of lowest f-value among the vector of nodes
 * to be searched.
 *
 * @return RouteModel::Node*
 */
RouteModel::Node *RoutePlanner::NextNode() {
  sort(open_list.begin(), open_list.end(),
       [](const RouteModel::Node *a, const RouteModel::Node *b) {
         return a->g_value + a->h_value > b->g_value + b->h_value;
       });
  RouteModel::Node *lowest = this->open_list.back();
  this->open_list.pop_back();
  return lowest;
}

/**
 * @brief Construct final path found from start to end nodes.
 *
 * @param current_node
 * @return std::vector<RouteModel::Node>
 */
std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node != this->start_node) {
    distance += current_node->distance(*current_node->parent);
    path_found.emplace_back(*current_node);
    current_node = current_node->parent;
  }

  path_found.emplace_back(*this->start_node);
  std::reverse(path_found.begin(), path_found.end());
  distance *= m_Model.MetricScale();

  return path_found;
}

/**
 * @brief Construct path from start node to end node using A* heuristic search.
 *
 */
void RoutePlanner::AStarSearch() {

  RouteModel::Node *current_node = this->start_node;

  while (current_node != this->end_node) {
    current_node->visited = true;
    AddNeighbors(current_node);
    current_node = NextNode();
  }

  this->m_Model.path = ConstructFinalPath(current_node);
}