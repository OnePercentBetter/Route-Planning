#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    
    start_node = &model.FindClosestNode(start_x, start_y);
  	end_node = &model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for(auto n : current_node->neighbors) {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value = current_node->g_value + current_node->distance(*n);
        open_list.emplace_back(n);
        n->visited = true;
    }
}

bool compare(const RouteModel::Node* f1,const RouteModel::Node *f2) {
    float first = f1 -> g_value + f1 -> h_value;
    float second = f2 -> g_value + f2 -> h_value;
    return f1 > f2;

}
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), compare);
    RouteModel::Node* lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node -> parent) {
        path_found.insert(path_found.begin(),*current_node -> parent);
        distance += current_node->distance(*(current_node -> parent));
        current_node = current_node -> parent;
    }
    path_found.insert(path_found.begin(), *current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    current_node = start_node;
    current_node ->visited = true;
    open_list.emplace_back(current_node);
    while(open_list.size() > 0) {
        current_node = NextNode();

        if(current_node ->x == end_node ->x && current_node ->y == end_node -> y ) {
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }

}