#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

   start_node = &m_Model.FindClosestNode(start_x,start_y);
   end_node  = &m_Model.FindClosestNode(end_x,end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    //-Task 3 start
      float hValue = node->distance(*this->end_node);
      return hValue;
    //-Task 3 end
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();

    for(size_t a = 0; a < current_node->neighbors.size();a++)
    {
      if(!current_node->neighbors[a]->visited)
      {
           current_node->neighbors[a]->parent = current_node;
           current_node->neighbors[a]->g_value = current_node->g_value + current_node->neighbors[a]->distance(*current_node);
           current_node->neighbors[a]->h_value = CalculateHValue(current_node->neighbors[a]);
           this->open_list.push_back(current_node->neighbors[a]);
           current_node->neighbors[a]->visited = true;
      }
    }
}

// Function to sort the map according
// to value in a (key-value) pairs
namespace local
{

    bool cmp(std::pair<RouteModel::Node*,float>& a,
             std::pair<RouteModel::Node*,float>& b)
    {
        return a.second < b.second;
    }

    std::vector<std::pair<RouteModel::Node*,float>> sort(std::unordered_map<RouteModel::Node*,float>& M)
    {

        // Declare vector of pairs
        std::vector<std::pair<RouteModel::Node*,float> > A;

        // Copy key-value pair from Map
        // to vector of pairs
        for (auto& it : M) {
            A.push_back(it);
        }

        // Sort using comparator function
        sort(A.begin(), A.end(), cmp);
        return A;
    }
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::unordered_map<RouteModel::Node*,float> myMap;
    for(size_t a = 0; a < open_list.size();a++)
    {
        myMap.insert({open_list[a],open_list[a]->h_value + open_list[a]->g_value});
    }
    std::vector<std::pair<RouteModel::Node*,float>> sorted = local::sort(myMap);

    for(size_t b = 0; b < sorted.size();b++)
    {
        open_list[b] = sorted[b].first;
    }
    open_list.erase(open_list.begin());

    return sorted[0].first;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    // For each node in the path

    while (current_node->parent != nullptr) {

        // Add the distance
        distance += current_node->distance(*current_node->parent);
        // Store node in the path
        path_found.push_back(*current_node);

        // Move to the parent node
        current_node = current_node->parent;
    }

    // Push back initial node
    path_found.push_back(*current_node);

    // Sort path (end2start) -> (start2end)
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    this->open_list.push_back(this->start_node); // Add start node to open list
    this->start_node->visited = true;

    // While there are still nodes to check
    while (open_list.size() > 0) {

        // Get the next node
        current_node = this->NextNode();

        if (current_node == this->end_node) {
            this->m_Model.path = this->ConstructFinalPath(current_node);
            return;
        }

        // If not done, expand search to current node's neighbors
        this->AddNeighbors(current_node);
    }

    // If the search is not successful
    std::cout << "No path was found! :(\n";
    return;
}
