#include "RRTstar.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>

struct Node {
    id_t id;
    double x, y;
    Node* parent;
    double cost;

    Node(id_t id, double x, double y, Node* parent = nullptr, double cost = 0.0)
        : id(id), x(x), y(y), parent(parent), cost(cost) {}
};

class RRTstar {
public:
    RRTstar(Graph* graph, double step_size, double search_radius)
        : graph(graph), step_size(step_size), search_radius(search_radius) {}

    std::vector<id_t> findPath(double start_x, double start_y, double goal_x, double goal_y);

private:
    Graph* graph;
    double step_size;
    double search_radius;

    Node* nearestNode(const std::vector<Node*>& tree, double x, double y);
    std::vector<Node*> nearNodes(const std::vector<Node*>& tree, double x, double y);
    bool isCollisionFree(double x1, double y1, double x2, double y2); // Stub for collision checking
    double euclideanDistance(double x1, double y1, double x2, double y2);
    void rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& near_nodes);
};

// Main RRT* pathfinding function
std::vector<id_t> RRTstar::findPath(double start_x, double start_y, double goal_x, double goal_y) {
    std::vector<Node*> tree;
    tree.push_back(new Node(0, start_x, start_y));

    Node* goal_node = nullptr;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(0, graph->getWidth()); // Define bounds for x and y
    std::uniform_real_distribution<> dis_y(0, graph->getHeight());

    while (goal_node == nullptr) {
        double rand_x = dis_x(gen);
        double rand_y = dis_y(gen);

        Node* nearest = nearestNode(tree, rand_x, rand_y);
        double theta = atan2(rand_y - nearest->y, rand_x - nearest->x);
        double new_x = nearest->x + step_size * cos(theta);
        double new_y = nearest->y + step_size * sin(theta);

        if (!isCollisionFree(nearest->x, nearest->y, new_x, new_y))
            continue;

        Node* new_node = new Node(tree.size(), new_x, new_y, nearest, nearest->cost + step_size);
        tree.push_back(new_node);

        auto near_nodes = nearNodes(tree, new_x, new_y);
        rewire(tree, new_node, near_nodes);

        if (euclideanDistance(new_x, new_y, goal_x, goal_y) <= step_size) {
            goal_node = new Node(tree.size(), goal_x, goal_y, new_node, new_node->cost + step_size);
            tree.push_back(goal_node);
        }
    }

    // Reconstruct path from goal_node
    std::vector<id_t> path;
    Node* current = goal_node;
    while (current) {
        path.push_back(current->id);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

Node* RRTstar::nearestNode(const std::vector<Node*>& tree, double x, double y) {
    Node* nearest = nullptr;
    double min_distance = std::numeric_limits<double>::max();

    for (Node* node : tree) {
        double distance = euclideanDistance(node->x, node->y, x, y);
        if (distance < min_distance) {
            min_distance = distance;
            nearest = node;
        }
    }
    return nearest;
}

std::vector<Node*> RRTstar::nearNodes(const std::vector<Node*>& tree, double x, double y) {
    std::vector<Node*> near_nodes;
    for (Node* node : tree) {
        if (euclideanDistance(node->x, node->y, x, y) <= search_radius) {
            near_nodes.push_back(node);
        }
    }
    return near_nodes;
}

bool RRTstar::isCollisionFree(double x1, double y1, double x2, double y2) {
    // TODO check for collision
    return true;
}

double RRTstar::euclideanDistance(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void RRTstar::rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& near_nodes) {
    for (Node* near_node : near_nodes) {
        double new_cost = new_node->cost + euclideanDistance(new_node->x, new_node->y, near_node->x, near_node->y);
        if (new_cost < near_node->cost) {
            near_node->parent = new_node;
            near_node->cost = new_cost;
        }
    }
}
