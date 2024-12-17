#pragma once

#include "Vertex.h"
#include "Algorithm.h"
#include "id_type.h"
#include <vector>

class RRTstar : public Algorithm
{
private:
    double step_size;     // Maximum step size for tree expansion
    double search_radius; // Radius for considering nodes during rewiring

    // Helper functions
    struct Node {
        id_t id;
        double x, y;
        Node* parent;
        double cost;

        Node(id_t id, double x, double y, Node* parent = nullptr, double cost = 0.0)
            : id(id), x(x), y(y), parent(parent), cost(cost) {}
    };

    Node* nearestNode(const std::vector<Node*>& tree, double x, double y);
    std::vector<Node*> nearNodes(const std::vector<Node*>& tree, double x, double y);
    bool isCollisionFree(double x1, double y1, double x2, double y2); // Stub for collision checking
    double euclideanDistance(double x1, double y1, double x2, double y2);
    void rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& near_nodes);

public:
    // Constructor inheriting from Algorithm and initializing RRT* parameters
    RRTstar(Graph* graph, double step_size, double search_radius);

    // Override the findPath method from Algorithm
    std::vector<id_t> findPath(const id_t start_id, const id_t end_id) override;
};
