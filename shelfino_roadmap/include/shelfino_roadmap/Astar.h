#pragma once

#include "Vertex.h"
#include "Dubins.h"  

class Astar
{
private:
    double heuristic_distance_estimator(const Vertex& vnext, const Vertex& vend);
    double dubins_distance(const Vertex& vcurrent, const Vertex& vnext);  // Add a helper function for Dubins path cost

    struct CompareF {
        bool operator()(const Vertex* a, const Vertex* b) const {
            return a->getf() < b->getf();
        }
    };

public:
    std::vector<id_t> findPath(const id_t start_id, const id_t end_id);
};
