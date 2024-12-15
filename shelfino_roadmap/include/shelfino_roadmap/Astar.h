#pragma once

#include <map>

#include "GVertex.h"
#include "Dubins.h"  

class Astar
{
private:
    double heuristic_distance_estimator(const GVertex& vnext, const GVertex& vend);
    double dubins_distance(const GVertex& vcurrent, const GVertex& vnext);  // Add a helper function for Dubins path cost

    struct CompareF {
        bool operator()(const GVertex* a, const GVertex* b) const {
            return a->getf() < b->getf();
        }
    };

public:
    std::vector<id_t> findPath(const id_t start_id, const id_t end_id, std::map<id_t, GVertex>& graph);
};
