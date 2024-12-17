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
    void resolveConflict(size_t t, id_t conflict_id, id_t end_id, std::vector<std::string>& names, std::map<std::string, std::vector<id_t>>& paths, std::map<id_t, GVertex>& graph);
    std::map<std::string, std::vector<id_t>> generatePaths(const std::vector<id_t> start_ids, const std::vector<std::string> names, const id_t end_id, std::map<id_t, GVertex>& graph);
};
