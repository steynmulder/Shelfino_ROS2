#pragma once

#include "GEdge.h"
#include <list>
#include <vector>
#include <string>

// Vertices represent road intersections
class GVertex
{
private:
    id_t state_id_;
    std::string state_name_;
    std::vector<GEdge> edges_;  // List of edges connected to this vertex

    double x_;      // X-coordinate
    double y_;      // Y-coordinate
    double theta_;  // Orientation (in radians)

    double g_;  // Cost from start to this vertex
    double f_;  // Total cost (g + heuristic)

    id_t predecessor_;  // ID of the predecessor vertex

public:
    // Constructors
    GVertex();
    GVertex(const id_t state_id, const std::string& state_name = "", double x = 0, double y = 0, double theta = 0.0);
    ~GVertex() = default;

    // Operators for comparisons
    bool operator<(const GVertex& e) const;
    bool operator>(const GVertex& e) const;
    bool operator==(const GVertex& e) const;

    // Getters for position and orientation
    inline double getx() const { return x_; }
    inline double gety() const { return y_; }
    inline double getOrientation() const { return theta_; }

    // Setter for orientation
    inline void setOrientation(double theta) { theta_ = theta; }

    // Getters and setters for costs
    inline double getf() const { return f_; }
    inline double getg() const { return g_; }
    inline void setf(const double f) { f_ = f; }
    inline void setg(const double g) { g_ = g; }

    // Predecessor management
    id_t getPredecessor() const { return predecessor_; }
    void setPredecessor(GVertex* predecessor) { predecessor_ = predecessor->getStateID(); }

    // Vertex identification
    inline id_t getStateID() const { return state_id_; }
    inline std::string getStateName() const { return state_name_; }

    // Edge management
    inline std::vector<GEdge> getEdgeVector() const { return this->edges_; }
    std::list<GEdge> getEdgeList();
    std::list<id_t> getDestinationsList() const;

    // Edge addition methods
    void addEdge(id_t dest_vertex_id, const double length, const std::string& name);
    // void addEdgeWithDubins(id_t dest_vertex_id, const GVertex& dest_vertex, double turningRadius, const std::string& name);

    // Utility methods
    void printEdgeList() const;
    double getLengthToNeighbourVertex(id_t neighbour_id);
};
