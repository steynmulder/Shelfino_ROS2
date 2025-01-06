#include "GVertex.h"

#include <list>
#include <iostream>
#include <limits>
#include <cmath>

GVertex::GVertex() {}

GVertex::GVertex(const id_t state_id, const std::string &state_name, double x, double y, double theta)
{
    state_id_ = state_id;
    state_name_ = state_name;
    x_ = x;
    y_ = y;
    theta_ = theta;
    g_ = 0;
    f_ = 0;
    predecessor_ = 0;
}

std::list<GEdge> GVertex::getEdgeList()
{
    std::list<GEdge> edge_list(this->edges_.begin(), this->edges_.end());
    return edge_list;
}

std::list<id_t> GVertex::getDestinationsList() const
{
    std::list<id_t> destinations;
    for (auto &e : this->edges_)
        destinations.push_back(e.getDestVID());
    return destinations;
}

void GVertex::printEdgeList() const
{
    std::cout << getStateID() << "[";
    for (auto it = this->edges_.begin(); it != this->edges_.end(); it++)
    {
        std::cout << it->getDestVID() << "(" << it->getLength() << ") --> ";
    }
    std::cout << "]";
    std::cout << std::endl;
}

void GVertex::addEdge(id_t dest_vertex_id, const double length, const std::string &name)
{
    GEdge e(this->getStateID(), dest_vertex_id, length, name);
    this->edges_.push_back(e);
    // std::cout << "Edge between " << state_id_ << " and " << toVertexID << " added Successfully" << std::endl;
}

// void GVertex::addEdgeWithDubins(id_t dest_vertex_id, const GVertex& dest_vertex, double turningRadius, const std::string& name)
// {
//     GEdge e(this->getStateID(), dest_vertex_id, 0.0, name);
//     e.updateLengthWithDubins(*this, dest_vertex, turningRadius);  // Update edge length using Dubins curves
//     this->edges_.push_back(e);
// }

bool GVertex::operator<(const GVertex &v) const
{
    return this->state_id_ < v.state_id_;
}

bool GVertex::operator>(const GVertex &v) const
{
    return this->state_id_ > v.state_id_;
}

bool GVertex::operator==(const GVertex &v) const
{
    return this->state_id_ == v.state_id_;
}

double GVertex::getLengthToNeighbourVertex(id_t neighbour_id)
{
    if (this->getStateID() == neighbour_id)
        return 0;

    for (GEdge &e : this->getEdgeVector())
    {
        if (e.getDestVID() == neighbour_id)
            return e.getLength();
    }
    return std::numeric_limits<double>::max(); // big number :)
}
// double GVertex::getx() const
// {
//     return x_;
// }

// double GVertex::gety() const
// {
//     return y_;
// }

// double GVertex::getOrientation() const
// {
//     return theta_;  
// }

// void GVertex::setOrientation(double theta)
// {
//     theta_ = theta;  
// }
