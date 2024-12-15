#include "GEdge.h"
#include "GVertex.h"  
#include "Dubins.h"  
#include <iostream>

GEdge::GEdge(const id_t source_vid, const id_t dest_vid, const double length, const std::string& name)
{
    this->source_vid = source_vid;
    this->dest_vid = dest_vid;
    this->length = length;  // Initial length can be set directly or calculated later
    this->name = name;
}

GEdge::GEdge(const GEdge& e)
{
    source_vid = e.source_vid;
    dest_vid = e.dest_vid;
    length = e.length;
    name = e.name;
}

double GEdge::getLength() const
{
    return this->length;
}

void GEdge::setLength(double length)
{
    this->length = length;  // Allow dynamic updates to the length
}

// void GEdge::updateLengthWithDubins(const Vertex& source, const Vertex& dest, double turningRadius)
// {
//     // Calculate Dubins path length between source and destination vertices
//     this->length = dubinsShortestPath(
//         source.getx(), source.gety(), source.getOrientation(),
//         dest.getx(), dest.gety(), dest.getOrientation(),
//         turningRadius
//     ).L;
// }

id_t GEdge::getSourceVID() const
{
    return this->source_vid;
}

id_t GEdge::getDestVID() const
{
    return this->dest_vid;
}

std::string GEdge::getName() const
{
    return this->name;
}

void GEdge::print()
{
    std::cout << "Edge " << source_vid << "--" << dest_vid << " length " << length << " name " << name << std::endl;
}

GEdge& GEdge::operator=(const GEdge& e)
{
    source_vid = e.source_vid;
    dest_vid = e.dest_vid;
    length = e.length;
    name = e.name;
    return *this;
}
