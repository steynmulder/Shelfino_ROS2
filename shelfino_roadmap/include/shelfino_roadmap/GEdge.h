#pragma once

#include <string>

// Edges represent a road segment between two intersections
class GEdge
{
private:
    id_t source_vid;
    id_t dest_vid;
    double length;
    std::string name;

public:
    GEdge(const id_t, const id_t, const double, const std::string &);
    GEdge(const GEdge &);
    ~GEdge() = default;

    id_t getSourceVID() const;
    id_t getDestVID() const;
    double getLength() const;
    std::string getName() const;

    void setLength(double length);  
    void updateLengthWithDubins(double sx, double dx, double sy, double dy, double sor, double dor, double turningRadius);  

    void print();
    GEdge &operator=(const GEdge &);
};