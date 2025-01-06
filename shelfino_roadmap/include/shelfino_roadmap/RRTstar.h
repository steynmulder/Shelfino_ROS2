#pragma once

#include <vector>
#include "Dubins.h"  
#include <map>

class RRTstar
{

    public:

// Helper functions
    struct Node {
        id_t id;
        double x, y;
        Node* parent;
        double cost;

        Node(id_t id, double x, double y, Node* parent = nullptr, double cost = 0.0)
            : id(id), x(x), y(y), parent(parent), cost(cost) {}
    };

    struct Obstacle {
	bool is_cylinder;
	double x,y;
	double radius;
	double length_vertice;
	std::vector<std::pair<double,double>> vertices;	
};


    struct Point {
        float x, y;
        Point() = default;
        Point(float _x, float _y) : x(_x), y(_y) {}
        bool operator==(const Point &other) const
        { return (x == other.x&& y == other.y);}
    };

    struct Graph{
        std::vector<Obstacle> obstacles;
        std::vector<Point> borders;	
        

        double getWidthStart()const{
            double min_x = borders[0].x;
            for(const auto& p: borders){
                if(p.x < min_x) min_x = p.x;
            }
            return min_x;
        }

        double getWidthEnd()const{
            double max_x = borders[0].x;
            for(const auto& p: borders){
                if(p.x > max_x) max_x = p.x;
            }
            return max_x;
        }

        double getHeightStart()const{
            double min_y = borders[0].y;
            for(const auto& p: borders){
                if(p.y < min_y) min_y = p.y;
            }
            return min_y;
        }

        double getHeightEnd()const{
            double max_y = borders[0].y;
            for(const auto& p: borders){
                if(p.y > max_y) max_y = p.y;
            }
            return max_y;
        }

        double getWidth() const { return getWidthEnd() - getWidthStart();};
        double getHeight() const { return getHeightEnd() - getHeightStart();};

    };
    // Constructor inheriting from Algorithm and initializing RRT* parameters
    RRTstar(Graph* graph, double step_size, double search_radius)
    : graph(graph), step_size(step_size), search_radius(search_radius) {}



    std::vector<RRTstar::Point> findPath(double start_x, double start_y, double goal_x, double goal_y);

private:
    double step_size;     // Maximum step size for tree expansion
    double search_radius; // Radius for considering nodes during rewiring
    RRTstar::Graph* graph;
    



    Node* nearestNode(const std::vector<Node*>& tree, double x, double y);
    std::vector<Node*> nearNodes(const std::vector<Node*>& tree, double x, double y);
    bool isCollisionFree(double x1, double y1, double x2, double y2); // Stub for collision checking
    double euclideanDistance(double x1, double y1, double x2, double y2);
    void rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& near_nodes);


};
