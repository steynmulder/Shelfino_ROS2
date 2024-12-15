#include <memory>
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "path_interface/srv/generate_graph.hpp"
#include "GVertex.h"
#include "GEdge.h"

using std::placeholders::_1;
using namespace std;

class RoadmapServer : public rclcpp::Node
{
  public:
    RoadmapServer()
    : Node("send_roadmap")
    {
      subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", 10, bind(&RoadmapServer::obstacles_callback, this, _1));
      subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", 10, bind(&RoadmapServer::borders_callback, this, _1));
      service_ = this->create_service<path_interface::srv::GenerateGraph>(
            "generate_graph", std::bind(&RoadmapServer::generateGraph, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:

    typedef enum {FREE, OCCUPIED, MIXED, OUTSIDE} STATUS;

    const float min_length = 0.1;

    id_t id = 0;

    struct Point {
        float x, y;
        Point(float _x, float _y) : x(_x), y(_y) {}
    };

    struct Edge {
        Point p1, p2;
        Edge(Point _p1, Point _p2) : p1(_p1), p2(_p2) {}
    };

    struct QuadNode {
        float x, y, size;
        STATUS status;
        vector<QuadNode> children;
        id_t id;

        QuadNode(float _x, float _y, float _size, STATUS _status) : x(_x), y(_y), size(_size), status(_status) {}
    };

    vector<Edge> total_edges;
    vector<Point> wall_vertices;
    vector<vector<Point>> obstacle_vertices;
    unordered_map<id_t, GVertex> vertices;

    // function<bool (const Edge& a, const Edge& b)> edgeCompare = [](const Edge& a, const Edge& b) {
    //     float min_a = min(a.p1.x, a.p2.x);
    //     float min_b = min(b.p1.x, b.p2.x);

    //     if (min_a == min_b) {
    //         float max_a_y = max(a.p1.y, a.p2.y);
    //         float max_b_y = max(b.p1.y, b.p2.y);
    //         return max_a_y > max_b_y;
    //     }

    //     return min_a < min_b;
        
    // };

    bool linesIntersect(Point& p1, Point& p2, Point& q1, Point& q2) {
        Point vp = {p2.x - p1.x, p2.y - p1.y};
        Point vq = {q2.x - q1.x, q2.y - q1.y};
        float denominator = vp.x * vq.x - vp.y * vq.y;

        if (denominator == 0) {
            return false;
        }

        Point d = {q1.x - p1.x, q1.y - p1.y};
        float ip = (d.x * vp.x - d.y * vp.y);
        float iq = (d.x * vq.x - d.y * vq.y);

        return (ip >= 0 && ip <= 1) && (iq >= 0 && iq <= 1);
    }

    bool inBoundingBox(Point& p, QuadNode& node) {
        return p.x >= node.x && p.x <= (node.x + node.size) && p.y >= node.y && p.y <= (node.y + node.size);
    }

    bool edgeIntersectsNode(QuadNode& node, Edge& edge) {

        pair<Point, Point> top_edge = {{node.x, node.y}, {node.x + node.size, node.y}};
        pair<Point, Point> bottom_edge = {{node.x, node.y + node.size}, {node.x + node.size, node.y + node.size}};
        pair<Point, Point> left_edge = {{node.x, node.y}, {node.x, node.y + node.size}};
        pair<Point, Point> right_edge = {{node.x + node.size, node.y + node.size}, {node.x + node.size, node.y + node.size}};

        // TODO check if y grows downward
        if (linesIntersect(edge.p1, edge.p2, top_edge.first, top_edge.second) ||
            linesIntersect(edge.p1, edge.p2, bottom_edge.first, bottom_edge.second) || // node bottom edge
            linesIntersect(edge.p1, edge.p2, left_edge.first, left_edge.second) || // node left edge
            linesIntersect(edge.p1, edge.p2, right_edge.first, right_edge.second) || // node right edge
            inBoundingBox(edge.p1, node) ||
            inBoundingBox(edge.p2, node)
            ) {
            return true;
        }

        return false;
    }

    bool pointInPolygon(Point& p, vector<Point>& polygon) {
        int n = polygon.size();
        bool inside = false;

        for (int i = 0, j = n - 1; i < n; j = i++) {
            if ((polygon[i].y > p.y) != (polygon[j].y > p.y) &&
                (p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)) {
                inside = !inside;
            }
        }

        return inside;
    }

    bool nodeInPolygon(QuadNode& node, vector<Point>& polygon) {
        Point tl = {node.x, node.y};
        Point tr = {node.x + node.size, node.y};
        Point bl = {node.x, node.y + node.size};
        Point br = {node.x + node.size, node.y + node.size};
        if (!pointInPolygon(tl, polygon) ||
            !pointInPolygon(tr, polygon) ||
            !pointInPolygon(bl, polygon) ||
            !pointInPolygon(br, polygon)
        ) {
            return false;
        }

        return true;
    }

    STATUS node_free(QuadNode& node) {
        if (!nodeInPolygon(node, wall_vertices)) { // node outside of wall bounds
            return OUTSIDE;
        }

        for (vector<Point>& edges : obstacle_vertices) { // node completely in polygon
            if (nodeInPolygon(node, edges)) {
                return OCCUPIED;
            }
        }

        for (Edge& edge : total_edges) {
            if (edgeIntersectsNode(node, edge)) {
                return MIXED;
            }
        }

        return FREE;
    }

    void subdivide(QuadNode& node) {
        float size = node.size / 2;
        node.children.push_back({node.x, node.y, size, MIXED});
        node.children.push_back({node.x + size, node.y, size, MIXED});
        node.children.push_back({node.x, node.y + size, size, MIXED});
        node.children.push_back({node.x + size, node.y + size, size, MIXED});
    }

    void decompose(QuadNode& node) {
        if (node.size <= min_length || node.status != MIXED) {
            return;
        }

        STATUS status = node_free(node);

        if (status == FREE || status == OCCUPIED) {
            node.status = status;
        } else {
            subdivide(node);
            for (QuadNode& n : node.children) {
                decompose(n);
            }
        }
    }

    float euclidianDistance(QuadNode& n1, QuadNode& n2) {
        return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
    }

    void buildGraph(QuadNode& node) {
        if (node.status == FREE) {
            GVertex v = {id, "", node.x + node.size / 2, node.y + node.size / 2, 0.0};
            vertices[id] = v;
            node.id = id;
            ++id;
            return;
        }
         
        if (!node.children.empty()) {
            for (QuadNode& n : node.children) {
                buildGraph(n);
            }
        }

        if (!node.children.empty()) {
            for (size_t i = 0; i < node.children.size(); ++i) {
                for (size_t j = i + 1; j < node.children.size(); ++j) {
                    if (node.children[j].status != FREE) {
                        continue;
                    }

                    QuadNode node1 = node.children[i];
                    QuadNode node2 = node.children[j];

                    if (abs(node1.x - node2.x) <= node1.size &&
                        abs(node1.y - node2.y) <= node1.size) {
                            vertices[node1.id].addEdge(node2.id, euclidianDistance(node1, node2), "");
                            vertices[node2.id].addEdge(node1.id, euclidianDistance(node1, node2), "");
                    }
                }
            }
        }
    }
    
    void generateGraph(const std::shared_ptr<path_interface::srv::GenerateGraph::Request> request, 
        std::shared_ptr<path_interface::srv::GenerateGraph::Response>      response) {

        float min_x = INFINITY, max_x = -INFINITY, min_y = INFINITY, max_y = -INFINITY;
        for (const Edge& edge : total_edges) {
            if (edge.p1.x < min_x) {
                min_x = edge.p1.x;
            }

            if (edge.p2.x < min_x) {
                min_x = edge.p2.x;
            }

            if (edge.p1.x > max_x) {
                max_x = edge.p1.x;
            }

            if (edge.p2.x > max_x) {
                max_x = edge.p2.x;
            }

            if (edge.p1.y < min_y) {
                min_y = edge.p1.y;
            }

            if (edge.p2.y < min_y) {
                min_y = edge.p2.y;
            }

            if (edge.p1.y > max_y) {
                max_y = edge.p1.y;
            }

            if (edge.p2.y > max_y) {
                max_y = edge.p2.y;
            }
        }

        QuadNode boundary = {min_x, min_y, fmax(max_x - min_x, max_y - min_y), MIXED};

        decompose(boundary);

        buildGraph(boundary);



        for (auto v = vertices.begin(); v != vertices.end();) {
            if (v->second.getEdgeList().empty()) {
                v = vertices.erase(v);
            } else {
                ++v;
            }
        }

        RCLCPP_INFO(this->get_logger(), "We have %zu vertices", vertices.size());   

        //TODO add gate node and nodes for initial position of robots
        // TODO make response


        
        // sort(total_edges.begin(), total_edges.end(), edgeCompare);
        
    }

    // struct Event {
    //     float x;
    //     Edge edge;
    //     bool isStart;
    //     Event(float _x, Edge _edge, bool _isStart) : x(_x), edge(_edge), isStart(_isStart) {}
    // };

    // // Comparator for sorting events by x-coordinate (and y as tie-breaker)
    // bool eventComparator(const Event& e1, const Event& e2) {
    //     if (e1.x != e2.x) return e1.x < e2.x;
    //     return e1.edge.p1.y < e2.edge.p1.y; // Tie-break by y-coordinate
    // }

    // // Comparator for ordering edges by their intersection with the sweep line
    // struct EdgeComparator {
    //     bool operator()(const Edge& e1, const Edge& e2) const {
    //         if (e1.p1.y == e2.p1.y) return e1.p2.y < e2.p2.y;
    //         return e1.p1.y < e2.p1.y;
    //     }
    // };


    // vector<Edge> obstacle_vertices;

    // // Find intersection of an edge with the sweep line at x
    // float getYIntersection(const Edge& edge, float x) {
    //     if (edge.p1.x == edge.p2.x) return edge.p1.y; // Vertical line
    //     float slope = (edge.p2.y - edge.p1.y) / (edge.p2.x - edge.p1.x);
    //     return edge.p1.y + slope * (x - edge.p1.x);
    // }

    // // Calculate the midpoint of two points
    // Point midpoint(const Point& p1, const Point& p2) {
    //     return Point((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
    // }

    // // Generate area segments and build the graph
    // void generateGraph(const std::shared_ptr<path_interface::srv::GenerateGraph::Request> request,
    //       std::shared_ptr<path_interface::srv::GenerateGraph::Response>      response) {
    //     vector<Event> events;

    //     // Create events for all edges
    //     for (const auto& edge : obstacle_vertices) {
    //         events.emplace_back(edge.p1.x, edge, true);  // Start event
    //         events.emplace_back(edge.p2.x, edge, false); // End event
    //     }

    //     // Sort events by x-coordinate
    //     sort(events.begin(), events.end(), [this](const Event& a, const Event& b) {
    //                   return eventComparator(a, b);
    //               });

    //     // Active edge set
    //     set<Edge, EdgeComparator> activeEdges;

    //     // Resultant graph vertices and edges
    //     vector<Point> vertices; // All vertices
    //     vector<pair<Point, Point>> edgesGraph; // Edges of the graph

    //     vector<Vertex> vertices_astar;
    //     id_t id = 0;

    //     float lastX = -1; // Track the previous x-position of the sweep line
    //     vector<Point> lastCenters; // Centers of areas at the last step

    //     for (const auto& event : events) {
    //         float currentX = event.x;

    //         // Process active edges and generate area segments at the midpoint
    //         vector<Point> centers;

    //         if (!activeEdges.empty() && lastX != -1) {
    //             auto it = activeEdges.begin();
    //             while (next(it) != activeEdges.end()) {
    //                 auto currentEdge = *it;
    //                 auto nextEdge = *next(it);

    //                 // Compute center of the area between two edges
    //                 float y1 = getYIntersection(currentEdge, currentX);
    //                 float y2 = getYIntersection(nextEdge, currentX);
    //                 float centerX = (lastX + currentX) / 2;
    //                 float centerY = (y1 + y2) / 2;

    //                 centers.push_back(Point(centerX, centerY));
    //                 ++it;
    //             }
    //         }

    //         // Add connections between current centers and previous centers
    //         for (size_t i = 0; i < centers.size(); ++i) {
    //             vertices.push_back(centers[i]);
    //             Vertex v = {id, "", centers[i].x, centers[i].y, 0.0};
    //             vertices_astar.push_back(v);
    //             id++;


    //             // Connect area centers
    //             if (i > 0) {
    //                 edgesGraph.emplace_back(centers[i - 1], centers[i]);
    //             }

    //             // Connect to previous centers
    //             if (!lastCenters.empty() && i < lastCenters.size()) {
    //                 Point edgeCenter = midpoint(centers[i], lastCenters[i]);
    //                 vertices.push_back(edgeCenter);

    //                 Vertex v = {id, "", edgeCenter.x, edgeCenter.y, 0.0};
    //                 vertices_astar.push_back(v);
    //                 id++;

    //                 edgesGraph.emplace_back(centers[i], edgeCenter);
    //                 edgesGraph.emplace_back(lastCenters[i], edgeCenter);
    //             }
    //         }

    //         // Update active edges
    //         if (event.isStart) {
    //             activeEdges.insert(event.edge);
    //         } else {
    //             activeEdges.erase(event.edge);
    //         }

    //         // Update last state
    //         lastCenters = centers;
    //         lastX = currentX;
    //     }

    //     path_interface::msg::Graph graph;
    //     vector<path_interface::msg::GraphEdge> edges;
    //     for (auto const &edge : edgesGraph) {
    //       path_interface::msg::GraphEdge e;
    //       path_interface::msg::GraphNode n1;
    //       path_interface::msg::GraphNode n2;
    //       n1.x = edge.first.x; n1.y = edge.first.y;
    //       n2.x = edge.second.x; n2.y = edge.second.y;

    //       e.start = n1;
    //       e.end = n2;

    //       edges.push_back(e);
    //       RCLCPP_INFO(this->get_logger(), "Next edge: ('%f', '%f'), ('%f', '%f')", n1.x, n1.y, n2.x, n2.y);
    //     }
    //     RCLCPP_INFO(this->get_logger(), "I AM HERE");
    //     graph.edges = edges;
    //     graph.vertices = vertices;
    //     response->graph = graph;
    //     RCLCPP_INFO(this->get_logger(), "%zu", graph.edges.size());
    // }



    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "HERE");
        for (const auto &obs : msg->obstacles) {
            vector<geometry_msgs::msg::Point32> points = obs.polygon.points;
            vector<Point> polygon;
            for (unsigned i = 0; i < points.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "I heard an obstacle vertex: '%f', '%f", points[i].x, points[i].y);
                total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
                polygon.emplace_back(Point(points[i].x, points[i].y));
            }
            
            obstacle_vertices.emplace_back(polygon);
            
        }
      
    }

    void borders_callback(const geometry_msgs::msg::Polygon msg)
    {
        vector<geometry_msgs::msg::Point32> points = msg.points;
        for (unsigned i = 0; i < points.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f", points[i].x, points[i].y);
            total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
            wall_vertices.push_back(Point(points[i].x, points[i].y));
        }
            
      
    }
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;
    rclcpp::Service<path_interface::srv::GenerateGraph>::SharedPtr service_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RoadmapServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}