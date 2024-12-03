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

using std::placeholders::_1;
using namespace std;

class RoadmapPublisher : public rclcpp::Node
{
  public:
    RoadmapPublisher()
    : Node("send_roadmap")
    {
      subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", 10, bind(&RoadmapPublisher::obstacles_callback, this, _1));
      subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
      "/map_borders", 10, bind(&RoadmapPublisher::borders_callback, this, _1));
      service_ = this->create_service<path_interface::srv::GenerateGraph>(
            "generate_graph", std::bind(&RoadmapPublisher::generateGraph, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:

    struct Point {
        float x, y;
        Point(float _x, float _y) : x(_x), y(_y) {}
    };

    struct Edge {
        Point p1, p2;
        Edge(Point _p1, Point _p2) : p1(_p1), p2(_p2) {}
    };

    struct Event {
        float x;
        Edge edge;
        bool isStart;
        Event(float _x, Edge _edge, bool _isStart) : x(_x), edge(_edge), isStart(_isStart) {}
    };

    // Comparator for sorting events by x-coordinate (and y as tie-breaker)
    bool eventComparator(const Event& e1, const Event& e2) {
        if (e1.x != e2.x) return e1.x < e2.x;
        return e1.edge.p1.y < e2.edge.p1.y; // Tie-break by y-coordinate
    }

    // Comparator for ordering edges by their intersection with the sweep line
    struct EdgeComparator {
        bool operator()(const Edge& e1, const Edge& e2) const {
            if (e1.p1.y == e2.p1.y) return e1.p2.y < e2.p2.y;
            return e1.p1.y < e2.p1.y;
        }
    };


    vector<Edge> obstacle_vertices;

    // Find intersection of an edge with the sweep line at x
    float getYIntersection(const Edge& edge, float x) {
        if (edge.p1.x == edge.p2.x) return edge.p1.y; // Vertical line
        float slope = (edge.p2.y - edge.p1.y) / (edge.p2.x - edge.p1.x);
        return edge.p1.y + slope * (x - edge.p1.x);
    }

    // Calculate the midpoint of two points
    Point midpoint(const Point& p1, const Point& p2) {
        return Point((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
    }

    // Generate area segments and build the graph
    void generateGraph(const std::shared_ptr<path_interface::srv::GenerateGraph::Request> request,
          std::shared_ptr<path_interface::srv::GenerateGraph::Response>      response) {
        vector<Event> events;

        // Create events for all edges
        for (const auto& edge : obstacle_vertices) {
            events.emplace_back(edge.p1.x, edge, true);  // Start event
            events.emplace_back(edge.p2.x, edge, false); // End event
        }

        // Sort events by x-coordinate
        sort(events.begin(), events.end(), [this](const Event& a, const Event& b) {
                      return eventComparator(a, b);
                  });

        // Active edge set
        set<Edge, EdgeComparator> activeEdges;

        // Resultant graph vertices and edges
        vector<Point> vertices; // All vertices
        vector<pair<Point, Point>> edgesGraph; // Edges of the graph

        float lastX = -1; // Track the previous x-position of the sweep line
        vector<Point> lastCenters; // Centers of areas at the last step

        for (const auto& event : events) {
            float currentX = event.x;

            // Process active edges and generate area segments at the midpoint
            vector<Point> centers;

            if (!activeEdges.empty() && lastX != -1) {
                auto it = activeEdges.begin();
                while (next(it) != activeEdges.end()) {
                    auto currentEdge = *it;
                    auto nextEdge = *next(it);

                    // Compute center of the area between two edges
                    float y1 = getYIntersection(currentEdge, currentX);
                    float y2 = getYIntersection(nextEdge, currentX);
                    float centerX = (lastX + currentX) / 2;
                    float centerY = (y1 + y2) / 2;

                    centers.push_back(Point(centerX, centerY));
                    ++it;
                }
            }

            // Add connections between current centers and previous centers
            for (size_t i = 0; i < centers.size(); ++i) {
                vertices.push_back(centers[i]);

                // Connect area centers
                if (i > 0) {
                    edgesGraph.emplace_back(centers[i - 1], centers[i]);
                }

                // Connect to previous centers
                if (!lastCenters.empty() && i < lastCenters.size()) {
                    Point edgeCenter = midpoint(centers[i], lastCenters[i]);
                    vertices.push_back(edgeCenter);
                    edgesGraph.emplace_back(centers[i], edgeCenter);
                    edgesGraph.emplace_back(lastCenters[i], edgeCenter);
                }
            }

            // Update active edges
            if (event.isStart) {
                activeEdges.insert(event.edge);
            } else {
                activeEdges.erase(event.edge);
            }

            // Update last state
            lastCenters = centers;
            lastX = currentX;
        }

        path_interface::msg::Graph graph;
        vector<path_interface::msg::GraphEdge> edges;
        for (auto const &edge : edgesGraph) {
          path_interface::msg::GraphEdge e;
          path_interface::msg::GraphNode n1;
          path_interface::msg::GraphNode n2;
          n1.x = edge.first.x; n1.y = edge.first.y;
          n2.x = edge.second.x; n2.y = edge.second.y;

          e.start = n1;
          e.end = n2;

          edges.push_back(e);
          RCLCPP_INFO(this->get_logger(), "Next edge: ('%f', '%f'), ('%f', '%f')", n1.x, n1.y, n2.x, n2.y);
        }
        graph.edges = edges;
        response->graph = graph;
    }

    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        for (const auto &obs : msg->obstacles) {
            vector<geometry_msgs::msg::Point32> points = obs.polygon.points;
            for (unsigned i = 0; i < points.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "I heard an obstacle vertex: '%f', '%f", points[i].x, points[i].y);
                obstacle_vertices.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
            }
            
        }
      
    }

    void borders_callback(const geometry_msgs::msg::Polygon msg)
    {
        vector<geometry_msgs::msg::Point32> points = msg.points;
        for (unsigned i = 0; i < points.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f", points[i].x, points[i].y);
            obstacle_vertices.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
        }
            
      
    }
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;
    rclcpp::Service<path_interface::srv::GenerateGraph>::SharedPtr service_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RoadmapPublisher>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}