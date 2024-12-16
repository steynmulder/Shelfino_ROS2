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
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "path_interface/srv/generate_graph.hpp"

#include "GVertex.h"
#include "GEdge.h"

using std::placeholders::_1;
using namespace std;

int numberRobots;

class RoadmapServer : public rclcpp::Node
{
  public:
    RoadmapServer()
    : Node("send_roadmap")
    {
        static const rmw_qos_profile_t rmw_qos_profile_custom =
        {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            100,
            RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
        };
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);
        subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos, bind(&RoadmapServer::obstacles_callback, this, _1));
        subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "/map_borders", qos, bind(&RoadmapServer::borders_callback, this, _1));
        subscription_gates_= this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gates", qos, bind(&RoadmapServer::gates_callback, this, _1));
        service_ = this->create_service<path_interface::srv::GenerateGraph>(
            "generate_graph", std::bind(&RoadmapServer::generateGraph, this, std::placeholders::_1, std::placeholders::_2));


        this->declare_parameter<std::vector<std::string>>("init_names", {});
        auto robot_names = this->get_parameter("init_names").as_string_array();

        if (robot_names.empty())
        {
            RCLCPP_WARN(this->get_logger(), "There are no robots in the parameter file!");
            return;
        }


        for (const auto &robot_name : robot_names)
        {
            std::string topic_name = "/" + robot_name + "/amcl_pose";

            RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", topic_name.c_str());

            auto callback = [this, robot_name](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                this->position_callback(robot_name, msg);
            };


            robot_position_subscribers_.emplace_back(
                this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    topic_name, qos, callback));
        }
    }

  private:

    typedef enum {FREE, OCCUPIED, MIXED, OUTSIDE} STATUS;

    const float min_length = 1.0;

    id_t id = 0;

    struct Point {
        float x, y;
        Point() = default;
        Point(float _x, float _y) : x(_x), y(_y) {}
        bool operator==(const Point &other) const
        { return (x == other.x
                    && y == other.y);
        }
    };

    // struct PointHasher {
    // std::size_t operator()(const Point& k) const {
    //         using std::size_t;
    //         using std::hash;

    //         return ((hash<float>()(k.x)
    //                 ^ (hash<float>()(k.y) << 1)) >> 1);
    //     }
    // };

    struct Edge {
        Point p1, p2;
        Edge(Point _p1, Point _p2) : p1(_p1), p2(_p2) {}
    };

    struct QuadNode {
        float x, y, size;
        STATUS status;
        vector<QuadNode> children;
        id_t id;

        // QuadNode() = default;
        QuadNode(float _x, float _y, float _size, STATUS _status) : x(_x), y(_y), size(_size), status(_status) {}

        bool operator!=(const QuadNode &other) const {
            return (x != other.x || y != other.y);
        }
    };

    vector<Edge> total_edges;
    vector<Point> wall_vertices;
    vector<vector<Point>> obstacle_vertices;
    vector<pair<float, Point>> cylinder_obstacles; // radius, center
    vector<Point> gates;
    unordered_map<id_t, GVertex> vertices;
    // unordered_map<Point, QuadNode, PointHasher> point_map;
    vector<QuadNode> quads;
    unordered_map<string, Point> robot_positions;
            vector<path_interface::msg::GraphNode> nodes; // for response message

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
        //TODO add cylinder obstacles
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

    bool pointInRadius(Point& p, float radius, Point& center) {
        return sqrt(pow(p.x - center.x, 2) + pow(p.y - center.y, 2)) <= radius;
    }

    bool nodeInRadius(QuadNode& node, float radius, Point& center) {
        Point tl = {node.x, node.y};
        Point tr = {node.x + node.size, node.y};
        Point bl = {node.x, node.y + node.size};
        Point br = {node.x + node.size, node.y + node.size};
        if (!pointInRadius(tl, radius, center) ||
            !pointInRadius(tr, radius, center) ||
            !pointInRadius(bl, radius, center) ||
            !pointInRadius(br, radius, center)
        ) {
            return false;
        }

        return true;
    }

    bool nodeIntersectRadius(QuadNode& node, float radius, Point& center) {
        Point tl = {node.x, node.y};
        Point tr = {node.x + node.size, node.y};
        Point bl = {node.x, node.y + node.size};
        Point br = {node.x + node.size, node.y + node.size};
        if (pointInRadius(tl, radius, center) ||
            pointInRadius(tr, radius, center) ||
            pointInRadius(bl, radius, center) ||
            pointInRadius(br, radius, center)
        ) {
            return true;
        }

        return false;
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

        for (pair<float, Point> cyl : cylinder_obstacles) { // node completely in cylinder
            if (nodeInRadius(node, cyl.first, cyl.second)) {
                return OCCUPIED;
            }
        }

        for (Edge& edge : total_edges) {
            if (edgeIntersectsNode(node, edge)) {
                return MIXED;
            }
        }

        for (pair<float, Point> cyl : cylinder_obstacles) { // because we did a check for complete intersection before, this must be partial
            if (nodeIntersectRadius(node, cyl.first, cyl.second)) {
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

    vector<QuadNode> getNeighbors(QuadNode& node) {
        vector<QuadNode> result;
        for (QuadNode& n : quads) {
            if (node != n) {
                // north
                if ((n.y + n.size) == node.y &&
                    (n.x + n.size) > node.x &&
                    n.x < (node.x + node.size)) {
                        result.push_back(n);
                        continue;
                }

                // east
                if (n.x == (node.x + node.size) &&
                    (n.y + n.size) > node.y &&
                    n.y < (node.y + node.size)) {
                        result.push_back(n);
                        continue;
                }

                // south
                if (n.y == (node.y + node.size) &&
                    (n.x + n.size) > node.x &&
                    n.x < (node.x + node.size)) {
                        result.push_back(n);
                        continue;
                }

                // west
                if ((n.x + n.size) == node.x &&
                    (n.y + n.size) > node.y &&
                    n.y < (node.y + node.size)) {
                        result.push_back(n);
                        continue;
                }


            }
        }

        return result;
    }

    void buildVertices(QuadNode& node) {

        if (node.status == FREE) {
            GVertex v = {id, "", node.x + node.size / 2, node.y + node.size / 2, 0.0};
            node.id = id;
            vertices[id++] = v;
            // Point p = {node.x, node.y};
            // point_map[p] = node;
            quads.push_back(node);
            return;
        }
         
        if (!node.children.empty()) {
            for (QuadNode& n : node.children) {
                buildVertices(n);
            }
        }
    }

    void buildEdges(QuadNode& node) {
        if (node.status == FREE) {
            // do neighbor search and edge construciton
            vector<QuadNode> neighbors = getNeighbors(node);
            for (QuadNode& neighbor : neighbors) {
                vertices[node.id].addEdge(neighbor.id, euclidianDistance(node, neighbor), "");
            }
            return;
        }

        if (!node.children.empty()) {
            for (QuadNode& n : node.children) {
                buildEdges(n);
            }
        }
    }

    id_t getClosestVertex(Point& p) {
        GVertex closest;
        float closestDistance = INFINITY;
        for (QuadNode node : quads) {
            if (node.status == FREE) {
                if (closestDistance == INFINITY) {
                    closest = vertices[node.id];
                    closestDistance = sqrt(pow(p.x - node.x, 2) + pow(p.y - node.y, 2));
                    continue;
                }

                GVertex vert = vertices[node.id];
                float dist = sqrt(pow(p.x - node.x, 2) + pow(p.y - node.y, 2));
                if (closestDistance > dist) {
                    closest = vert;
                    closestDistance = dist;
                }
            }
            
        }

        return closest.getStateID();
    }
    
    void generateGraph(const std::shared_ptr<path_interface::srv::GenerateGraph::Request> request, 
        std::shared_ptr<path_interface::srv::GenerateGraph::Response>      response) {

        if (vertices.empty()) { // only generate the graph with the first call
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

            buildVertices(boundary);
            buildEdges(boundary);

            for (auto v = vertices.begin(); v != vertices.end();) {
                if (v->second.getEdgeList().empty()) {
                    v = vertices.erase(v);
                } else {
                    path_interface::msg::GraphNode n;
                    n.id = v->first;
                    n.x = (float) v->second.getx();
                    n.y = (float) v->second.gety();

                    for (GEdge& e : v->second.getEdgeList()) {

                        n.edges.push_back(e.getDestVID());
                    }

                    RCLCPP_INFO(this->get_logger(), "We have %u, %f, %f, with %zu vertices", n.id, n.x, n.y, n.edges.size());
                    
                    nodes.push_back(n);

                    ++v;
                }
            }

            RCLCPP_INFO(this->get_logger(), "We have %zu vertices", vertices.size());
        }

        
        // RCLCPP_INFO(this->get_logger(), "We have %zu points in the map", point_map.size());

        response->vertices = nodes;

        Point gate = {gates[0].x, gates[0].y};
        response->gate_id = getClosestVertex(gate);
        for (auto it = robot_positions.begin(); it != robot_positions.end(); ++it) {
            Point start = {(float)it->second.x, (float)it->second.y};
            response->start_ids.push_back(getClosestVertex(start));
            response->names.push_back(it->first);
        }
        
        
    }

    void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "HERE");
        for (const auto &obs : msg->obstacles) {
            vector<geometry_msgs::msg::Point32> points = obs.polygon.points;
            vector<Point> polygon;

            if (obs.radius > 0.0) { // cylinder
                cylinder_obstacles.push_back(make_pair(obs.radius, Point(obs.polygon.points[0].x, obs.polygon.points[0].y)));
                continue;
            }

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

    void gates_callback(const geometry_msgs::msg::PoseArray msg) {
        vector<geometry_msgs::msg::Pose> poses = msg.poses;
        for (geometry_msgs::msg::Pose& p : poses) {
            RCLCPP_INFO(this->get_logger(), "I heard a gate vertex: '%f', '%f", p.position.x, p.position.y);
            gates.push_back(Point(p.position.x, p.position.y));
        }
    }

    void position_callback(const string name, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        Point p = {(float) msg->pose.pose.position.x, (float) msg->pose.pose.position.y};
        robot_positions[name] = p;
        RCLCPP_INFO(this->get_logger(), "I heard a robot: %s. It has position(%f, %f)", name.c_str(), (float)p.x, p.y);
    }
    
    rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> robot_position_subscribers_;
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