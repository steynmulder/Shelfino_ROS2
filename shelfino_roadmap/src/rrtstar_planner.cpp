#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "RRTstar.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point.hpp"

bool complete = false;

struct Obstacle {
	bool is_cylinder;
	bool is_box;
	double x,y;
	double radius;
	double length_x, length_y;
};

struct Graph{
	std::vector<Node*> nodes;
	std::vector<Obstacle> obstacles;
    std::vector<geometry_msgs::msg::Point> borders;	
	double width, height;
}

class RRTStarPlanner : public rclcpp::Node {
	public:
		RRTStarPlanner() : Node("rrtstar_planner") {
			// subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			// 	"/shelfino1/amcl_pose", qos, bind(&AStarPlanner::position_callback, this, std::placeholders::_1));
			

			// call "generate_graph" server to get graph
			// client_ = this->create_client<path_interface::srv::GenerateGraph>("generate_graph");

		}


        subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", qos, std::bind(&RRTStarPlanner::position_callback, this, std::placeholders::_1));

		// obstacles and boders
        subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos, bind(&RRTStarPlanner::obstacles_callback, this, _1));

        subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "/map_borders", qos, bind(&RRTStarPlanner::borders_callback, this, _1));

        subscription_gates_= this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gates", qos, bind(&RRTStarPlanner::gates_callback, this, _1));		
		

        void getPath() {		


			
			// TODO define the input parameters
			
			// Initialize RRT* Planner
			double step_size = 0.1;
			double radius = 1.0;
			RRTstar rrtstar(&graph, step_sizde, radius);

			// Compute the path
			std::vector<id_t> path = rrtstar.findPath(
				start_pose.position.x, start_pose.position.y, goal.x, goal.y
			);

			// TODO: Call follow path client


		}


		// void position_callback(const geometry_msgs::msg::PoseWithCovarianceStamped msg) {
		// 	RCLCPP_INFO(this->get_logger(), "I AM HERE!!!");
		// 	getGraph(msg.pose.pose.position.x, msg.pose.pose.position.y);
		// }

        

    private:
		rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_robot_position_;
		rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
  		rclcpp::Subscription<Polygon>::SharedPtr subscription_borders_;
		rclcpp::Subscription<Polygon>::SharedPtr subscription_gates_;	
        rclcpp::Client<path_interface::srv::GenerateGraph>::SharedPtr client_;


		Graph& graph;
	    geometry_msgs::msg::Pose start_pose;
		geometry_msgs::msg::Point goal;


		// STARTING POINT
		void position_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
			start_pose = msg->pose.pose;
		}


		void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
		{
			// graph.obstacles.clear();

			for (const auto &obs : msg->obstacles) {
				vector<geometry_msgs::msg::Point32> points = obs.polygon.points;
				vector<Point> polygon;

				for (const auto& obs: points){
					Obstacle obstacle;
					obstacle.x = points.x;
					obstacle.y = points.y;
					if (obs.radius > 0.0){
						obstacle.is_cylinder = true;
						obstacle.radius = obs.radius;
						graph.obstacles.push_back(obstacle);
					}
					else {
						obstacle.is_box = true;
						obstacle.length_x = ;
						obstacle.length_y = ;
					}
				}

				// if (obs.radius > 0.0) { // cylinder
				// 	is_cylinder = true;
				// 	cylinder_obstacles.push_back(make_pair(obs.radius, Point(obs.polygon.points[0].x, obs.polygon.points[0].y)));
				// 	continue;
				// }

				// for (unsigned i = 0; i < points.size(); ++i) {
				// 	RCLCPP_INFO(this->get_logger(), "I heard an obstacle vertex: '%f', '%f", points[i].x, points[i].y);
				// 	total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
				// 	polygon.emplace_back(Point(points[i].x, points[i].y));
				// }

				// obstacle_vertices.emplace_back(polygon);
				
			}
		
		}


		//
		void borders_callback(const geometry_msgs::msg::Polygon msg)
		{
			vector<geometry_msgs::msg::Point32> points = msg.points;
			for (unsigned i = 0; i < points.size(); ++i) {
				RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f", points[i].x, points[i].y);
				// total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
				// wall_vertices.push_back(Point(points[i].x, points[i].y));
			}	
		
		}

		// GOAL
		void gates_callback(const geometry_msgs::msg::PoseArray msg) {
			vector<geometry_msgs::msg::Pose> poses = msg.poses;
			for (geometry_msgs::msg::Pose& p : poses) {
				RCLCPP_INFO(this->get_logger(), "I heard a gate vertex: '%f', '%f", p.position.x, p.position.y);
				gates.push_back(Point(p.position.x, p.position.y));
			}
		}
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);


	auto node = std::make_shared<RRTStarPlanner>();
    node->getPath();

	if (!complete) {
		rclcpp::spin(node);
	}
    
    rclcpp::shutdown();
    return 0;
}