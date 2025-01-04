#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "path_interface/srv/generate_graph.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "RRTstar.h"

bool complete = false;

class RRTStarPlanner : public rclcpp::Node {
	public:
		RRTStarPlanner() : Node("rrtstar_planner") {
			// subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			// 	"/shelfino1/amcl_pose", qos, bind(&AStarPlanner::position_callback, this, std::placeholders::_1));
			

			// call "generate_graph" server to get graph
			// client_ = this->create_client<path_interface::srv::GenerateGraph>("generate_graph");


			get_obstacles = 
		}



		// obstacles and boders
		this->subscription_obstacles_ = this->create_subscription<ObstacleArrayMsg>(
			"/obstacles", qos, std::bind(&follow_client::obstacles_callback, this, std::placeholders::_1, std::placeholders::_2)
		);
		this->subscription_borders_ = this->create_subscription<Polygon>(
			"/map_borders", qos, std::bind(&follow_client::borders_callback, this, std::placeholders::_1)
		);
		
		
		
        // for graph construction
		std::map<id_t, GVertex> vertices;


        void getPath() {		


			
			// TODO define the input parameters
			RRTstar rrtstar(&graph, step_sizde, radius);



		}

        void generate_graph_response_callback(rclcpp::Client<path_interface::srv::GenerateGraph>::SharedFuture future) {
            this->graph_response_ = future.get();
        }



		// void position_callback(const geometry_msgs::msg::PoseWithCovarianceStamped msg) {
		// 	RCLCPP_INFO(this->get_logger(), "I AM HERE!!!");
		// 	getGraph(msg.pose.pose.position.x, msg.pose.pose.position.y);
		// }

        

    private:
		// rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_robot_position_;
		rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
  		rclcpp::Subscription<Polygon>::SharedPtr subscription_borders_;	
        rclcpp::Client<path_interface::srv::GenerateGraph>::SharedPtr client_;
        path_interface::srv::GenerateGraph::Response::SharedPtr graph_response_;


		void obstacles_callback(const geometry_msgs::msg::Polygon msg)
		{
			vector<geometry_msgs::msg::Point32> points = msg.points;
			for (unsigned i = 0; i < points.size(); ++i) {
				RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f", points[i].x, points[i].y);
				total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
				wall_vertices.push_back(Point(points[i].x, points[i].y));
			}
				
		
		}

		void borders_callback(const geometry_msgs::msg::PoseArray msg) {
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
    node->getGraph();

	if (!complete) {
		rclcpp::spin(node);
	}
    
    rclcpp::shutdown();
    return 0;
}