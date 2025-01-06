#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "RRTstar.h"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "path_interface/srv/move_robots.hpp"

bool complete = false;

struct Obstacle {
	bool is_cylinder;
	double x,y;
	double radius;
	double length_vertice;
	std::vector<std::pair<double,double>> vertices;	
};

struct Graph{
	std::vector<Obstacle> obstacles;
    std::vector<Point> borders;	
	double getWidth() const { return getWidthEnd - getWidthStart};
	double getHeight() const { return getHeightEnd - getHeightStart };

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

}

struct Point {
	float x, y;
	Point() = default;
	Point(float _x, float _y) : x(_x), y(_y) {}
	bool operator==(const Point &other) const
	{ return (x == other.x&& y == other.y);}
};


Graph graph;
geometry_msgs::msg::Pose start_pose;
std::vector<Point> goal;


class RRTStarPlanner : public rclcpp::Node {
	public:
		RRTStarPlanner() : Node("rrtstar_planner") {
			// subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			// 	"/shelfino1/amcl_pose", qos, bind(&AStarPlanner::position_callback, this, std::placeholders::_1));

		}

		// shelfino#/initialpose? 
		// !TODO: this topic needs to be modfified!
        // subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     "/amcl_pose", qos, std::bind(&RRTStarPlanner::position_callback, this, std::placeholders::_1));

		// Create a callback group
		auto amcl_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		// auto global_path_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

		rclcpp::SubscriptionOptions amcl_pose_options;
		amcl_pose_options.callback_group = amcl_pose_cb_group;

		// rclcpp::SubscriptionOptions global_path_options;
		// global_path_options.callback_group = global_path_cb_group;  

		if (robot_names.empty())
		{
			RCLCPP_WARN(this->get_logger(), "There are no robots in the parameter file!");
			return;
		}

		for (const auto &robot_name : robot_names)
		{
			std::string topic_name = "/" + robot_name + "/amcl_pose";


			RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", topic_name.c_str());
			
			auto callback = [this, robot_name](const PoseWithCovarianceStamped::SharedPtr msg) {
				this->position_callback(robot_name, msg);
			};

			robot_position_subscribers_.emplace_back(
				this->create_subscription<PoseWithCovarianceStamped>(
					topic_name, qos, callback, amcl_pose_options));
		}

		

		// obstacles and boders
        subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", qos, bind(&RRTStarPlanner::obstacles_callback, this, _1));

        subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
            "/map_borders", qos, bind(&RRTStarPlanner::borders_callback, this, _1));

        subscription_gates_= this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/gates", qos, bind(&RRTStarPlanner::gates_callback, this, _1));		
		

        void getPath() {		
			if (goal.empty()) {
				RCLCPP_WARN(this->get_logger(), "Goal is not set. Skipping path computation.");
				return;
			}
			
			// TODO define the input parameters
			
			// Initialize RRT* Planner
			double step_size = 0.1;
			double radius = 1.0;
			RRTstar rrtstar(&graph, step_size, radius);

			// TODO Compute the path array
			std::vector<id_t> path = rrtstar.findPath(
				start_pose.position.x, start_pose.position.y, goal[0].x, goal[0].y
			);

			// TODO call follow_path client with path_array
			if (!move_robots_client_->wait_for_service(std::chrono::seconds(5))) {
				RCLCPP_ERROR(this->get_logger(), "Cannot call move_robots service after waiting 5 seconds");
				return;
			}
			auto move_robots_request = std::make_shared<path_interface::srv::MoveRobots::Request>();
			move_robots_request->paths = path_array;
			move_robots_client_->async_send_request(move_robots_request);
			complete = true;			


		}
       

    private:
	  	std::vector<rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr> robot_position_subscribers_;
		//rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_robot_position_;
		rclcpp::Subscription<ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
  		rclcpp::Subscription<Polygon>::SharedPtr subscription_borders_;
		rclcpp::Subscription<Polygon>::SharedPtr subscription_gates_;	
		rclcpp::Client<path_interface::srv::MoveRobots>::SharedPtr move_robots_client_;



		// ROBOTS' NAME
		void position_callback(const std::string name, const PoseWithCovarianceStamped::SharedPtr msg)
		{
		this->shelfino_pose = msg->pose.pose;                             // store shelfino pose[x,y] in shelfino_pose
		this->robot_poses_.emplace_back(*msg);                            // an vector of shelfinos' poses
		RCLCPP_INFO(this->get_logger(), "Received shelfino pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
		}		

		// // TODO: STARTING POINT
		// void position_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
		// 	start_pose = msg->pose.pose;
		// }

		// OBSTACLES
		void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
		{
			graph.obstacles.clear();

			for (const auto &obs : msg->obstacles) {
				Obstacle obstacles_info;
				vector<geometry_msgs::msg::Point32> points = obs.polygon.points;

				// FOR CYLINDER
				if (obs.radius > 0.0) { 
					RCLCPP_INFO(this->get_logger(), "I heard a cylinder obstacle: '%f', '%f', '%f'", obs.polygon.points[0].x, obs.polygon.points[0].y, obs.radius);
					obstacles_info.is_cylinder = true;
					obstacles_info.x = obs.polygon.points[0].x;
					obstacles_info.y = obs.polygon.points[0].y;
					obstacles_info.radius = obs.radius;
 					graph.obstacles.push_back(obstacles_info);
				}

				else{
					obstacles_info.is_cylinder = false;
					
					for (unsigned i = 0; i < points.size(); ++i) {
						RCLCPP_INFO(this->get_logger(), "I heard an obstacle vertex: '%f', '%f'", points[i].x, points[i].y);
						//total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
						obstacles_info.vertices.emplace_back(points[i].x, points[i].y);
					}
					graph.obstacles.push_back(obstacles_info);					
				}

				
			}
		
		}

		// BORDERS VERTICES
		void borders_callback(const geometry_msgs::msg::Polygon msg)
		{
			graph.borders.clear();
			vector<geometry_msgs::msg::Point32> points = msg.points;
			for (unsigned i = 0; i < points.size(); ++i) {
				RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f'", points[i].x, points[i].y);
				//total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
				graph.borders.push_back(Point(points[i].x, points[i].y));
			}	
		}



		// GATE = GOAL
		void gates_callback(const geometry_msgs::msg::PoseArray msg) {
			//vector<geometry_msgs::msg::Pose> poses = msg.poses;
			for (const auto& p : msg-> poses) {
				RCLCPP_INFO(this->get_logger(), "I heard a gate vertex: '%f', '%f'", p.position.x, p.position.y);
				goal.push_back(Point(p.position.x, p.position.y));
			}
		}	
}

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