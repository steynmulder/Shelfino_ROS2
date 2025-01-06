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
#include "nav_msgs/msg/path.hpp"
#include "path_interface/srv/move_robots.hpp"

bool complete = false;




std::vector<RRTstar::Point> start_point;
std::vector<RRTstar::Point> goal;
std::map<std::string, geometry_msgs::msg::PoseWithCovarianceStamped> robot_poses_;
std::map<std::string, std::vector<id_t>> robot_paths;


class RRTStarPlanner : public rclcpp::Node {
	public:
		RRTStarPlanner() : Node("rrtstar_planner") {
			// subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			// 	"/shelfino1/amcl_pose", qos, bind(&AStarPlanner::position_callback, this, std::placeholders::_1));

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

			this->declare_parameter<std::vector<std::string>>("init_names", {});
  			auto robot_names = this->get_parameter("init_names").as_string_array();
			auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);


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

			

			// obstacles and boders
			subscription_obstacles_ = this->create_subscription<obstacles_msgs::msg::ObstacleArrayMsg>(
				"/obstacles", qos, bind(&RRTStarPlanner::obstacles_callback, this, std::placeholders::_1));

			subscription_borders_ = this->create_subscription<geometry_msgs::msg::Polygon>(
				"/map_borders", qos, bind(&RRTStarPlanner::borders_callback, this, std::placeholders::_1));

			subscription_gates_= this->create_subscription<geometry_msgs::msg::PoseArray>(
				"/gates", qos, bind(&RRTStarPlanner::gates_callback, this, std::placeholders::_1));

			// move_robot_client
			move_robots_client_ = this->create_client<path_interface::srv::MoveRobots>("start");

			}

		// shelfino#/initialpose? 
		// !TODO: this topic needs to be modfified!
        // subscription_robot_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //     "/amcl_pose", qos, std::bind(&RRTStarPlanner::position_callback, this, std::placeholders::_1));

		// Create a callback group
		// auto amcl_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		// auto global_path_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

		// rclcpp::SubscriptionOptions amcl_pose_options;
		// amcl_pose_options.callback_group = amcl_pose_cb_group;

		// rclcpp::SubscriptionOptions global_path_options;
		// global_path_options.callback_group = global_path_cb_group;  

		std::map<std::string, geometry_msgs::msg::PoseWithCovarianceStamped> robot_poses_;
		std::map<std::string, std::vector<RRTstar::Point>> robot_paths;	
		RRTstar::Graph graphmap;	
		

        void getPath() {		
			if (goal.empty()) {
				RCLCPP_WARN(this->get_logger(), "Goal is not set. Skipping path computation.");
				return;
			}


			// Initialize RRT* Planner
			double step_size = 0.1;
			double radius = 1.0;
			RRTstar rrtstar(&graphmap, step_size, radius);


			// FIND THE PATHs FOR 3 ROBOTS
			for (const auto& pair : robot_poses_){
				std::string robot_name = pair.first;
				const auto& pose_msg = pair.second;
				double start_x = pose_msg.pose.pose.position.x;
				double start_y = pose_msg.pose.pose.position.y;
				double goal_x = goal[0].x;
				double goal_y = goal[0].y;

				// find path using RRT*
				std::vector<RRTstar::Point> path = rrtstar.findPath(start_x, start_y, goal_x, goal_y);
				robot_paths[robot_name] = path;
			}

			path_interface::msg::PathArray path_array;
			
			for (auto it = robot_paths.begin(); it != robot_paths.end(); ++it) {
				nav_msgs::msg::Path path_msg;

				for (auto id : it->first) {
					for ( RRTstar::Point n : it ->second ){
						geometry_msgs::msg::PoseStamped pose;
						pose.pose.position.x = n.x;
						pose.pose.position.y = n.y;
						path_msg.poses.push_back(pose);
						RCLCPP_INFO(this->get_logger(), "path id: %d, x: %f, y: %f", id, n.x, n.y);
					}
				}
				RCLCPP_INFO(this->get_logger(), "path size: %zu", it->second.size());
				path_array.paths.push_back(path_msg);
				path_array.names.push_back(it->first);
			}

			// call follow_path client with path_array
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
	  	std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> robot_position_subscribers_;
		//rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_robot_position_;
		rclcpp::Subscription<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr subscription_obstacles_;
  		rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr subscription_borders_;
		rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_gates_;	
		rclcpp::Client<path_interface::srv::MoveRobots>::SharedPtr move_robots_client_;

		

		// STARTING POINTS 
		void position_callback(const std::string name, const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
		{
			this->robot_poses_[name] = *msg;                            // an vector of shelfinos' poses
			RCLCPP_INFO(this->get_logger(), "Received shelfino pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
		}		


		// OBSTACLES
		void obstacles_callback(const obstacles_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
		{
			graphmap.obstacles.clear();

			for (const auto &obs : msg->obstacles) {
				RRTstar::Obstacle obstacles_info;
				std::vector<geometry_msgs::msg::Point32> points = obs.polygon.points;

				// FOR CYLINDER
				if (obs.radius > 0.0) { 
					RCLCPP_INFO(this->get_logger(), "I heard a cylinder obstacle: '%f', '%f', '%f'", obs.polygon.points[0].x, obs.polygon.points[0].y, obs.radius);
					obstacles_info.is_cylinder = true;
					obstacles_info.x = obs.polygon.points[0].x;
					obstacles_info.y = obs.polygon.points[0].y;
					obstacles_info.radius = obs.radius;
 					graphmap.obstacles.push_back(obstacles_info);
				}

				else{
					obstacles_info.is_cylinder = false;
					
					for (unsigned i = 0; i < points.size(); ++i) {
						RCLCPP_INFO(this->get_logger(), "I heard an obstacle vertex: '%f', '%f'", points[i].x, points[i].y);
						//total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
						obstacles_info.vertices.emplace_back(points[i].x, points[i].y);
					}
					graphmap.obstacles.push_back(obstacles_info);					
				}

				
			}
		
		}

		// BORDERS VERTICES
		void borders_callback(const geometry_msgs::msg::Polygon msg)
		{
			graphmap.borders.clear();
			std::vector<geometry_msgs::msg::Point32> points = msg.points;
			for (unsigned i = 0; i < points.size(); ++i) {
				RCLCPP_INFO(this->get_logger(), "I heard a border vertex: '%f', '%f'", points[i].x, points[i].y);
				//total_edges.push_back(Edge(Point(points[i].x, points[i].y), Point(points[(i+1)%points.size()].x, points[(i+1)%points.size()].y)));
				graphmap.borders.push_back(RRTstar::Point(points[i].x, points[i].y));
			}	
		}



		// GATE = GOAL
		// void gates_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
		// 	for (const auto& p : msg->poses) {
		// 		RCLCPP_INFO(this->get_logger(), "I heard a gate vertex: '%f', '%f'", p.position.x, p.position.y);
		// 		goal.push_back(RRTstar::Point(p.position.x, p.position.y));
		// 	}
		// }

		void gates_callback(const geometry_msgs::msg::PoseArray msg) {
        std::vector<geometry_msgs::msg::Pose> poses = msg.poses;
        for (geometry_msgs::msg::Pose& p : poses) {
            RCLCPP_INFO(this->get_logger(), "I heard a gate vertex: '%f', '%f", p.position.x, p.position.y);
            goal.push_back(RRTstar::Point(p.position.x, p.position.y));
        }
    }	

                                                         // get gates yaw[x,y,yaw]
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