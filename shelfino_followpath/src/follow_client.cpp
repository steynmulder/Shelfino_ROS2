#include "shelfino_pathfollow/followclient.hpp"



follow_client::follow_client() : Node("follow_client")
{
  // Define the QoS
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

  // set Kmax
  this->Kmax_ = 1.0;

  // get the name of robot
  this->declare_parameter<std::vector<std::string>>("init_names", {});
  auto robot_names = this->get_parameter("init_names").as_string_array();

  // Create a callback group
  auto amcl_pose_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto global_path_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


  rclcpp::SubscriptionOptions amcl_pose_options;
  amcl_pose_options.callback_group = amcl_pose_cb_group;

  rclcpp::SubscriptionOptions global_path_options;
  global_path_options.callback_group = global_path_cb_group;  

  if (robot_names.empty())
  {
      RCLCPP_WARN(this->get_logger(), "There are no robots in the parameter file!");
      return;
  }

  for (const auto &robot_name : robot_names)
  {
      std::string topic_name = "/" + robot_name + "/amcl_pose";
      std::string path_topic_name = "/" + robot_name + "/global_path";


      RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", topic_name.c_str());
      
      //SUBSCRIPTION 1: Listen to robots' position /shelfino#/amcl_pose
      auto callback = [this, robot_name](const PoseWithCovarianceStamped::SharedPtr msg) {
          this->position_callback(robot_name, msg);
      };

      robot_position_subscribers_.emplace_back(
          this->create_subscription<PoseWithCovarianceStamped>(
              topic_name, qos, callback, amcl_pose_options));

      //SUBSCRIPTION 2: Listen to path topic published by A*/RRT* 
      auto path_callback = [this, robot_name](const Path::SharedPtr msg) {
          this->global_path_callback(robot_name, msg);
      };

      global_path_subscribers_.emplace_back(
          this->create_subscription<Path>(
              path_topic_name, qos, path_callback, global_path_options));
  }

 
  // SUBSCRIPTION 3: Listen to the /gate_pose+/obstacles+/borders topic
  this->gates_pose_sub_ = this->create_subscription<PoseArray>(
    "/gates", qos, std::bind(&follow_client::handle_gate_pose, this, std::placeholders::_1)
  );
  this->subscription_obstacles_ = this->create_subscription<ObstacleArrayMsg>(
    "/obstacles", qos, std::bind(&follow_client::obstacles_callback, this, std::placeholders::_1)
  );
  this->subscription_borders_ = this->create_subscription<Polygon>(
    "/map_borders", qos, std::bind(&follow_client::borders_callback, this, std::placeholders::_1)
  );

  // TODO1: define the msgs type
  // TODO2: define callback function
  // this->dubins_path_sub_ = this->create_subscription<Path>(
  //   "/globalpath", qos, std::bind(&follow_client::handle_dubins_path, this, std::placeholders::_1)
  // );

  // PUBLISH: to tell other shelfino robots that it is ready
  this->shelfino_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("ready", qos);
  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  this->shelfino_ready_pub_->publish(msg);

  // SERVICE: Create the start service
  this->start_service_ = this->create_service<Empty>(
    "start", std::bind(&follow_client::start_callback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // ACTION1: Send the path to action server: followpath [local planner]
  this->follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");


  RCLCPP_INFO(this->get_logger(), "follow path client node created.");
}


void follow_client::position_callback(const string name, const PoseWithCovarianceStamped::SharedPtr msg)
{
  this->shelfino_pose = msg->pose.pose;                             // store shelfino pose[x,y] in shelfino_pose
  this->robot_poses_.emplace_back(*msg);                            // an vector of shelfinos' poses
  RCLCPP_INFO(this->get_logger(), "Received shelfino pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
}


void follow_client::handle_gate_pose(const PoseArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "%ld gates pose received.", msg->poses.size());
  for (size_t i=0; i<msg->poses.size(); i++){
    double yaw = get_yaw_from_q(msg->poses[i].orientation);
    RCLCPP_INFO(this->get_logger(), "%ld pose: x=%f, y=%f, th=%f", i, msg->poses[i].position.x, msg->poses[i].position.y, yaw);
  }                                                                // get gates yaw[x,y,yaw]

  this->gate_pose_ready = true;
  RCLCPP_INFO(this->get_logger(), "Gate pose set to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  // Publish that the gate pose is ready
  auto msg_ready = std_msgs::msg::Bool();
  msg_ready.data = true;
  this->shelfino_ready_pub_->publish(msg_ready);       
}

void follow_client::global_path_callback(const Path::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received path generated by A*");
  this->global_paths_.emplace_back(*msg);                            // an vector of shelfino#' path
  this->global_path_received = true; 
}


void follow_client::start_callback(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "`start` service called.");
  if (!this->gate_pose_ready){
    RCLCPP_ERROR(this->get_logger(), "Gate pose not set.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Navigating to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  // generate dubins path
  for (size_t robot_idx = 0; robot_idx < global_paths_.size();++robot_idx){
    const auto &robot_name = robot_names[robot_idx]
    const auto &global_path = global_paths_[robot_idx];
    if(!global_path.empty()){
      const auto &poses = this->global_path.poses;       
      if (poses.size()<2){
        RCLCPP_INFO(this-get_logger(),"Global Path has less than 2 points");
      }

      Path dubins_path;
      dubins_path.header = global_path.header;

      // iterate all points in the global path
      for (size_t i = 0; i < poses.size() -1; ++i){
        const auto &start_pose = poses[i].pose;
        const auto &goal_pose  = pose[i+1].pose;

        double x0 = start_pose.position.x;
        double y0 = start_pose.position.y;
        double th0 = get_yaw_from_q(start_pose.orientation);

        double xf = start_pose.position.x;
        double yf = start_pose.position.y;
        double thf = get_yaw_from_q(start_pose.orientation);    

        // call dubinShortestPath function in dubins.cpp
        DubinsCurve curve = Dubins::dubinsShortestPath(x0,y0,th0,xf,yf,thf,Kmax_);

        // Discrete dubins curve into path points
        constexpr int num_samples = 50;                  // sample numbers
        for (int j = 0; j< num_samples; ++j){
          double s = curve.L * j/ num_samples;
          double x, y, th;
          if (s <= curve.a1.L){
            std::tie(x,y,th) = computerArcPOint(s,curve.a1);
          }
          else if( s <= curve.a1.L + curve.a2.L){
            std::tie(x,y,th) = computerArcPOint(s - curve.a1.L,curve.a2);
          }
          else{
            std::tie(x,y,th) = computerArcPOint(s - curve.a1.L,curve.a3);        
          }

          // Add points into path
          geometry_msgs::msg:PoseStamped pose_stamped;
          pose_stamped.header = global_path.header;
          pose_stamped.pose.position.x = x;
          pose_stamped.pose.position.y = y;
          pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,sin(th/2),cos(th/2)));
          dubins_path.poses.push_back(pose_stamped);
        } // END: discretion of one arc dubins path
          
      }// END: iteration global path

      //call move function to activate the action server: follow path
      this->move(robot_name->robot_name,dubins_path->path);

    }

  
  }

}

void follow_client::move(const std::string &robot_name,const Path& path)
{
  // find if there already exists a shelfino#/followpath action server
  // if not create one 
  if(follow_path_clients_array.find(robot_name) == follow_path_clients_array.end()){
    follow_path_clients_array[robot_name] = rclcpp_action::create_client<FollowPath>(
      this, "/"+ robot_name + "/follow_path")
  }

  auto client = follow_path_clients_array[robot_name];

  RCLCPP_INFO(client->get_logger(), "Moving to gate");
  if(!client->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(this->get_logger(), "Action server for %s not available after waiting.",robot_name.c_str());
    exit(1);
  }  



  auto action_msg = FollowPath::Goal();
  action_msg.path = path;

  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath action server.");

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this, robot_name](const auto &goal_handle){
    if(!goal_handle){
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the FollowPath server.");
      exit(1);   
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted by the FollowPath server.");

  };


  send_goal_options.feedback_callback = [this, robot_name](auto, const auto &feedback){
    RCLCPP_INFO(this->get_logger(), "Received feedback from FollowPath");
  };


  send_goal_options.result_callback = [this, robot_name](const auto &result){
    switch (result.code){
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Path followed, reached gate.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted from FollowPath");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled by FollowPath");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "[FollowPath] Unknown result code");
        exit(1);
        break;
    }
  };

  client->async_send_goal(action_msg, send_goal_options);       // send msgs to action server: followpath 
}

// void follow_client::goal_response_path_following_callback(
//   const ClientFollowPathGoalHandle::SharedPtr & goal_handle)
// {
//   if (!goal_handle){
//     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the FollowPath server.");
//     exit(1);
//   }
//   RCLCPP_INFO(this->get_logger(), "Goal accepted by the FollowPath server.");
// }

// void follow_client::feedback_path_following_callback(
//   ClientFollowPathGoalHandle::SharedPtr,
//   const std::shared_ptr<const FollowPath::Feedback> feedback)
// {
//   RCLCPP_INFO(this->get_logger(), "Received feedback from FollowPath");
// }

// void follow_client::result_path_following_callback(
//   const ClientFollowPathGoalHandle::WrappedResult & result)
// {
//   switch (result.code){
//     case rclcpp_action::ResultCode::SUCCEEDED:
//       RCLCPP_INFO(this->get_logger(), "Path followed, reached gate.");
//       break;
//     case rclcpp_action::ResultCode::ABORTED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was aborted from FollowPath");
//       break;
//     case rclcpp_action::ResultCode::CANCELED:
//       RCLCPP_ERROR(this->get_logger(), "Goal was canceled by FollowPath");
//       break;
//     default:
//       RCLCPP_ERROR(this->get_logger(), "[FollowPath] Unknown result code");
//       exit(1);
//       break;
//   }
// }


int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("follow_client"), "Starting follow path client node.");
  srand(time(NULL));
  rclcpp::init(argc, argv);                                       // initializes ROS2 C++ client library
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<follow_client>();                  // create a node named "follow_client"
  executor.add_node(node);
  executor.spin();
  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
