#include "shelfino_followpath/follow_client.hpp"
#include "shelfino_followpath/Dubins.h"


follow_client::follow_client() : Node("follow_client")
{
  // Define the QoS
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

  // set Kmax
  this->Kmax_ = 0.1;

  // get the name of robot
  this->declare_parameter<std::vector<std::string>>("init_names", {});
  auto robot_names = this->get_parameter("init_names").as_string_array();

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
      // std::string path_topic_name = "/" + robot_name + "/global_path";


      RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", topic_name.c_str());
      
      //SUBSCRIPTION 1: Listen to robots' position /shelfino#/amcl_pose
      auto callback = [this, robot_name](const PoseWithCovarianceStamped::SharedPtr msg) {
          this->position_callback(robot_name, msg);
      };

      robot_position_subscribers_.emplace_back(
          this->create_subscription<PoseWithCovarianceStamped>(
              topic_name, qos, callback, amcl_pose_options));

      //SUBSCRIPTION 2: Listen to path topic published by A*/RRT* 
      // auto path_callback = [this, robot_name](const Path::SharedPtr msg) {
      //     this->global_path_callback(robot_name, msg);
      // };

      // global_path_subscribers_.emplace_back(
      //     this->create_subscription<Path>(
      //         path_topic_name, qos, path_callback, global_path_options));
  }

 
  // SUBSCRIPTION 3: Listen to the /gate_pose+/obstacles+/borders topic
  this->gates_pose_sub_ = this->create_subscription<PoseArray>(
    "/gates", qos, std::bind(&follow_client::handle_gate_pose, this, std::placeholders::_1)
  );

  // TODO use these
  // this->subscription_obstacles_ = this->create_subscription<ObstacleArrayMsg>(
  //   "/obstacles", qos, std::bind(&follow_client::obstacles_callback, this, std::placeholders::_1, std::placeholders::_2)
  // );
  // this->subscription_borders_ = this->create_subscription<Polygon>(
  //   "/map_borders", qos, std::bind(&follow_client::borders_callback, this, std::placeholders::_1)
  // );


  // this->subscription_paths_ = this->create_subscription<PathArray>(
  //   "/global_paths", qos, std::bind(&follow_client::paths_callback, this, std::placeholders::_1)
  // );

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
  this->start_service_ = this->create_service<MoveRobots>(
    "start", std::bind(&follow_client::start_callback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // ACTION1: Send the path to action server: followpath [local planner]
  // this->follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

  this->compute_path_to_pose_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/shelfino1/compute_path_to_pose");
  this->follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino1/follow_path");


  RCLCPP_INFO(this->get_logger(), "follow path client node created.");
}


void follow_client::position_callback(const std::string name, const PoseWithCovarianceStamped::SharedPtr msg)
{
  this->shelfino_pose = msg->pose.pose;                             // store shelfino pose[x,y] in shelfino_pose
  this->robot_poses_[name] = *msg;                            // an vector of shelfinos' poses
  RCLCPP_INFO(this->get_logger(), "Received shelfino pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

// void follow_client::paths_callback(const PathArray::SharedPtr msg) {
//   this->global_paths_ = msg;
// }


void follow_client::handle_gate_pose(const PoseArray::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "%ld gates pose received.", msg->poses.size());
  for (size_t i=0; i<msg->poses.size(); i++){
    double yaw = get_yaw_from_q(msg->poses[i].orientation);
    RCLCPP_INFO(this->get_logger(), "%ld pose: x=%f, y=%f, th=%f", i, msg->poses[i].position.x, msg->poses[i].position.y, yaw);
    this->goal_yaw = yaw;
  }                                                                // get gates yaw[x,y,yaw]

  this->gate_pose_ready = true;
  RCLCPP_INFO(this->get_logger(), "Gate pose set to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  // Publish that the gate pose is ready
  auto msg_ready = std_msgs::msg::Bool();
  msg_ready.data = true;
  this->shelfino_ready_pub_->publish(msg_ready);       
}

// void follow_client::global_path_callback(const Path::SharedPtr msg)
// {
//   RCLCPP_INFO(this->get_logger(), "Received path generated by A*");
//   this->global_paths_.emplace_back(*msg);                            // an vector of shelfino#' path
//   this->global_path_received = true; 
// }


void follow_client::start_callback(const std::shared_ptr<MoveRobots::Request> request,
            std::shared_ptr<MoveRobots::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "`start` service called.");
  if (!this->gate_pose_ready){
    RCLCPP_ERROR(this->get_logger(), "Gate pose not set.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Navigating to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  // generate dubins path
  for (size_t robot_idx = 0; robot_idx < request->paths.paths.size();++robot_idx){
    RCLCPP_INFO(this->get_logger(), "path size: %zu", request->paths.paths[robot_idx].poses.size());
    const std::string &robot_name = request->paths.names[robot_idx];
    const auto &global_path = request->paths.paths[robot_idx];
    RCLCPP_INFO(this->get_logger(), "here1");
    if(!global_path.poses.empty()){
      const auto &poses = global_path.poses;
      RCLCPP_INFO(this->get_logger(), "here2");

      if (poses.size()<2){
        RCLCPP_INFO(this->get_logger(),"Global Path has less than 2 points");
      }

      RCLCPP_INFO(this->get_logger(), "here3");

      float th_start = get_yaw_from_q(this->robot_poses_[robot_name].pose.pose.orientation);
      float th_goal = this->goal_yaw;
      int k = 360;

      // RCLCPP_INFO(this->get_logger(),"robot yaw: %f, robot z: %f, robot w: %f", th_start, this->robot_poses_[robot_name].pose.pose.orientation.z, this->robot_poses_[robot_name].pose.pose.orientation.w);


      std::map<int, float> thetas;
      std::map<int, DubinsCurve> curves;
      thetas[0] = th_start;
      thetas[poses.size() -1] = th_goal;

      for (unsigned j = poses.size() - 2; j > 0; j--) {

        float best_length = INFINITY;
        float best_err = INFINITY;
        int best_k = -1;
        RCLCPP_INFO(this->get_logger(), "j: %u", j);

        for (int alpha = 0; alpha < k; ++alpha) {
          // RCLCPP_INFO(this->get_logger(), "x: %f", poses[j].pose.position.x);
          // RCLCPP_INFO(this->get_logger(), "y: %f", poses[j].pose.position.y);
          // RCLCPP_INFO(this->get_logger(), "2pialpha: %f", (M_2_PI * alpha / k));
          // RCLCPP_INFO(this->get_logger(), "x j+1: %f", poses[j+1].pose.position.x);
          // RCLCPP_INFO(this->get_logger(), "y j+1: %f", poses[j+1].pose.position.y);
          // RCLCPP_INFO(this->get_logger(), "thetaj+1: %f", thetas[j+1]);
          // RCLCPP_INFO(this->get_logger(), "kmax: %f", Kmax_);

          DubinsCurve curve = Dubins::dubinsShortestPath(poses[j].pose.position.x,poses[j].pose.position.y, (M_2_PI * alpha / k),
                                                         poses[j+1].pose.position.x, poses[j+1].pose.position.y, thetas[j+1], Kmax_);


          if (curve.L < best_length && std::abs(curve.a3.xf - poses[j+1].pose.position.x) + std::abs(curve.a3.yf - poses[j+1].pose.position.y) < best_err) {
          //  RCLCPP_INFO(this->get_logger(), "alpha: %i", alpha);

            curves[j] = curve;
            best_k = alpha;
            best_length = curve.L;
            best_err = std::abs(curve.a3.xf - poses[j+1].pose.position.x) + std::abs(curve.a3.yf - poses[j+1].pose.position.y);
          }
        }
        RCLCPP_INFO(this->get_logger(), "x0: %f, y0: %f, xf: %f, yf: %f", curves[j].a1.x0, curves[j].a1.y0, curves[j].a3.xf, curves[j].a3.yf);

        thetas[j] = M_2_PI * best_k / k;
      }

      DubinsCurve curve = Dubins::dubinsShortestPath(poses[0].pose.position.x,poses[0].pose.position.y, thetas[0],
                                                         poses[1].pose.position.x, poses[1].pose.position.y, thetas[1], Kmax_);
      curves[0] = curve;


      Path dubins_path;
      dubins_path.header.frame_id = "map";
      dubins_path.header.stamp = rclcpp::Clock().now();

      RCLCPP_INFO(this->get_logger(), "here4");

      for (unsigned i = 0; i < curves.size(); ++i) {
        constexpr int num_samples = 15; // sample numbers
        for (int j = 0; j< num_samples; ++j){
          DubinsCurve curve = curves[i];
          double s = curve.L * ((double)(j + 1)/(double)num_samples);
          double x, y, th;
          if (s <= curve.a1.L){
            std::tie(x,y,th) = computeArcPoint(s,curve.a1);
          }
          else if( s <= curve.a1.L + curve.a2.L){
            std::tie(x,y,th) = computeArcPoint(s - curve.a1.L,curve.a2);
          }
          else{
            std::tie(x,y,th) = computeArcPoint(s - curve.a1.L,curve.a3);        
          }
          // RCLCPP_INFO(this->get_logger(), "sample: %i - s: %f - x: %f, y: %f - L: %f", j, s, (float)x, (float)y, (float)curve.L);

          // RCLCPP_INFO(this->get_logger(),"pose z: %f, pose w: %f", sin(th/2), cos(th/2));


          // Add points into path
          geometry_msgs::msg::PoseStamped pose_stamped;
          pose_stamped.header = global_path.header;
          pose_stamped.pose.position.x = x;
          pose_stamped.pose.position.y = y;
          pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,sin(th/2),cos(th/2)));
          dubins_path.poses.push_back(pose_stamped);
        } // END: discretion of one arc dubins path
      }


      // iterate all points in the global path
      // for (size_t i = 0; i < poses.size() -1; ++i){
      //    RCLCPP_INFO(this->get_logger(), "here5");

      //   const auto &start_pose = poses[i].pose;
      //   const auto &goal_pose  = poses[i+1].pose;

      //   double x0 = start_pose.position.x;
      //   double y0 = start_pose.position.y;
      //   double th0 = get_yaw_from_q(start_pose.orientation);

      //   double xf = goal_pose.position.x;
      //   double yf = goal_pose.position.y;
      //   double thf = get_yaw_from_q(goal_pose.orientation);    

      //   // RCLCPP_INFO(this->get_logger(), "x0: %f, y0: %f, xf: %f, yf: %f", x0, y0, xf, yf);

      //   // call dubinShortestPath function in dubins.cpp
      //   DubinsCurve curve = Dubins::dubinsShortestPath(x0,y0,th0,xf,yf,thf,Kmax_);

      //   // RCLCPP_INFO(this->get_logger(), "Lenght: %f", curve.L);


      //   // Discrete dubins curve into path points
      //   constexpr int num_samples = 15; // sample numbers
      //   for (int j = 0; j< num_samples; ++j){
      //     double s = curve.L * ((double)(j + 1)/(double)num_samples);
      //     double x, y, th;
      //     if (s <= curve.a1.L){
      //       std::tie(x,y,th) = computeArcPoint(s,curve.a1);
      //     }
      //     else if( s <= curve.a1.L + curve.a2.L){
      //       std::tie(x,y,th) = computeArcPoint(s - curve.a1.L,curve.a2);
      //     }
      //     else{
      //       std::tie(x,y,th) = computeArcPoint(s - curve.a1.L,curve.a3);        
      //     }
      //     // RCLCPP_INFO(this->get_logger(), "sample: %i - s: %f - x: %f, y: %f - L: %f", j, s, (float)x, (float)y, (float)curve.L);

      //     // Add points into path
      //     geometry_msgs::msg::PoseStamped pose_stamped;
      //     pose_stamped.header = global_path.header;
      //     pose_stamped.pose.position.x = x;
      //     pose_stamped.pose.position.y = y;
      //     pose_stamped.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,sin(th/2),cos(th/2)));
      //     dubins_path.poses.push_back(pose_stamped);
      //   } // END: discretion of one arc dubins path
          
      // }// END: iteration global path
    RCLCPP_INFO(this->get_logger(), "here6");

    for (auto pose : dubins_path.poses) {
      RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z);

    }
    
    
    // Path path;
    // path.header.frame_id = "map";
    // path.header.stamp = rclcpp::Clock().now();

    // tf2::Quaternion q;
    // q.setRPY( 0, 0, 0);

    // for (float i = 5.0; i > 1.0; i-=0.05) {
    //   geometry_msgs::msg::PoseStamped pose;
    //   pose.pose.position.x = 2.0;
    //   pose.pose.position.y = i;
    //   pose.pose.position.z = 0.0;
    //   pose.pose.orientation = tf2::toMsg(q);

    //   path.poses.push_back(pose);
    // }

      //call move function to activate the action server: follow path
      this->move(robot_name, dubins_path);

    }

  
  }

}

void follow_client::goal_response_path_planning_callback(
  const ClientComputePathToPoseGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the ComputePathToPose server.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the ComputePathToPose server.");
}

void follow_client::feedback_path_planning_callback(
  ClientComputePathToPoseGoalHandle::SharedPtr,
  const std::shared_ptr<const ComputePathToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback from ComputePathToPose");
}

void follow_client::result_path_planning_callback(
  const ClientComputePathToPoseGoalHandle::WrappedResult & result)
{
  switch (result.code){
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Path planned, starting to move.");
      // this->move(result.result->path);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted from ComputePathToPose");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled by ComputePathToPose");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "[ComputePathToPose] Unknown result code");
      exit(1);
      break;
  }
}

// void follow_client::move(const std::string &robot_name,const Path& path)
// {
//   using namespace std::placeholders;
//   // find if there already exists a shelfino#/followpath action server
//   // if not create one 
//     RCLCPP_INFO(this->get_logger(), "here7");

//   // if(follow_path_clients_array.find(robot_name) == follow_path_clients_array.end()){
//     // follow_path_clients_array[robot_name] = rclcpp_action::create_client<FollowPath>(
//     //   this, "/"+ robot_name + "/follow_path");
//   // }
//     RCLCPP_INFO(this->get_logger(), "here8");

//   rclcpp_action::Client<FollowPath>::SharedPtr client_ = rclcpp_action::create_client<FollowPath>(this, "/"+ robot_name + "/follow_path");
//   follow_path_clients_array[robot_name] = client_;

//     RCLCPP_INFO(this->get_logger(), "here9");


//   RCLCPP_INFO(this->get_logger(), "Moving to gate");
//   if(!client_->wait_for_action_server(std::chrono::seconds(10))){
//     RCLCPP_ERROR(this->get_logger(), "Action server for %s not available after waiting.",robot_name.c_str());
//     exit(1);
//   }  

//     RCLCPP_INFO(this->get_logger(), "here10");


//   for (auto pose : path.poses) {
//     RCLCPP_INFO(this->get_logger(), "name: %s, x: %f, y: %f", robot_name.c_str(), pose.pose.position.x, pose.pose.position.y);

//   }

//   auto action_msg = FollowPath::Goal();
//   action_msg.path = path;
//   action_msg.controller_id = "FollowPath";

//     RCLCPP_INFO(this->get_logger(), "here11");


//   RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath action server.");

//   auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
//   send_goal_options.goal_response_callback = [this, robot_name](const auto &goal_handle){
//     if(!goal_handle){
//       RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the FollowPath server.");
//       exit(1);   
//     }
//     RCLCPP_INFO(this->get_logger(), "Goal accepted by the FollowPath server.");

//   };

//     RCLCPP_INFO(this->get_logger(), "here12");



//   send_goal_options.feedbackrobot_name
//   };
//     RCLCPP_INFO(this->get_logger(), "here13");


//   send_goal_options.result_callback = [this, robot_name](const auto &result){
//     switch (result.code){
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         RCLCPP_INFO(this->get_logger(), "Path followed, reached gate.");
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted from FollowPath");
//         break;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was canceled by FollowPath");
//         break;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "[FollowPath] Unknown result code");
//         exit(1);
//         break;
//     }
//   };

// //     send_goal_options.goal_response_callback = [this](const auto &goal_handle) {
// //     // Your callback logic
// //     if (goal_handle) {
// //         RCLCPP_INFO(this->get_logger(), "Goal was accepted");
// //     } else {
// //         RCLCPP_ERROR(this->get_logger(), "Goal was rejected");
// //     }
// // };

// //     send_goal_options.feedback_callback = [this](
// //     const auto &feedback) {
    
// //     if (feedback) {
// //         RCLCPP_INFO(this->get_logger(), "Current distance to goal: %f", feedback->distance_to_goal);
// //     } else {
// //         RCLCPP_WARN(this->get_logger(), "Feedback is null.");
// //     }
// // };

// //     send_goal_options.result_callback = [this](const auto &result) {
// //     try {
// //         // auto result = future.get();
// //         if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
// //             RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
// //         } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
// //             RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
// //         } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
// //             RCLCPP_WARN(this->get_logger(), "Goal was canceled.");
// //         } else {
// //             RCLCPP_ERROR(this->get_logger(), "Unknown result code: %d", result.code);
// //         }
// //     } catch (const std::exception &e) {
// //         RCLCPP_ERROR(this->get_logger(), "Error during result processing: %s", e.what());
// //     }
// // };


//     RCLCPP_INFO(this->get_logger(), "here14");


//   follow_path_clients_array[robot_name]->async_send_goal(action_msg, send_goal_options);       // send msgs to action server: followpath 

//     RCLCPP_INFO(this->get_logger(), "here15");

// }

void follow_client::move(const std::string &robot_name, const Path& path)
{
  RCLCPP_INFO(this->get_logger(), "Moving to gate");

  // for (auto pose : path.poses) {
  //   RCLCPP_INFO(this->get_logger(), "Pose: %f, %f", pose.pose.position.x, pose.pose.position.y);

  // }
  // rclcpp_action::Client<FollowPath>::SharedPtr client_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino1/follow_path");
  // follow_path_clients_array["shelfino1"] = client_;
  if(!this->follow_path_client_->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(this->get_logger(), "Action server for FollowPath not available after waiting.");
    exit(1);
  }

  auto action_msg = FollowPath::Goal();
  action_msg.path = path;


  RCLCPP_INFO(this->get_logger(), "Sending goal to FollowPath action server.");

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&follow_client::goal_response_path_following_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&follow_client::feedback_path_following_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&follow_client::result_path_following_callback, this, std::placeholders::_1);

  this->follow_path_client_->async_send_goal(action_msg, send_goal_options);
}

void follow_client::goal_response_path_following_callback(
  const ClientFollowPathGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle){
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the FollowPath server.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Goal accepted by the FollowPath server.");
}

void follow_client::feedback_path_following_callback(
  ClientFollowPathGoalHandle::SharedPtr,
  const std::shared_ptr<const FollowPath::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Received feedback from FollowPath");
  RCLCPP_INFO(this->get_logger(), "Distance to point: %f", feedback->distance_to_goal);
  RCLCPP_INFO(this->get_logger(), "Speed: %f", feedback->speed);

}

void follow_client::result_path_following_callback(
  const ClientFollowPathGoalHandle::WrappedResult & result)
{
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
  // rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<follow_client>();                  // create a node named "follow_client"
  // executor.add_node(node);
  // executor.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
