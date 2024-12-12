#include "shelfino_pathfollow/followclient.hpp"


follow_client::follow_client() : Node("follow_client")
{
  // Define the QoS
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_custom);

  // SUBSCRIPTION 1 : listen to amcl_pose topic
  // Listen to the /shelfino#/amcl_pose topic
  this->shelfino_pose_sub_ = this->create_subscription<PoseWithCovarianceStamped>(
    "/shelfino#/amcl_pose", qos, std::bind(&follow_client::handle_shelfino_pose, this, std::placeholders::_1),       // "topic_name" ?
    amcl_pose_options
  );

  // SUBSCRIPTION 2 : listen to dubins path topic?
  // TODO1: define the msgs type
  // TODO2: define callback function
  this->dubins_path_sub_ = this->create_subscription<>(
    "/dubinspath", qos, std::bind(&follow_client::handle_dubins_path, this, std::placeholders::_1)
  );

  // SUBSCRIPTION 3: Listen to the /gate_pose topic
  this->gates_pose_sub_ = this->create_subscription<PoseArray>(
    "/gates", qos, std::bind(&follow_client::handle_gate_pose, this, std::placeholders::_1)
  );

  // PUBLISH: to tell other shelfino robots that it is ready
  this->shelfino_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>("ready", qos);
  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  this->shelfino_ready_pub_->publish(msg);

  // SERVICE: Create the start service
  this->start_service_ = this->create_service<Empty>(
    "start", std::bind(&follow_client::start_callback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // ACTION: compute and follow path [Nav2]
  // TODO3: compute_path_to_pose client should be substitute to our own path planner
  this->compute_path_to_pose_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
  this->follow_path_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

  RCLCPP_INFO(this->get_logger(), "follow path client node created.");
}


void follow_client::handle_shelfino_pose(const PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received shelfino pose: x=%f, y=%f", msg->pose.pose.position.x, msg->pose.pose.position.y);
  this->shelfino_pose = msg->pose.pose;                             // store shelfino pose[x,y] in shelfino_pose
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

void follow_client::start_callback(const std::shared_ptr<Empty::Request> request,
            std::shared_ptr<Empty::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "`start` service called.");
  if (!this->gate_pose_ready){
    RCLCPP_ERROR(this->get_logger(), "Gate pose not set.");
    exit(1);
  }
  RCLCPP_INFO(this->get_logger(), "Navigating to x=%f, y=%f, yaw=%f", this->gate_pose.position.x, this->gate_pose.position.y, get_yaw_from_q(this->gate_pose.orientation));

  if(!this->compute_path_to_pose_client_->wait_for_action_server(std::chrono::seconds(10))){
    RCLCPP_ERROR(this->get_logger(), "Action server for ComputePathToPose not available after waiting.");
    exit(1);
  }

  // COMPUTE PATH SECTION: START
  auto action_msg = ComputePathToPose::Goal();
  action_msg.goal.pose = this->gate_pose;
  action_msg.use_start = false;                              // default setting: use robot pose as starting point
  action_msg.goal.header.stamp = this->now();
  action_msg.goal.header.frame_id = "map";

  RCLCPP_INFO(this->get_logger(), "Sending goal to ComputePathToPose action server.");

  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&follow_client::goal_response_path_planning_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(&follow_client::feedback_path_planning_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(&follow_client::result_path_planning_callback, this, std::placeholders::_1);

  this->compute_path_to_pose_client_->async_send_goal(action_msg, send_goal_options);
  // COMPUTE PATH SECTION: END
}

// COMPUTE PATH SECTION: CALLBACK FUNCTION START

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
      this->move(result.result->path);
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

// COMPUTE PATH SECTION: CALLBACK FUNCTION END
// RESULT: 


void follow_client::move(const Path& path)
{
  RCLCPP_INFO(this->get_logger(), "Moving to gate");
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


int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("follow_client"), "Starting follow path client node.");
  srand(time(NULL));
  rclcpp::init(argc, argv);                                       // initializes ROS2 C++ client library
  auto node = std::make_shared<follow_client>();                  // create a node named "follow_client"
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
