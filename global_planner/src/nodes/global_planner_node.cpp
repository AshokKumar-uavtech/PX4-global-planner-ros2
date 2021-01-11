#include "global_planner/global_planner_node.h"

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode()
   : Node("global_planner_node") ,
     gp_cmdloop_dt_(100ms),
     gp_plannerloop_dt_(1000ms),
     start_yaw_(0.0)
    {
  RCLCPP_INFO_ONCE(this->get_logger(), "GlobalPlannerNode STARTED!");
 
 // #ifndef DISABLE_SIMULATION
  world_visualizer_ = std::make_shared<avoidance::WorldVisualizer>();
  world_visualizer_executor_.add_node(world_visualizer_);
  world_visualizer_thread = new std::thread([&]() {
    world_visualizer_executor_.spin();
  });
// #endif

  // Read Ros parameters
  readParams();

  // Subscribers
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  octomap_full_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_full", qos, std::bind(&GlobalPlannerNode::octomapFullCallback, this, _1));
  position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/agent1/vehicle_local_position", qos, std::bind(&GlobalPlannerNode::positionCallback, this, _1));
  attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
    "agent1/vehicle_attitude", qos, std::bind(&GlobalPlannerNode::attitudeCallback, this, _1));
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", qos, std::bind(&GlobalPlannerNode::clickedPointCallback, this, _1));
  move_base_simple_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", qos, std::bind(&GlobalPlannerNode::moveBaseSimpleCallback, this, _1));
  // nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlannerNode::moveBaseSimpleCallback, this);
  // fcu_input_sub_ = nh_.subscribe("/mavros/trajectory/desired", 1, &GlobalPlannerNode::fcuInputGoalCallback, this);

  // Publishers
  global_temp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_temp_path", 10);
  actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_path", 10);
  smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smooth_path", 10);
  global_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/global_goal", 10);
  global_temp_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/global_temp_goal", 10);
  explored_cells_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/explored_cells", 10);
  // mavros_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/setpoint_position/local", 10);
  mavros_obstacle_free_path_pub_ = this->create_publisher<px4_msgs::msg::VehicleTrajectoryWaypoint>("/trajectory/generated", 10);  
  current_waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_setpoint", 10);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_in", 10);

  transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  actual_path_.header.frame_id = frame_id_;

  gp_cmdloop_timer_ = this->create_wall_timer(gp_cmdloop_dt_, [&](){ cmdLoopCallback(); });
  gp_plannerloop_timer_ = this->create_wall_timer(gp_plannerloop_dt_, [&](){ plannerLoopCallback(); });

  current_goal_.header.frame_id = frame_id_;
  current_goal_.pose.position = start_pos_;
  current_goal_.pose.orientation = avoidance::createQuaternionMsgFromYaw(start_yaw_);
  last_goal_ = current_goal_;

  speed_ = 1.0;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  start_time_ = rclcpp::Clock().now();
}

GlobalPlannerNode::~GlobalPlannerNode() {}

void GlobalPlannerNode::readParams() {
  std::vector<std::string> camera_topics;

  this->declare_parameter("frame_id", "/base_frame");
  
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter_or("start_pos_x", start_pos_.x, 0.5);
  this->get_parameter_or("start_pos_y", start_pos_.y, 0.5);
  this->get_parameter_or("start_pos_z", start_pos_.z, 3.5);

  this->get_parameter("pointcloud_topics", camera_topics);
  camera_topics.push_back("/camera/points");

  initializeCameraSubscribers(camera_topics);
  global_planner_.goal_pos_ = GoalCell(start_pos_.x, start_pos_.y, start_pos_.z);
  double robot_radius;
  this->get_parameter_or("robot_radius", robot_radius, 0.5);
  global_planner_.setFrame(frame_id_);
  global_planner_.setRobotRadius(robot_radius);
}

void GlobalPlannerNode::initializeCameraSubscribers(std::vector<std::string>& camera_topics) {
  cameras_.resize(camera_topics.size());

  for (size_t i = 0; i < camera_topics.size(); i++) {
    cameras_[i].pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      camera_topics[i], 1, std::bind(&GlobalPlannerNode::depthCameraCallback, this, _1));
  }
}

// Sets a new goal, plans a path to it and publishes some info
void GlobalPlannerNode::setNewGoal(const GoalCell& goal) {
  RCLCPP_INFO(this->get_logger(), "========== Set goal : %s ==========", goal.asString().c_str());
  global_planner_.setGoal(goal);
  publishGoal(goal);
}

// Sets the next waypoint to be the current goal
void GlobalPlannerNode::popNextGoal() {
  if (!waypoints_.empty()) {
    // Set the first goal in waypoints_ as the new goal
    GoalCell new_goal = waypoints_.front();
    waypoints_.erase(waypoints_.begin());
    setNewGoal(new_goal);
  } else if (global_planner_.goal_is_blocked_) {
    // Goal is blocked but there is no other goal in waypoints_, just stop
    RCLCPP_INFO(this->get_logger(), "  STOP  ");
    global_planner_.stop();
  }
}

// Plans a new path and publishes it
void GlobalPlannerNode::planPath() {
  std::clock_t start_time = std::clock();
  if (global_planner_.octree_) {
    RCLCPP_INFO(this->get_logger(), "OctoMap memory usage: %2.3f MB", global_planner_.octree_->memoryUsage() / 1000000.0);
  }

  bool found_path = global_planner_.getGlobalPath();

  if (!found_path) {
    // TODO: popNextGoal(), instead of checking if goal_is_blocked in
    // positionCallback?
    RCLCPP_INFO(this->get_logger(), "Failed to find a path");
  } else if (global_planner_.overestimate_factor_ > 1.05) {
    // The path is not good enough, set an intermediate goal on the path
    setIntermediateGoal();
  }
  printf("Total time: %2.2f ms \n", (std::clock() - start_time) / (double)(CLOCKS_PER_SEC / 1000));
}

// Sets a temporary goal on the path to the current goal
void GlobalPlannerNode::setIntermediateGoal() {
  int curr_path_length = global_planner_.curr_path_.size();
  if (curr_path_length > 10) {
    printf("\n ===== Half-way path ====== \n");
    waypoints_.insert(waypoints_.begin(), global_planner_.goal_pos_);
    Cell middle_cell = global_planner_.curr_path_[curr_path_length / 2];
    setNewGoal(GoalCell(middle_cell, curr_path_length / 4, true));
  }
}

// void GlobalPlannerNode::dynamicReconfigureCallback(global_planner::GlobalPlannerNodeConfig& config, uint32_t level) {
//   // global_planner_
//   global_planner_.min_altitude_ = config.min_altitude_;
//   global_planner_.max_altitude_ = config.max_altitude_;
//   global_planner_.max_cell_risk_ = config.max_cell_risk_;
//   global_planner_.smooth_factor_ = config.smooth_factor_;
//   global_planner_.vert_to_hor_cost_ = config.vert_to_hor_cost_;
//   global_planner_.risk_factor_ = config.risk_factor_;
//   global_planner_.neighbor_risk_flow_ = config.neighbor_risk_flow_;
//   global_planner_.expore_penalty_ = config.expore_penalty_;
//   global_planner_.up_cost_ = config.up_cost_;
//   global_planner_.down_cost_ = config.down_cost_;
//   global_planner_.search_time_ = config.search_time_;
//   global_planner_.min_overestimate_factor_ = config.min_overestimate_factor_;
//   global_planner_.max_overestimate_factor_ = config.max_overestimate_factor_;
//   global_planner_.max_iterations_ = config.max_iterations_;
//   global_planner_.goal_must_be_free_ = config.goal_must_be_free_;
//   global_planner_.use_current_yaw_ = config.use_current_yaw_;
//   global_planner_.use_risk_heuristics_ = config.use_risk_heuristics_;
//   global_planner_.use_speedup_heuristics_ = config.use_speedup_heuristics_;

//   // global_planner_node
//   clicked_goal_alt_ = config.clicked_goal_alt_;
//   clicked_goal_radius_ = config.clicked_goal_radius_;
//   simplify_iterations_ = config.simplify_iterations_;
//   simplify_margin_ = config.simplify_margin_;

//   // cell
//   if (level == 2) {
//     CELL_SCALE = config.CELL_SCALE;
//   }

//   // node
//   if (level == 4) {
//     SPEEDNODE_RADIUS = config.SPEEDNODE_RADIUS;
//     global_planner_.default_node_type_ = config.default_node_type_;
//   }
// }

void GlobalPlannerNode::attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  tf2::Quaternion q( msg->q[0], msg->q[1], msg->q[2], msg->q[3] );
  double yaw, pitch, roll;
  tf2::getEulerYPR(q, yaw, pitch, roll);
  tf2::Quaternion q_NED;
  // q_NED.setEuler(-roll-3.141592, -pitch, yaw);
  q_NED.setRPY(yaw, -pitch, -roll-3.141592);
  geometry_msgs::msg::Quaternion quat_geomsg;
  tf2::convert(q_NED, quat_geomsg);
  // printf("Roll %lf, Pitch %lf, Yaw %lf\n", roll*180/3.141592, pitch*180/3.141592, yaw*180/3.141592);
  // roll = yaw
  // pitch = -pitch
  // yaw = -(180+roll)
  // printf("Roll %lf, Pitch %lf, Yaw %lf\n\n", yaw*180/3.141592, -pitch*180/3.141592, -(180+roll*180/3.141592));

  geometry_msgs::msg::TransformStamped tfmsg;
  tfmsg.header.stamp = rclcpp::Clock().now();
  tfmsg.header.frame_id = "local_origin";
  tfmsg.child_frame_id = "local_origin_odom";
  tfmsg.transform.rotation = quat_geomsg;
  transform_broadcaster_->sendTransform(tfmsg);
}

// Sets the current position and checks if the current goal has been reached
void GlobalPlannerNode::positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped tfmsg;
  tfmsg.header.stamp = rclcpp::Clock().now();
  tfmsg.header.frame_id = "base_frame";
  tfmsg.child_frame_id = "local_origin";
  tfmsg.transform.translation.x = msg->x;
  tfmsg.transform.translation.y = msg->y;
  tfmsg.transform.translation.z = -(msg->z);
  transform_broadcaster_->sendTransform(tfmsg);

  auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose->pose.position.x = msg->x;
  pose->pose.position.y = msg->y;
  pose->pose.position.z = -(msg->z);
  pose->pose.orientation = avoidance::createQuaternionMsgFromYaw(msg->yaw);

  // Update position
  last_pos_ = *pose;
  global_planner_.setPose(pose, msg->yaw);

  // Update velocity
  auto vel = std::make_shared<geometry_msgs::msg::Vector3>();
  vel->x = msg->vx;
  vel->y = msg->vy;
  vel->z = -(msg->vz);
  global_planner_.curr_vel_ = *vel;

  // Check if a new goal is needed
  if (num_pos_msg_++ % 100 == 0) {
    // Keep track of and publish the actual travel trajectory
    RCLCPP_INFO(this->get_logger(), "Travelled path extended");
    last_pos_.header.frame_id = frame_id_;
    actual_path_.poses.push_back(last_pos_);
    actual_path_pub_->publish(actual_path_);
  }

  position_received_ = true;

  // Check if we are close enough to current goal to get the next part of the
  // path
  if (path_.size() > 0 && isCloseToGoal()) {
    double yaw1 = tf2::getYaw(current_goal_.pose.orientation);
    double yaw2 = tf2::getYaw(last_pos_.pose.orientation);
    double yaw_diff = std::abs(yaw2 - yaw1);
    // Transform yaw_diff to [0, 2*pi]
    yaw_diff -= std::floor(yaw_diff / (2 * M_PI)) * (2 * M_PI);
    double max_yaw_diff = M_PI / 1.0;
    if (yaw_diff < max_yaw_diff || yaw_diff > 2 * M_PI - max_yaw_diff) {
      // If we are facing the right direction, then pop the first point of the
      // path
      last_goal_ = current_goal_;
      current_goal_ = path_[0];
      path_.erase(path_.begin());
    }
  }
}

void GlobalPlannerNode::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  printPointInfo(msg->point.x, msg->point.y, msg->point.z);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose.position = msg->point;
  pose.pose.position.z = global_planner_.curr_pos_.z;
  last_clicked_points.push_back(pose);
}

void GlobalPlannerNode::moveBaseSimpleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  printf("########### moveBaseSimpleCallback. %lf %lf %lf\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  setNewGoal(GoalCell(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
}

// void GlobalPlannerNode::fcuInputGoalCallback(const mavros_msgs::msg::Trajectory& msg) {
//   const GoalCell new_goal = GoalCell(msg.point_2.position.x, msg.point_2.position.y, msg.point_2.position.z, 1.0);
//   if (msg.point_valid[1] == true && ((std::fabs(global_planner_.goal_pos_.xPos() - new_goal.xPos()) > 0.001) ||
//                                      (std::fabs(global_planner_.goal_pos_.yPos() - new_goal.yPos()) > 0.001))) {
//     setNewGoal(new_goal);
//   }
// }

// // Check if the current path is blocked
void GlobalPlannerNode::octomapFullCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "octomapFullCallback called!");
  
  std::lock_guard<std::mutex> lock(mutex_);

  rclcpp::Time current = rclcpp::Clock().now();
  // Update map at a fixed rate. This is useful on setting replanning rates for the planner.
  if ((current - last_wp_time_).seconds() < mapupdate_dt_) {
    return;
  }
  last_wp_time_ = rclcpp::Clock().now();

  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);

  global_planner_.updateFullOctomap(tree);
}

// Go through obstacle points and store them
void GlobalPlannerNode::depthCameraCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pointcloud2_ = *msg;
  tf_buffer_->waitForTransform("base_frame", "camera_frame", rclcpp::Clock().now(), std::chrono::milliseconds(1000),
    std::bind(&GlobalPlannerNode::tf2Callback, this, _1));
}

void GlobalPlannerNode::tf2Callback(const std::shared_future<geometry_msgs::msg::TransformStamped>& tf) {
  try {
    geometry_msgs::msg::TransformStamped transformStamped = tf.get();

    // printf("Got Transformation!! %lf %lf %lf\n", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    sensor_msgs::msg::PointCloud2 transformed_msg;
    tf2::doTransform(pointcloud2_, transformed_msg, transformStamped);
    pcl::PointCloud<pcl::PointXYZ> cloud;  // Easier to loop through pcl::PointCloud
    pcl::fromROSMsg(transformed_msg, cloud);

    // Store the obstacle points
    for (const auto& p : cloud) {
      // printf("p.x : %lf, p.y : %lf, p.z : %lf\n", p.x, p.y, p.z);
      if (!std::isnan(p.x) && p.z >= 1.0) {
        // TODO: Not all points end up here
        // printf("p.x : %lf, p.y : %lf, p.z : %lf\n", p.x, p.y, p.z);
        Cell occupied_cell(p.x, p.y, p.z);
        global_planner_.occupied_.insert(occupied_cell);
      }
    }
    
    pointcloud_pub_->publish(transformed_msg);

    // RCLCPP_INFO(this->get_logger(), "success to transfrom!");
  } catch(tf2::TimeoutException const& ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
  }
}

void GlobalPlannerNode::setCurrentPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses) {
  path_.clear();

  if (poses.size() < 2) {
    RCLCPP_INFO(this->get_logger(), "  Received empty path\n");
    return;
  }
  last_goal_ = poses[0];
  current_goal_ = poses[1];

  for (int i = 2; i < poses.size(); ++i) {
    path_.push_back(poses[i]);
  }
}

void GlobalPlannerNode::cmdLoopCallback() {
  hover_ = false;

  // Check if all information was received
  rclcpp::Time now = rclcpp::Clock().now();

  rclcpp::Duration since_last_cloud = now - last_wp_time_;
  rclcpp::Duration since_start = now - start_time_;

  avoidance_node_.checkFailsafe(since_last_cloud, since_start, hover_);
  publishSetpoint();
}

void GlobalPlannerNode::plannerLoopCallback() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool is_in_goal = global_planner_.goal_pos_.withinPositionRadius(global_planner_.curr_pos_);
  if (is_in_goal || global_planner_.goal_is_blocked_) {
    popNextGoal();
  }

  planPath();

  // Print and publish info
  if (is_in_goal && !waypoints_.empty()) {
    RCLCPP_INFO(this->get_logger(), "Reached current goal %s, %d goals left\n\n", global_planner_.goal_pos_.asString().c_str(), (int)waypoints_.size());
    RCLCPP_INFO(this->get_logger(), "Actual travel distance: %2.2f \t Actual energy usage: %2.2f", pathLength(actual_path_), pathEnergy(actual_path_, global_planner_.up_cost_));
  }

  publishPath();

  // world_visualizer_.spin_some();
}

// Publish the position of goal
void GlobalPlannerNode::publishGoal(const GoalCell& goal) {
  geometry_msgs::msg::PointStamped pointMsg;
  pointMsg.header.frame_id = frame_id_;
  pointMsg.point = goal.toPoint();

  // Always publish as temporary to remove any obsolete temporary path
  global_temp_goal_pub_->publish(pointMsg);
  if (!goal.is_temporary_) {
    global_goal_pub_->publish(pointMsg);
  }
}

// Publish the current path
void GlobalPlannerNode::publishPath() {
  auto path_msg = global_planner_.getPathMsg();
  // PathWithRiskMsg risk_msg = global_planner_.getPathWithRiskMsg();
  // Always publish as temporary to remove any obsolete temporary path
  global_temp_path_pub_->publish(path_msg);
  setCurrentPath(path_msg.poses);
  smooth_path_pub_->publish(smoothPath(path_msg));

  // auto simple_path = simplifyPath(&global_planner_, global_planner_.curr_path_, simplify_iterations_, simplify_margin_);
  // auto simple_path_msg = global_planner_.getPathMsg(simple_path);
  // global_temp_path_pub_->publish(simple_path_msg);
  // setCurrentPath(simple_path_msg.poses);
  // smooth_path_pub_->publish(smoothPath(simple_path_msg));
}

// Prints information about the point, mostly the risk of the containing cell
void GlobalPlannerNode::printPointInfo(double x, double y, double z) {
  // Update explored cells
  printPointStats(&global_planner_, x, y, z);
}

void GlobalPlannerNode::publishSetpoint() {
  // Vector pointing from current position to the current goal
  tf2::Vector3 vec = toTfVector3(subtractPoints(current_goal_.pose.position, last_pos_.pose.position));
  // If we are less than 1.0 away, then we should stop at the goal
  double new_len = vec.length() < 1.0 ? vec.length() : speed_;
  vec.normalize();
  vec *= new_len;

  auto setpoint = current_goal_;  // The intermediate position sent to Mavros
  setpoint.pose.position.x = last_pos_.pose.position.x + vec.getX();
  setpoint.pose.position.y = last_pos_.pose.position.y + vec.getY();
  setpoint.pose.position.z = last_pos_.pose.position.z + vec.getZ();

  // Publish setpoint for vizualization
  current_waypoint_publisher_->publish(setpoint);

  // Publish setpoint to Mavros
  // mavros_waypoint_publisher_->publish(setpoint);
  px4_msgs::msg::VehicleTrajectoryWaypoint obst_free_path;
  geometry_msgs::msg::Twist velocity_setpoint{};
  velocity_setpoint.linear.x = NAN;
  velocity_setpoint.linear.y = NAN;
  velocity_setpoint.linear.z = NAN;

  avoidance::transformToTrajectory(obst_free_path, setpoint, velocity_setpoint);
  mavros_obstacle_free_path_pub_->publish(obst_free_path);
}

bool GlobalPlannerNode::isCloseToGoal() { 
  return distance(current_goal_, last_pos_) < 1.5;
}

}  // namespace global_planner
