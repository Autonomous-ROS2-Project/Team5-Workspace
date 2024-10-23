/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      scenario_runner_node.cpp
 * @brief     generate mission events node
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#include "scenario_runner_node.hpp"

ScenarioRunnerNode::ScenarioRunnerNode(const std::string& node_name, const double& loop_rate,
                           const rclcpp::NodeOptions& options)
    : Node(node_name, options) {
    
    RCLCPP_INFO(this->get_logger(), "Initializing Obstacle Node...");
    
    // QoS init
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // Parameters    
    this->declare_parameter("scenario_runner/num_of_lanes", 3);
    if (!this->get_parameter("scenario_runner/num_of_lanes", param_.num_of_lanes)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get num_of_lanes");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "num_of_lanes: %d", param_.num_of_lanes);
    }
    this->declare_parameter("scenario_runner/num_of_obstacles", 3);
    if (!this->get_parameter("scenario_runner/num_of_obstacles", param_.num_of_obstacles)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get num_of_obstacles");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "num_of_obstacles: %d", param_.num_of_obstacles);
    }
    this->declare_parameter("scenario_runner/obstacle_init_s", 30.0);
    if (!this->get_parameter("scenario_runner/obstacle_init_s", param_.obstacle_init_s)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get obstacle_init_s");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "obstacle_init_s: %f", param_.obstacle_init_s);
    }
    this->declare_parameter("scenario_runner/obstacle_detection_distance", 20.0);
    if (!this->get_parameter("scenario_runner/obstacle_detection_distance", param_.obstacle_detection_distance)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get obstacle_detection_distance");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "obstacle_detection_distance: %f", param_.obstacle_detection_distance);
    }
    
    std::string dir(getenv("PWD"));
    std::string csv_path("/resources/csv/evaluation_lane");
    param_.ref_csv_path = dir + csv_path;
    RCLCPP_INFO(this->get_logger(), "ref_csv_path: %s", param_.ref_csv_path.c_str());

    // Publisher
    p_obstacles_ = this->create_publisher<ad_msgs::msg::Obstacles>(
        "obstacles", qos_profile);
    p_limit_speed_ = this->create_publisher<std_msgs::msg::Float32>(
        "limit_speed", qos_profile);

    // Subscriber
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput> (
        "vehicle_state", qos_profile, std::bind(&ScenarioRunnerNode::CallbackVehicleState, this, std::placeholders::_1));

    // Initialize
    Init(this->now());

    // Timer init
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]() { this->Run(this->now()); });        
}

ScenarioRunnerNode::~ScenarioRunnerNode() {}

void ScenarioRunnerNode::Init(const rclcpp::Time& current_time) {   

    // Algorithm
    ptr_scenario_algorithm_ = std::make_unique<ScenarioAlgorithm>(param_);    

    time_prev_ = current_time.seconds();
}

void ScenarioRunnerNode::Run(const rclcpp::Time& current_time) {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get subscribe variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_output     = i_vehicle_state_;
    mutex_vehicle_state_.unlock();
    
    double curr_time = current_time.seconds();
    time_dt_ = curr_time - time_prev_;
    time_prev_ = curr_time;

    VehicleState vehicle_state;
    vehicle_state.id = vehicle_output.id;
    vehicle_state.x = vehicle_output.x;
    vehicle_state.y = vehicle_output.y;
    vehicle_state.yaw = vehicle_output.yaw;
    vehicle_state.velocity = vehicle_output.velocity;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Set output variables
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    std::vector<ObstacleState> obstacles;
    std_msgs::msg::Float32 limit_speed;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Algorithm
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    ptr_scenario_algorithm_->UpdateEgoState(vehicle_state);
    ptr_scenario_algorithm_->UpdateObstacleState(time_dt_);

    obstacles = ptr_scenario_algorithm_->GetObstacleState();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Update output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    UpdateObstacles(obstacles);
    o_limit_speed_.data = ptr_scenario_algorithm_->GetEgoLimitSpeed();

    // ptr_scenario_algorithm_->GenerateCarFollowingReferenceLane(time_dt_);
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Publish output
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    Publish();
}

void ScenarioRunnerNode::UpdateObstacles(const std::vector<ObstacleState>& obstacles) {
    ad_msgs::msg::Obstacles obstacle_msg;

    for(auto ob : obstacles) {
        if(ob.is_reach_target == false) {
            ad_msgs::msg::VehicleOutput obstacle;
            obstacle.id = ob.id;
            obstacle.x = ob.x;
            obstacle.y = ob.y;
            obstacle.yaw = ob.yaw;
            obstacle.velocity = ob.velocity;
            obstacle.length = 4.635;
            obstacle.width = 1.890;

            obstacle_msg.obstacles.push_back(obstacle);
        }
    }

    o_obstacles_ = obstacle_msg;
}

void ScenarioRunnerNode::Publish() {
    p_limit_speed_->publish(o_limit_speed_);
    p_obstacles_->publish(o_obstacles_);
}

int main(int argc, char **argv) {
    std::string node_name = "scenario_runner_node";
    double loop_rate      = 100.0;

    // Initialize node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScenarioRunnerNode>(node_name, loop_rate));
    rclcpp::shutdown();

    return 0;
}
