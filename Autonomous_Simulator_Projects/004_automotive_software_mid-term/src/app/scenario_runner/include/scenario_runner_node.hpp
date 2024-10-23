/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      scenario_runner_node.hpp
 * @brief     generate mission events node
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#ifndef __SCENARIO_RUNNER_NODE_HPP__
#define __SCENARIO_RUNNER_NODE_HPP__
#pragma once

// STD Header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>

// ROS Header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// Message Header
#include <ad_msgs/msg/vehicle_output.hpp>
#include <ad_msgs/msg/obstacles.hpp>
#include <std_msgs/msg/float32.hpp>

// Algorithm Header
#include "scenario_algorithm.hpp"

// Interface Header
#include "interface_vehicle.hpp"

// Parameter Header

class ScenarioRunnerNode : public rclcpp::Node {
public:
    ScenarioRunnerNode(const std::string& node_name, const double& loop_rate,
                 const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ScenarioRunnerNode();

    void Init(const rclcpp::Time& current_time);
    void Run(const rclcpp::Time& current_time);
    void UpdateObstacles(const std::vector<ObstacleState>& obstacles);
    void Publish();

private:
    // Functions
    inline void CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {
        i_vehicle_state_ = *msg;
        param_is_vehicle_ = true;
    }

    // Variables

    // Publisher
    rclcpp::Publisher<ad_msgs::msg::Obstacles>::SharedPtr p_obstacles_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr p_limit_speed_;

    // Subscriber
    rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;

    // Timer
    rclcpp::TimerBase::SharedPtr t_run_node_;

    // Input
    ad_msgs::msg::VehicleOutput i_vehicle_state_;

    // Output
    ad_msgs::msg::Obstacles o_obstacles_;
    std_msgs::msg::Float32 o_limit_speed_;

    // Mutex
    std::mutex mutex_vehicle_state_;

    // Algorithm
    std::unique_ptr<ScenarioAlgorithm> ptr_scenario_algorithm_;

    // Configuration parameters   
    ScenarioParams param_;

    // Time
    double time_prev_ = 0.0;
    double time_dt_ = 0.0;

    // Simulator
    bool param_is_vehicle_ = false;
};

#endif // __SCENARIO_RUNNER_NODE_HPP__
