/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 * 
 * @file      autonomous_driving.hpp
 * @brief     autonomous driving algorithm 
 * 
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 *            2024-10-21 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 *              : add the missions
 */

#ifndef __AUTONOMOUS_DRIVING_HPP__
#define __AUTONOMOUS_DRIVING_HPP__
#pragma once

// STD Header - Standard C++ libraries (표준 C++ 라이브러리)
#include <memory>        // Smart pointers and memory management (스마트 포인터 및 메모리 관리)
#include <mutex>        // Mutual exclusion for thread safety (스레드 안전성을 위한 상호 배제)
#include <utility>      // Pair and utility functions (페어 및 유틸리티 함수)
#include <vector>       // Dynamic array container (동적 배열 컨테이너)
#include <string>       // String handling (문자열 처리)
#include <cmath>        // Mathematical operations (수학 연산)
#include <chrono>       // Time and duration utilities (시간 및 지속시간 유틸리티)

// ROS Header - Robot Operating System core functionality (로봇 운영체제 핵심 기능)
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/message_info.hpp>

// ROS Message Header - Custom message types for autonomous driving (자율주행을 위한 커스텀 메시지 타입)
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>    // Lane detection using polynomial fitting (다항식 피팅을 이용한 차선 검출)
#include <ad_msgs/msg/lane_point_data_array.hpp>      // Array of lane point data (차선 포인트 데이터 배열)
#include <ad_msgs/msg/lane_point_data.hpp>            // Individual lane point data (개별 차선 포인트 데이터)
#include <ad_msgs/msg/vehicle_input.hpp>              // Vehicle control inputs (차량 제어 입력)
#include <ad_msgs/msg/vehicle_output.hpp>             // Vehicle state outputs (차량 상태 출력)
#include <ad_msgs/msg/obstacles.hpp>                  // Obstacle detection data (장애물 감지 데이터)
#include <std_msgs/msg/float32.hpp>                   // Standard floating point messages (표준 부동소수점 메시지)
#include <geometry_msgs/msg/point.hpp>                // Geometric point data (기하학적 포인트 데이터)
#include <geometry_msgs/msg/point_stamped.hpp>        // Timestamped point data (타임스탬프가 있는 포인트 데이터)

// Algorithm Header - Mathematical tools (수학적 도구)
#include <eigen3/Eigen/Dense>                         // Linear algebra operations (선형대수 연산)

// Main Autonomous Driving Class (주요 자율주행 클래스)
class AutonomousDriving : public rclcpp::Node {
    public:
        // Constructor and Destructor (생성자와 소멸자)
        AutonomousDriving(const std::string& node_name, const double& loop_rate,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~AutonomousDriving();

        // Core functions (핵심 함수들)
        void Init(const rclcpp::Time& current_time);          // Initialization function (초기화 함수)
        void Run(const rclcpp::Time& current_time);           // Main execution loop (메인 실행 루프)
        void Publish(const rclcpp::Time& current_time);       // Data publishing function (데이터 발행 함수)
        void UpdateParameter();                               // Parameter update function (파라미터 업데이트 함수)

        // Control function (제어 함수)
        // PID controller for vehicle control (차량 제어를 위한 PID 컨트롤러)
        void PID(const rclcpp::Time& current_time, double target_value, double current_value);

    private:
        // Callback functions for subscribers (구독자를 위한 콜백 함수들)
        
        // Manual input callback (수동 입력 콜백)
        inline void CallbackManualInput(const ad_msgs::msg::VehicleInput::SharedPtr msg) {            
            mutex_manual_input_.lock();
            if(param_use_manual_inputs_ == true) {
                o_vehicle_command_.accel = msg->accel;         // Acceleration command (가속 명령)
                o_vehicle_command_.brake = msg->brake;         // Brake command (제동 명령)
                o_vehicle_command_.steering = msg->steering;   // Steering command (조향 명령)
            }
            mutex_manual_input_.unlock();
        }

        // Vehicle state callback (차량 상태 콜백)
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {            
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = *msg;
            mutex_vehicle_state_.unlock();
        }

        // Lane points callback (차선 포인트 콜백)
        inline void CallbackLanePoints(const ad_msgs::msg::LanePointData::SharedPtr msg) {            
            mutex_lane_points_.lock();
            i_lane_points_ = *msg;
            mutex_lane_points_.unlock();
        }

        // Obstacles callback (장애물 콜백)
        inline void CallbackObstacles(const ad_msgs::msg::Obstacles::SharedPtr msg) {
            mutex_obstacles_.lock();
            i_obstacles_ = *msg;
            mutex_obstacles_.unlock();
        }

        // Speed limit callback (속도 제한 콜백)
        inline void CallbackLimitSpeed(const std_msgs::msg::Float32::SharedPtr msg) {
            mutex_limit_speed_.lock();
            i_limit_speed_ = msg->data;
            mutex_limit_speed_.unlock();
        }
        
        // Class member variables (클래스 멤버 변수들)

        // Publishers (발행자들)
        rclcpp::Publisher<ad_msgs::msg::VehicleInput>::SharedPtr p_vehicle_command_;        // Vehicle command publisher (차량 명령 발행자)
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_driving_way_;         // Driving path publisher (주행 경로 발행자)
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_poly_lanes_;     // Lane polynomial publisher (차선 다항식 발행자)

        // Subscribers (구독자들)
        rclcpp::Subscription<ad_msgs::msg::VehicleInput>::SharedPtr s_manual_input_;        // Manual input subscriber (수동 입력 구독자)
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;      // Vehicle state subscriber (차량 상태 구독자)
        rclcpp::Subscription<ad_msgs::msg::Obstacles>::SharedPtr s_obstacles_;              // Obstacles subscriber (장애물 구독자)
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr s_limit_speed_;            // Speed limit subscriber (속도 제한 구독자)
        rclcpp::Subscription<ad_msgs::msg::LanePointData>::SharedPtr s_lane_points_;        // Lane points subscriber (차선 포인트 구독자)
        
        // Timer for periodic execution (주기적 실행을 위한 타이머)
        rclcpp::TimerBase::SharedPtr t_run_node_;
        
        // Input variables (입력 변수들)
        ad_msgs::msg::VehicleOutput i_vehicle_state_;     // Current vehicle state (현재 차량 상태)
        ad_msgs::msg::Obstacles i_obstacles_;             // Detected obstacles (감지된 장애물)
        ad_msgs::msg::LanePointData i_lane_points_;       // Detected lane points (감지된 차선 포인트)
        double i_limit_speed_ = 0.0;                      // Current speed limit (현재 속도 제한)

        // Mutex locks for thread safety (스레드 안전성을 위한 뮤텍스 잠금)
        std::mutex mutex_manual_input_;
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_obstacles_;
        std::mutex mutex_limit_speed_;
        std::mutex mutex_lane_points_;        

        // Output variables (출력 변수들)
        ad_msgs::msg::VehicleInput o_vehicle_command_;        // Vehicle control commands (차량 제어 명령)
        ad_msgs::msg::PolyfitLaneData o_driving_way_;         // Generated driving path (생성된 주행 경로)
        ad_msgs::msg::PolyfitLaneDataArray o_poly_lanes_;     // Polynomial lane representations (다항식 차선 표현)
        
        // Parameters and constants (파라미터와 상수들)
        double time_prev_csv_lanes_;
        bool param_is_simulator_on_ = false;              // Simulator mode flag (시뮬레이터 모드 플래그)

        std::string param_vehicle_namespace_;
        bool param_use_manual_inputs_ = false;            // Manual input mode flag (수동 입력 모드 플래그)
        const double param_wheel_base_ = 1.302 + 1.398;   // Vehicle wheelbase - L_f + L_r (차량 축거)
        const double param_max_lateral_accel_ = 6200.0 / 1319.91;   // Maximum lateral acceleration - Fyf_max / Mass (최대 횡방향 가속도)

        // Control parameters (제어 파라미터들)
        // Pure pursuit controller gains (Pure pursuit 제어기 게인)
        double param_pp_kd_ = 1.0;                        // Distance gain (거리 게인)
        double param_pp_kv_ = 0.0;                        // Velocity gain (속도 게인)
        double param_pp_kc_ = 0.0;                        // Curvature gain (곡률 게인)
        
        // PID controller gains (PID 제어기 게인)
        double param_pid_kp_ = 0.0;                       // Proportional gain (비례 게인)
        double param_pid_ki_ = 0.0;                       // Integral gain (적분 게인)
        double param_pid_kd_ = 0.0;                       // Derivative gain (미분 게인)
        double param_brake_ratio_ = 1.0;                  // Brake force ratio (제동력 비율)

        // Region of Interest parameters (관심 영역 파라미터)
        double param_m_ROIFront_param = 20.0;             // Forward ROI distance (전방 ROI 거리)
        double param_m_ROIRear_param = 10.0;              // Rear ROI distance (후방 ROI 거리)
        double param_m_ROILeft_param = 3.0;               // Left ROI distance (좌측 ROI 거리)
        double param_m_ROIRight_param = 3.0;              // Right ROI distance (우측 ROI 거리)
        std::string param_ref_csv_path;                   // Reference path CSV file (기준 경로 CSV 파일)

        // Control algorithm variables (제어 알고리즘 변수들)
        double speed_error_integral_ = 0.0;               // Integrated speed error (적분된 속도 오차)
        double speed_error_prev_     = 0.0;               // Previous speed error (이전 속도 오차)

        // PID control variables (PID 제어 변수들)
        double e = 0.0;                                   // Current error (현재 오차)
        double e_prev = 0.0;                              // Previous error (이전 오차)
        double int_e = 0.0;                               // Integral of error (오차 적분)
        double int_e_prev = 0.0;                          // Previous integral (이전 적분값)
        double dev_e = 0.0;                               // Derivative of error (오차 미분)
        double computed_input = 0.0;                      // Computed control input (계산된 제어 입력)
        double input = 0.0;                               // Final control input (최종 제어 입력)
        rclcpp::Time pid_last_time = this->now();        // Last PID update time (마지막 PID 업데이트 시간)
};

#endif // __AUTONOMOUS_DRIVING_HPP__
