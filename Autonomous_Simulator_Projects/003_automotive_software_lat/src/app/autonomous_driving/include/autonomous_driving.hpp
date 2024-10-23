/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 * 
 * @file      autonomous_driving.hpp
 * @brief     autonomous driving algorithm with PID control and Pure Pursuit steering
 * 
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : adapt new template
 *            2023-08-20 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : change to ROS2
 */

#ifndef __AUTONOMOUS_DRIVING_HPP__
#define __AUTONOMOUS_DRIVING_HPP__
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

// ROS Message Header
#include <ad_msgs/msg/polyfit_lane_data_array.hpp>
#include <ad_msgs/msg/lane_point_data_array.hpp>
#include <ad_msgs/msg/lane_point_data.hpp>
#include <ad_msgs/msg/vehicle_input.hpp>
#include <ad_msgs/msg/vehicle_output.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

// Algorithm Header
#include <eigen3/Eigen/Dense>  // 차선 피팅을 위한 Eigen 라이브러리

class AutonomousDriving : public rclcpp::Node {
    public:
        // 생성자 및 소멸자
        AutonomousDriving(const std::string& node_name, const double& loop_rate,
                          const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~AutonomousDriving();

        // 기본 함수들
        void Init(const rclcpp::Time& current_time);
        void Run(const rclcpp::Time& current_time);
        void Publish(const rclcpp::Time& current_time);
        void UpdateParameter();

    private:
        // [기존] Callback 함수들
        inline void CallbackManualInput(const ad_msgs::msg::VehicleInput::SharedPtr msg) {            
            mutex_manual_input_.lock();
            if(param_use_manual_inputs_ == true) {
                o_vehicle_command_.accel = msg->accel;
                o_vehicle_command_.brake = msg->brake;
                o_vehicle_command_.steering = msg->steering;
            }
            mutex_manual_input_.unlock();
        }
        
        inline void CallbackVehicleState(const ad_msgs::msg::VehicleOutput::SharedPtr msg) {            
            mutex_vehicle_state_.lock();
            i_vehicle_state_ = *msg;
            mutex_vehicle_state_.unlock();
        }
        
        inline void CallbackLanePointsArray(const ad_msgs::msg::LanePointDataArray::SharedPtr msg) {            
            mutex_lane_points_.lock();
            i_lane_points_array_ = *msg;
            mutex_lane_points_.unlock();
        }

        // [신규] PID 제어기 함수 추가
        void PID(const rclcpp::Time& current_time, double target_value, double current_value);
        
        // ROS2 Publisher 변수들
        rclcpp::Publisher<ad_msgs::msg::VehicleInput>::SharedPtr p_vehicle_command_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneData>::SharedPtr p_driving_way_;
        rclcpp::Publisher<ad_msgs::msg::PolyfitLaneDataArray>::SharedPtr p_poly_lanes_;

        // ROS2 Subscriber 변수들
        rclcpp::Subscription<ad_msgs::msg::VehicleInput>::SharedPtr s_manual_input_;
        rclcpp::Subscription<ad_msgs::msg::VehicleOutput>::SharedPtr s_vehicle_state_;
        rclcpp::Subscription<ad_msgs::msg::LanePointDataArray>::SharedPtr s_lane_points_array_;

        // Timer 변수
        rclcpp::TimerBase::SharedPtr t_run_node_;
        
        // 입력 메시지 저장 변수
        ad_msgs::msg::VehicleOutput i_vehicle_state_;
        ad_msgs::msg::LanePointDataArray i_lane_points_array_;

        // 뮤텍스 변수들 - 쓰레드 안전성 보장
        std::mutex mutex_manual_input_;
        std::mutex mutex_vehicle_state_;
        std::mutex mutex_lane_points_;        

        // 출력 메시지 변수들
        ad_msgs::msg::VehicleInput o_vehicle_command_;
        ad_msgs::msg::PolyfitLaneData o_driving_way_;
        ad_msgs::msg::PolyfitLaneDataArray o_poly_lanes_;
        
        // 시스템 파라미터들
        double time_prev_csv_lanes_;
        bool param_is_simulator_on_ = false;
        std::string param_vehicle_namespace_;
        bool param_use_manual_inputs_ = false;

        // 차량 물리 파라미터
        const double param_wheel_base_ = 1.302 + 1.398; // 앞뒤 바퀴 간격(m)
        const double param_max_lateral_accel_ = 6200.0 / 1319.91; // 최대 횡방향 가속도

        // 제어 튜닝 파라미터들
        double param_pp_kd_ = 1.0;    // Pure Pursuit 거리 게인
        double param_pp_kv_ = 0.0;    // Pure Pursuit 속도 게인
        double param_pp_kc_ = 0.0;    // Pure Pursuit 곡률 게인
        double param_pid_kp_ = 0.0;   // PID 비례 게인
        double param_pid_ki_ = 0.0;   // PID 적분 게인
        double param_pid_kd_ = 0.0;   // PID 미분 게인
        double param_brake_ratio_ = 1.0; // 브레이크 비율

        // ROI(Region of Interest) 파라미터
        double param_m_ROIFront_param = 20.0;  // 전방 ROI 거리
        double param_m_ROIRear_param = 10.0;   // 후방 ROI 거리
        double param_m_ROILeft_param = 3.0;    // 좌측 ROI 거리
        double param_m_ROIRight_param = 3.0;   // 우측 ROI 거리
        std::string param_ref_csv_path;

        // [신규] PID 제어를 위한 변수들
        double e = 0.0;          // 현재 오차
        double e_prev = 0.0;     // 이전 오차
        double int_e = 0.0;      // 적분 오차
        double int_e_prev = 0.0; // 이전 적분 오차
        double dev_e = 0.0;      // 미분 오차
        double computed_input = 0.0; // PID 계산 결과
        double input = 0.0;      // 최종 입력값
        rclcpp::Time pid_last_time = this->now(); // PID 마지막 실행 시간
};

#endif // __AUTONOMOUS_DRIVING_HPP__
