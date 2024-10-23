/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved.
 *            Subject to limited distribution and restricted disclosure only.
 *
 * @file      autonomous_driving.cpp
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

#include "autonomous_driving.hpp"

// Constructor for autonomous driving node (자율주행 노드 생성자)
AutonomousDriving::AutonomousDriving(const std::string &node_name, const double &loop_rate,
                                     const rclcpp::NodeOptions &options)
    : Node(node_name, options) {    // Initialize base Node class (기본 Node 클래스 초기화)

    // Node initialization warning message (노드 초기화 경고 메시지)
    RCLCPP_WARN(this->get_logger(), "Initialize node...");

    // Quality of Service settings (통신 품질 설정)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));  // Keep last 10 messages (최근 10개 메시지 유지)

    // Parameter declarations and initialization (파라미터 선언 및 초기화)
    // Vehicle namespace parameter (차량 네임스페이스 파라미터)
    this->declare_parameter("autonomous_driving/ns", "");
    if (!this->get_parameter("autonomous_driving/ns", param_vehicle_namespace_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle_namespace");
        param_vehicle_namespace_ = "";
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_vehicle_namespace_: %s", param_vehicle_namespace_.c_str());
    }

    // Manual control mode parameter (수동 제어 모드 파라미터)
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);
    if (!this->get_parameter("autonomous_driving/use_manual_inputs", param_use_manual_inputs_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get use_manual_inputs");
        param_use_manual_inputs_ = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_use_manual_inputs_: %d", param_use_manual_inputs_);
    }

    // Pure Pursuit controller parameters (Pure Pursuit 제어기 파라미터)
    // Distance gain (거리 게인)
    this->declare_parameter("autonomous_driving/pure_pursuit_kd", 1.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kd", param_pp_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kd");
        param_pp_kd_ = 1.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pp_kd_: %f", param_pp_kd_);
    }

    // Velocity gain (속도 게인)
    this->declare_parameter("autonomous_driving/pure_pursuit_kv", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kv", param_pp_kv_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kv");
        param_pp_kv_ = 0.0;
    }

    // Curvature gain (곡률 게인)
    this->declare_parameter("autonomous_driving/pure_pursuit_kc", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kc", param_pp_kc_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kc");
        param_pp_kc_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pp_kv_: %f", param_pp_kv_);
    }

    // PID controller parameters (PID 제어기 파라미터)
    // Proportional gain (비례 게인)
    this->declare_parameter("autonomous_driving/pid_kp", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kp", param_pid_kp_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kp");
        param_pid_kp_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_kp_: %f", param_pid_kp_);
    }

    // Integral gain (적분 게인)
    this->declare_parameter("autonomous_driving/pid_ki", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_ki", param_pid_ki_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_ki");
        param_pid_ki_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_ki_: %f", param_pid_ki_);
    }

    // Derivative gain (미분 게인)
    this->declare_parameter("autonomous_driving/pid_kd", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kd", param_pid_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kd");
        param_pid_kd_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_pid_kd_: %f", param_pid_kd_);
    }

    // Brake ratio parameter (제동 비율 파라미터)
    this->declare_parameter("autonomous_driving/brake_ratio", 1.0);
    if (!this->get_parameter("autonomous_driving/brake_ratio", param_brake_ratio_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get brake_ratio");
        param_brake_ratio_ = 0.0;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_brake_ratio_: %f", param_brake_ratio_);
    }

    // Initialize subscribers (구독자 초기화)
    // Manual input subscriber (수동 입력 구독자)
    s_manual_input_ = this->create_subscription<ad_msgs::msg::VehicleInput>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    
    // Vehicle state subscriber (차량 상태 구독자)
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    
    // Speed limit subscriber (속도 제한 구독자)
    s_limit_speed_ = this->create_subscription<std_msgs::msg::Float32>(
        "limit_speed", qos_profile, std::bind(&AutonomousDriving::CallbackLimitSpeed, this, std::placeholders::_1));
    
    // Lane points subscriber (차선 포인트 구독자)
    s_lane_points_ = this->create_subscription<ad_msgs::msg::LanePointData>(
        "lane_points", qos_profile, std::bind(&AutonomousDriving::CallbackLanePoints, this, std::placeholders::_1));
    
    // Obstacles subscriber (장애물 구독자)
    s_obstacles_ = this->create_subscription<ad_msgs::msg::Obstacles>(
        "obstacles", qos_profile, std::bind(&AutonomousDriving::CallbackObstacles, this, std::placeholders::_1));

    // Initialize publishers (발행자 초기화)
    // Vehicle command publisher (차량 명령 발행자)
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleInput>(
        "vehicle_command", qos_profile);
    
    // Driving path publisher (주행 경로 발행자)
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    
    // Lane polynomials publisher (차선 다항식 발행자)
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

    // Initialize node state (노드 상태 초기화)
    Init(this->now());

    // Initialize timer for periodic execution (주기적 실행을 위한 타이머 초기화)
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this]()
        { this->Run(this->now()); });
}

// Destructor (소멸자)
AutonomousDriving::~AutonomousDriving() {}

// Initialization function (초기화 함수)
void AutonomousDriving::Init(const rclcpp::Time &current_time) {
    // Additional initialization if needed (필요한 경우 추가 초기화)
}

// Parameter update function (파라미터 업데이트 함수)
void AutonomousDriving::UpdateParameter() {
    // Update dynamic parameters if needed (필요한 경우 동적 파라미터 업데이트)
}

// Main execution loop (메인 실행 루프)
void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    UpdateParameter();

    // Get subscribed data with mutex protection (뮤텍스 보호와 함께 구독 데이터 가져오기)
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_obstacles_.lock();
    ad_msgs::msg::Obstacles obstacles = i_obstacles_;
    mutex_obstacles_.unlock();

    mutex_limit_speed_.lock();
    double limit_speed = i_limit_speed_;
    mutex_limit_speed_.unlock();

    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointData lane_points = i_lane_points_;
    mutex_lane_points_.unlock();

    // Initialize output messages (출력 메시지 초기화)
    ad_msgs::msg::PolyfitLaneData driving_way;
    driving_way.frame_id = param_vehicle_namespace_ + "/body";

    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    poly_lanes.frame_id = param_vehicle_namespace_ + "/body";

    ad_msgs::msg::VehicleInput vehicle_command;

    // Autonomous driving algorithm (자율 주행 알고리즘)
    if (param_use_manual_inputs_ == false) {
        // Separate lane points into left and right (차선 포인트를 좌/우로 분리)
        std::vector<double> lane_points_left_x;
        std::vector<double> lane_points_left_y;
        std::vector<double> lane_points_right_x;
        std::vector<double> lane_points_right_y;

        // Classify points based on y coordinate (y 좌표 기준으로 포인트 분류)
        for (int i = 0; i < lane_points.point.size(); i++) {
            if (lane_points.point[i].y >= 0) {  // Left lane (왼쪽 차선)
                lane_points_left_x.push_back(lane_points.point[i].x);
                lane_points_left_y.push_back(lane_points.point[i].y);
            } else if (lane_points.point[i].y < 0) {  // Right lane (오른쪽 차선)
                lane_points_right_x.push_back(lane_points.point[i].x);
                lane_points_right_y.push_back(lane_points.point[i].y);
            }
        }       

        // Lane fitting using cubic polynomial (3차 다항식을 이용한 차선 피팅)
        const int Num_of_state = 4;  // Cubic polynomial degree (3차 다항식 차수)
        // Matrix setup for least squares fitting (최소자승법을 위한 행렬 설정)
        Eigen::MatrixXd H_left(lane_points_left_x.size(), Num_of_state);
        Eigen::MatrixXd Y_left(lane_points_left_y.size(), 1);
        Eigen::MatrixXd H_right(lane_points_right_x.size(), Num_of_state);
        Eigen::MatrixXd Y_right(lane_points_right_y.size(), 1);

        // Build matrices for left lane (왼쪽 차선 행렬 구성)
        for (int i = 0; i < lane_points_left_x.size(); ++i) {
            double x = lane_points_left_x[i];
            H_left(i, 0) = 1.0;          // Constant term (상수항)
            H_left(i, 1) = x;            // Linear term (1차항)
            H_left(i, 2) = x * x;        // Quadratic term (2차항)
            H_left(i, 3) = x * x * x;    // Cubic term (3차항)
            Y_left(i, 0) = lane_points_left_y[i];
        }

        // Build matrices for right lane (오른쪽 차선 행렬 구성)
        for (int i = 0; i < lane_points_right_x.size(); ++i) {
            double x = lane_points_right_x[i];
            H_right(i, 0) = 1.0;
            H_right(i, 1) = x;
            H_right(i, 2) = x * x;
            H_right(i, 3) = x * x * x;
            Y_right(i, 0) = lane_points_right_y[i];
        }

        // Solve least squares problem (최소자승법 문제 해결)
        // Normal equation: (H^T * H)^(-1) * H^T * Y (정규 방정식: (H^T * H)^(-1) * H^T * Y)
        Eigen::VectorXd poly_lanes_left = (H_left.transpose() * H_left).inverse() * H_left.transpose() * Y_left;
        Eigen::VectorXd poly_lanes_right = (H_right.transpose() * H_right).inverse() * H_right.transpose() * Y_right;

        // Create lane polynomial messages (차선 다항식 메시지 생성)
        ad_msgs::msg::PolyfitLaneData left_lane;
        ad_msgs::msg::PolyfitLaneData right_lane;

        // Set left lane coefficients (왼쪽 차선 계수 설정)
        left_lane.id = "1";
        left_lane.frame_id = param_vehicle_namespace_ + "/body";
        left_lane.a3 = poly_lanes_left(3);  // Cubic coefficient (3차 계수)
        left_lane.a2 = poly_lanes_left(2);  // Quadratic coefficient (2차 계수)
        left_lane.a1 = poly_lanes_left(1);  // Linear coefficient (1차 계수)
        left_lane.a0 = poly_lanes_left(0);  // Constant term (상수항)

        // Set polynomial coefficients for right lane (오른쪽 차선의 다항식 계수 설정)
        right_lane.id = "2";
        right_lane.frame_id = param_vehicle_namespace_ + "/body";
        right_lane.a3 = poly_lanes_right(3);
        right_lane.a2 = poly_lanes_right(2);
        right_lane.a1 = poly_lanes_right(1);
        right_lane.a0 = poly_lanes_right(0);

        // Store lane polynomials for visualization (시각화를 위한 차선 다항식 저장)
        poly_lanes.polyfitlanes.push_back(left_lane);
        poly_lanes.polyfitlanes.push_back(right_lane);

        // Calculate center line as average of left and right lanes (좌우 차선의 평균으로 중심선 계산)
        // Center line will be the driving path (중심선이 주행 경로가 됨)
        driving_way.a3 = (poly_lanes_left(3) + poly_lanes_right(3)) / 2.0;  // Average cubic term (3차항 평균)
        driving_way.a2 = (poly_lanes_left(2) + poly_lanes_right(2)) / 2.0;  // Average quadratic term (2차항 평균)
        driving_way.a1 = (poly_lanes_left(1) + poly_lanes_right(1)) / 2.0;  // Average linear term (1차항 평균)
        driving_way.a0 = (poly_lanes_left(0) + poly_lanes_right(0)) / 2.0;  // Average constant term (상수항 평균)

        // Longitudinal control - Speed control (종방향 제어 - 속도 제어)
        double obstacle_speed = limit_speed;  // Default to speed limit (기본값은 제한 속도)
        double min_distance = std::numeric_limits<double>::max();

        // Find closest obstacle and its speed (가장 가까운 장애물과 속도 찾기)
        for (const auto& obstacle : obstacles.obstacles) {
            // Calculate distance to obstacle (장애물까지의 거리 계산)
            double distance = std::sqrt(obstacle.x * obstacle.x + obstacle.y * obstacle.y);

            if (distance < min_distance) {
                min_distance = distance;
                obstacle_speed = obstacle.velocity;  // Update target speed based on obstacle (장애물 기준으로 목표 속도 갱신)
            }
        }

        // Determine target speed considering obstacles (장애물을 고려한 목표 속도 결정)
        double target_speed;
        if (obstacles.obstacles.empty()) {
            target_speed = limit_speed;  // No obstacles, use speed limit (장애물 없음, 제한 속도 사용)
        } else {
            target_speed = std::min(limit_speed, obstacle_speed);  // Use lower speed (더 낮은 속도 사용)
        }
        
        // Apply PID control for speed (속도에 대한 PID 제어 적용)
        PID(current_time, target_speed, vehicle_state.velocity);

        // Convert PID output to acceleration/brake commands (PID 출력을 가속/제동 명령으로 변환)
        if (computed_input >= 0.0) {
            vehicle_command.accel = computed_input;  // Positive input for acceleration (양의 입력은 가속)
            vehicle_command.brake = 0;
        } else {
            vehicle_command.accel = 0;
            vehicle_command.brake = -computed_input * param_brake_ratio_;  // Negative input for braking (음의 입력은 제동)
        }
        
        // Lateral control - Pure Pursuit steering control (횡방향 제어 - Pure Pursuit 조향 제어)
        double x1 = param_wheel_base_ / 2;  // Current position (현재 위치)
        double x2 = 5.0;  // Look-ahead distance (전방 주시 거리)
        double y1 = 0.0;  // Current lateral position (현재 횡방향 위치)
        
        // Calculate target point on path (경로상의 목표 지점 계산)
        double y2 = driving_way.a3 * pow(x2, 3) + driving_way.a2 * pow(x2, 2) + 
                   driving_way.a1 * x2 + driving_way.a0;
        
        // Pure Pursuit steering angle calculation (Pure Pursuit 조향각 계산)
        double sin_alpha = sin(atan((y2 - y1)/(x2 - x1)));  // Path angle (경로각)
        double delta_radian = atan(2 * param_wheel_base_ * sin_alpha / x2);  // Steering angle (조향각)
        vehicle_command.steering = delta_radian;
    }

    // Update output variables (출력 변수 업데이트)
    o_driving_way_ = driving_way;      // Update planned path (계획된 경로 업데이트)
    o_poly_lanes_ = poly_lanes;        // Update detected lanes (감지된 차선 업데이트)
    o_vehicle_command_ = vehicle_command;  // Update control commands (제어 명령 업데이트)

    // Publish outputs (출력 발행)
    Publish(current_time);
}

// Publishing function (발행 함수)
void AutonomousDriving::Publish(const rclcpp::Time &current_time) {
    p_vehicle_command_->publish(o_vehicle_command_);  // Publish control commands (제어 명령 발행)
    p_driving_way_->publish(o_driving_way_);         // Publish planned path (계획된 경로 발행)
    p_poly_lanes_->publish(o_poly_lanes_);           // Publish detected lanes (감지된 차선 발행)
}

/**
 * PID Controller implementation (PID 제어기 구현)
 * target_value: Desired speed (목표 속도)
 * current_value: Current speed (현재 속도)
 */
void AutonomousDriving::PID(const rclcpp::Time &current_time, double target_value, double current_value) {
    // Calculate time difference (시간 차이 계산)
    double dt = (current_time - pid_last_time).seconds();

    if (dt <= 0.0) {
        dt = 0.01;  // Minimum time step (최소 시간 간격)
    }

    // Calculate error (오차 계산)
    e = target_value - current_value;

    // Calculate integral term using trapezoidal rule (사다리꼴 법칙을 이용한 적분항 계산)
    int_e = int_e_prev + (dt * (e + e_prev) / 2.0);

    // Calculate derivative term (미분항 계산)
    dev_e = (e - e_prev) / dt;

    // Calculate control input using PID formula (PID 공식을 이용한 제어 입력 계산)
    computed_input = param_pid_kp_ * e +          // Proportional term (비례항)
                    param_pid_ki_ * int_e +       // Integral term (적분항)
                    param_pid_kd_ * dev_e;        // Derivative term (미분항)

    // Update previous values (이전 값 업데이트)
    e_prev = e;
    int_e_prev = int_e;
    pid_last_time = current_time;
}

// Main function (메인 함수)
int main(int argc, char **argv) {
    std::string node_name = "autonomous_driving";
    double loop_rate = 100.0;  // 100Hz control rate (100Hz 제어 주기)

    // Initialize ROS2 (ROS2 초기화)
    rclcpp::init(argc, argv);
    // Create and spin node (노드 생성 및 실행)
    rclcpp::spin(std::make_shared<AutonomousDriving>(node_name, loop_rate));
    // Cleanup ROS2 (ROS2 정리)
    rclcpp::shutdown();
    return 0;
}
