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
 */

/**
 * @file      autonomous_driving.cpp
 * @brief     자율주행 알고리즘 구현 파일
 *            PID 기반 속도 제어와 Pure Pursuit 기반 조향 제어를 포함
 */

#include "autonomous_driving.hpp"

/**
 * @brief 생성자: ROS2 노드 초기화 및 설정
 * @param node_name 노드 이름
 * @param loop_rate 실행 주기 (Hz)
 * @param options ROS2 노드 옵션
 */
AutonomousDriving::AutonomousDriving(const std::string &node_name, const double &loop_rate,
                                     const rclcpp::NodeOptions &options)
    : Node(node_name, options) {

    RCLCPP_WARN(this->get_logger(), "Initialize node...");

    // QoS 프로파일 설정 - 최근 10개 메시지 유지
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // 파라미터 선언 및 초기화
    // 차량 네임스페이스 설정
    this->declare_parameter("autonomous_driving/ns", "");
    if (!this->get_parameter("autonomous_driving/ns", param_vehicle_namespace_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get vehicle_namespace");
        param_vehicle_namespace_ = "";
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_vehicle_namespace_: %s", param_vehicle_namespace_.c_str());
    }

    // 수동 입력 사용 여부 설정
    this->declare_parameter("autonomous_driving/use_manual_inputs", false);
    if (!this->get_parameter("autonomous_driving/use_manual_inputs", param_use_manual_inputs_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get use_manual_inputs");
        param_use_manual_inputs_ = true;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "param_use_manual_inputs_: %d", param_use_manual_inputs_);
    }

    // Pure Pursuit 제어 파라미터 설정
    // kd: 조향각 계산을 위한 거리 게인
    this->declare_parameter("autonomous_driving/pure_pursuit_kd", 1.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kd", param_pp_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kd");
        param_pp_kd_ = 1.0;
    }
    
    // kv: 속도에 따른 조향 보정 게인
    this->declare_parameter("autonomous_driving/pure_pursuit_kv", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kv", param_pp_kv_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kv");
        param_pp_kv_ = 0.0;
    }

    // kc: 곡률에 따른 조향 보정 게인
    this->declare_parameter("autonomous_driving/pure_pursuit_kc", 0.0);
    if (!this->get_parameter("autonomous_driving/pure_pursuit_kc", param_pp_kc_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pure_pursuit_kc");
        param_pp_kc_ = 0.0;
    }

    // PID 제어 파라미터 설정
    // Kp: 비례 게인
    this->declare_parameter("autonomous_driving/pid_kp", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kp", param_pid_kp_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kp");
        param_pid_kp_ = 0.0;
    }

    // Ki: 적분 게인
    this->declare_parameter("autonomous_driving/pid_ki", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_ki", param_pid_ki_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_ki");
        param_pid_ki_ = 0.0;
    }

    // Kd: 미분 게인
    this->declare_parameter("autonomous_driving/pid_kd", 0.0);
    if (!this->get_parameter("autonomous_driving/pid_kd", param_pid_kd_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pid_kd");
        param_pid_kd_ = 0.0;
    }

    // 브레이크 비율 설정
    this->declare_parameter("autonomous_driving/brake_ratio", 1.0);
    if (!this->get_parameter("autonomous_driving/brake_ratio", param_brake_ratio_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get brake_ratio");
        param_brake_ratio_ = 0.0;
    }

    // ROI(Region of Interest) 파라미터 설정
    this->declare_parameter("autonomous_driving/ROIFront", 20.0);
    if (!this->get_parameter("autonomous_driving/ROIFront", param_m_ROIFront_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROIFront");
        param_m_ROIFront_param = 20.0;
    }

    this->declare_parameter("autonomous_driving/ROIRear", 10.0);
    if (!this->get_parameter("autonomous_driving/ROIRear", param_m_ROIRear_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROIRear");
        param_m_ROIRear_param = 10.0;
    }

    this->declare_parameter("autonomous_driving/ROILeft", 3.0);
    if (!this->get_parameter("autonomous_driving/ROILeft", param_m_ROILeft_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROILeft");
        param_m_ROILeft_param = 3.0;
    }

    this->declare_parameter("autonomous_driving/ROIRight", 3.0);
    if (!this->get_parameter("autonomous_driving/ROIRight", param_m_ROIRight_param)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get ROIRight");
        param_m_ROIRight_param = 3.0;
    }

    // PID 제어기 변수 초기화
    double e = 0.0;          // 현재 오차
    double e_prev = 0.0;     // 이전 오차
    double int_e = 0.0;      // 적분 오차
    double int_e_prev = 0.0; // 이전 적분 오차
    double dev_e = 0.0;      // 미분 오차
    double computed_input = 0.0; // 계산된 제어 입력
    double input = 0.0;      // 최종 제어 입력
    rclcpp::Time pid_last_time = this->now();  // PID 마지막 실행 시간

    // Subscriber 설정
    // 수동 입력 구독
    s_manual_input_ = this->create_subscription<ad_msgs::msg::VehicleInput>(
        "/manual_input", qos_profile, std::bind(&AutonomousDriving::CallbackManualInput, this, std::placeholders::_1));
    
    // 차량 상태 구독
    s_vehicle_state_ = this->create_subscription<ad_msgs::msg::VehicleOutput>(
        "vehicle_state", qos_profile, std::bind(&AutonomousDriving::CallbackVehicleState, this, std::placeholders::_1));
    
    // 차선 포인트 구독
    s_lane_points_array_ = this->create_subscription<ad_msgs::msg::LanePointDataArray>(
        "lane_points_array", qos_profile, std::bind(&AutonomousDriving::CallbackLanePointsArray, this, std::placeholders::_1));

    // Publisher 설정
    // 차량 제어 명령 발행
    p_vehicle_command_ = this->create_publisher<ad_msgs::msg::VehicleInput>(
        "vehicle_command", qos_profile);
    
    // 주행 경로 발행
    p_driving_way_ = this->create_publisher<ad_msgs::msg::PolyfitLaneData>(
        "driving_way", qos_profile);
    
    // 차선 정보 발행
    p_poly_lanes_ = this->create_publisher<ad_msgs::msg::PolyfitLaneDataArray>(
        "poly_lanes", qos_profile);

    // 노드 초기화
    Init(this->now());

    // 타이머 설정 - 주기적 실행
    t_run_node_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)(1000 / loop_rate)),
        [this](){ this->Run(this->now()); });
}

// 소멸자
AutonomousDriving::~AutonomousDriving() {}

// 초기화 함수
void AutonomousDriving::Init(const rclcpp::Time &current_time) {
}

// 파라미터 업데이트 함수
void AutonomousDriving::UpdateParameter() {
}

/**
 * @brief 메인 실행 함수
 * @param current_time 현재 시간
 */
void AutonomousDriving::Run(const rclcpp::Time &current_time) {
    // 파라미터 업데이트
    UpdateParameter();

    // 구독한 데이터 가져오기
    mutex_vehicle_state_.lock();
    ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
    mutex_vehicle_state_.unlock();

    mutex_lane_points_.lock();
    ad_msgs::msg::LanePointDataArray lane_points_array = i_lane_points_array_;
    mutex_lane_points_.unlock();

    // 최대 속도 설정 (50km/h)
    double limit_speed = 50 / 3.6; // [mps]

    // 출력 변수 초기화
    ad_msgs::msg::PolyfitLaneDataArray poly_lanes;
    poly_lanes.frame_id = param_vehicle_namespace_ + "/body";

    ad_msgs::msg::PolyfitLaneData driving_way;
    driving_way.frame_id = param_vehicle_namespace_ + "/body";
    
    ad_msgs::msg::VehicleInput vehicle_command;

    // 자율주행 모드일 경우
    if (param_use_manual_inputs_ == false) {
        // 좌/우 차선 포인트 저장 벡터
        std::vector<double> lane_points_left_x;
        std::vector<double> lane_points_left_y;
        std::vector<double> lane_points_right_x;
        std::vector<double> lane_points_right_y;

        // 차선 포인트 분리 (좌/우)
        for (const auto &lane_point : lane_points_array.lane) {
            if (lane_point.id == "1") {  // 왼쪽 차선
                for (const auto &point : lane_point.point) {
                    lane_points_left_x.push_back(point.x);
                    lane_points_left_y.push_back(point.y);
                }
            } else if (lane_point.id == "2") {  // 오른쪽 차선
                for (const auto &point : lane_point.point) {
                    lane_points_right_x.push_back(point.x);
                    lane_points_right_y.push_back(point.y);
                }
            }
        }

        // 3차 다항식 피팅을 위한 행렬 설정
        const int Num_of_state = 4;  // 3차 다항식 계수의 수
        Eigen::MatrixXd H_left(lane_points_left_x.size(), Num_of_state);
        Eigen::MatrixXd Y_left(lane_points_left_y.size(), 1);
        Eigen::MatrixXd H_right(lane_points_right_x.size(), Num_of_state);
        Eigen::MatrixXd Y_right(lane_points_right_y.size(), 1);

        // 왼쪽 차선 행렬 구성
        for (int i = 0; i < lane_points_left_x.size(); ++i) {
            double x = lane_points_left_x[i];
            H_left(i, 0) = 1.0;                    // 상수항
            H_left(i, 1) = x;                      // 1차항
            H_left(i, 2) = x * x;                  // 2차항
            H_left(i, 3) = x * x * x;              // 3차항
            Y_left(i, 0) = lane_points_left_y[i];  // y 좌표
        }

        // 오른쪽 차선 행렬 구성
        for (int i = 0; i < lane_points_right_x.size(); ++i) {
            double x = lane_points_right_x[i];
            H_right(i, 0) = 1.0;
            H_right(i, 1) = x;
            H_right(i, 2) = x * x;
            H_right(i, 3) = x * x * x;
            Y_right(i, 0) = lane_points_right_y[i];
        }

        // 최소자승법을 이용한 다항식 계수 계산
        Eigen::VectorXd poly_lanes_left = (H_left.transpose() * H_left).inverse() * H_left.transpose() * Y_left;
        Eigen::VectorXd poly_lanes_right = (H_right.transpose() * H_right).inverse() * H_right.transpose() * Y_right;

        // 차선 정보 저장
        ad_msgs::msg::PolyfitLaneData left_lane;
        ad_msgs::msg::PolyfitLaneData right_lane;

// 왼쪽 차선 정보 설정
        left_lane.id = "1";
        left_lane.frame_id = param_vehicle_namespace_ + "/body";
        left_lane.a3 = poly_lanes_left(3);  // 3차항 계수
        left_lane.a2 = poly_lanes_left(2);  // 2차항 계수
        left_lane.a1 = poly_lanes_left(1);  // 1차항 계수
        left_lane.a0 = poly_lanes_left(0);  // 상수항 계수

        // 오른쪽 차선 정보 설정
        right_lane.id = "2";
        right_lane.frame_id = param_vehicle_namespace_ + "/body";
        right_lane.a3 = poly_lanes_right(3);
        right_lane.a2 = poly_lanes_right(2);
        right_lane.a1 = poly_lanes_right(1);
        right_lane.a0 = poly_lanes_right(0);

        // 차선 정보 저장
        poly_lanes.polyfitlanes.push_back(left_lane);
        poly_lanes.polyfitlanes.push_back(right_lane);

        // 중앙선(driving way) 계산 - 좌우 차선의 평균
        driving_way.a3 = (poly_lanes_left(3) + poly_lanes_right(3)) / 2.0;
        driving_way.a2 = (poly_lanes_left(2) + poly_lanes_right(2)) / 2.0;
        driving_way.a1 = (poly_lanes_left(1) + poly_lanes_right(1)) / 2.0;
        driving_way.a0 = (poly_lanes_left(0) + poly_lanes_right(0)) / 2.0;
        
        // PID 기반 속도 제어 실행
        PID(current_time, limit_speed, vehicle_state.velocity);

        // 가속/제동 명령 생성
        if (computed_input >= 0.0) {
            vehicle_command.accel = computed_input;    // 양수면 가속
            vehicle_command.brake = 0;
        } else {
            vehicle_command.accel = 0;
            vehicle_command.brake = -computed_input * param_brake_ratio_;  // 음수면 제동
        }
        
        // Pure Pursuit 기반 조향각 계산
        double x1 = param_wheel_base_ / 2;     // 현재 위치
        double x2 = 10.0;                      // 목표점까지의 거리 (look-ahead distance)
        double y1 = 0.0;                       // 현재 위치 y좌표
        // 목표점의 y좌표 계산 (3차 다항식)
        double y2 = driving_way.a3 * pow(x2, 3) + driving_way.a2 * pow(x2, 2) + 
                   driving_way.a1 * x2 + driving_way.a0;
        // 조향각 계산
        double sin_alpha = atan((y2 - y1)/(x2 - x1));  // 목표점까지의 각도
        double delta_radian = atan(2 * param_wheel_base_ * sin_alpha / x2);  // 조향각
        vehicle_command.steering = delta_radian;
    }

    // 출력값 업데이트
    o_driving_way_ = driving_way;
    o_poly_lanes_ = poly_lanes;
    o_vehicle_command_ = vehicle_command;

    // 결과 발행
    Publish(current_time);
}

/**
 * @brief 계산된 제어 명령 발행
 * @param current_time 현재 시간
 */
void AutonomousDriving::Publish(const rclcpp::Time &current_time) {
    p_vehicle_command_->publish(o_vehicle_command_);  // 차량 제어 명령 발행
    p_driving_way_->publish(o_driving_way_);         // 주행 경로 발행
    p_poly_lanes_->publish(o_poly_lanes_);           // 차선 정보 발행
}

/**
 * @brief PID 제어기 구현
 * @param current_time 현재 시간
 * @param target_value 목표값 (목표 속도)
 * @param current_value 현재값 (현재 속도)
 */
void AutonomousDriving::PID(const rclcpp::Time &current_time, double target_value, double current_value) {
    // 시간 간격 계산
    double dt = (current_time - pid_last_time).seconds();
    if (dt <= 0.0) {
        dt = 0.01;  // 최소 시간 간격 보장
    }

    // 현재 오차 계산
    e = target_value - current_value;

    // 적분 오차 계산 (사다리꼴 적분)
    int_e = int_e_prev + (dt * (e + e_prev) / 2.0);

    // 미분 오차 계산
    dev_e = (e - e_prev) / dt;

    // PID 제어 출력 계산
    computed_input = param_pid_kp_ * e +          // 비례항
                    param_pid_ki_ * int_e +       // 적분항
                    param_pid_kd_ * dev_e;        // 미분항

    // 이전 값 저장
    e_prev = e;
    int_e_prev = int_e;
    pid_last_time = current_time;
}

/**
 * @brief 메인 함수
 */
int main(int argc, char **argv) {
    std::string node_name = "autonomous_driving";  // 노드 이름 설정
    double loop_rate = 100.0;  // 실행 주기 설정 (100Hz)

    // ROS2 노드 초기화 및 실행
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousDriving>(node_name, loop_rate));
    rclcpp::shutdown();
    return 0;
}
