# 자율주행 시스템 구현 분석

## 1. 시스템 구조

### 1.1 ROS2 Node 구조
- Node Name: "autonomous_driving"
- 실행 주기: 100Hz
```cpp
std::string node_name = "autonomous_driving";
double loop_rate = 100.0;  // 100Hz control rate
```

### 1.2 주요 통신 구조
- Publisher/Subscriber 패턴 사용
- Topics:
  - 입력(Input):
    - `/manual_input`: 수동 제어 입력
    - `vehicle_state`: 차량 상태
    - `limit_speed`: 속도 제한
    - `lane_points`: 차선 포인트
    - `obstacles`: 장애물 정보
  - 출력(Output):
    - `vehicle_command`: 차량 제어 명령
    - `driving_way`: 주행 경로
    - `poly_lanes`: 인식된 차선

## 2. 차선 인식 및 경로 생성

### 2.1 차선 포인트 분류
- y 좌표 기준으로 좌/우 차선 분류
```cpp
if (lane_points.point[i].y >= 0) {  // Left lane
    lane_points_left_x.push_back(lane_points.point[i].x);
    lane_points_left_y.push_back(lane_points.point[i].y);
} else {  // Right lane
    lane_points_right_x.push_back(lane_points.point[i].x);
    lane_points_right_y.push_back(lane_points.point[i].y);
}
```

### 2.2 차선 피팅 (Lane Fitting)
- 최소자승법(Least Squares Method) 사용
- 3차 다항식(Cubic Polynomial) 피팅
```cpp
// Matrix form: y = a₃x³ + a₂x² + a₁x + a₀
Eigen::MatrixXd H_left(lane_points_left_x.size(), 4);
H_left(i, 0) = 1.0;          // 상수항
H_left(i, 1) = x;            // 1차항
H_left(i, 2) = x * x;        // 2차항
H_left(i, 3) = x * x * x;    // 3차항

// Normal equation solution
poly_lanes_left = (H_left.transpose() * H_left).inverse() * H_left.transpose() * Y_left;
```

### 2.3 주행 경로 생성
- 좌우 차선의 중간점을 주행 경로로 설정
```cpp
driving_way.a3 = (poly_lanes_left(3) + poly_lanes_right(3)) / 2.0;
driving_way.a2 = (poly_lanes_left(2) + poly_lanes_right(2)) / 2.0;
driving_way.a1 = (poly_lanes_left(1) + poly_lanes_right(1)) / 2.0;
driving_way.a0 = (poly_lanes_left(0) + poly_lanes_right(0)) / 2.0;
```

## 3. 차량 제어

### 3.1 종방향(Longitudinal) 제어
- PID 제어기 구현
- 속도 제어를 위한 가속/제동 명령 생성
```cpp
// PID control implementation
void PID(const rclcpp::Time &current_time, double target_value, double current_value) {
    e = target_value - current_value;  // Error
    int_e = int_e_prev + (dt * (e + e_prev) / 2.0);  // Integral
    dev_e = (e - e_prev) / dt;  // Derivative
    
    computed_input = param_pid_kp_ * e +          // P term
                    param_pid_ki_ * int_e +       // I term
                    param_pid_kd_ * dev_e;        // D term
}
```

### 3.2 횡방향(Lateral) 제어
- Pure Pursuit 알고리즘 사용
- Look-ahead distance 기반 조향각 계산
```cpp
// Pure Pursuit steering control
double x2 = 5.0;  // Look-ahead distance
double y2 = driving_way.a3 * pow(x2, 3) + driving_way.a2 * pow(x2, 2) + 
           driving_way.a1 * x2 + driving_way.a0;
           
double sin_alpha = sin(atan((y2 - y1)/(x2 - x1)));
double delta_radian = atan(2 * param_wheel_base_ * sin_alpha / x2);
```

## 4. 안전 기능

### 4.1 장애물 회피
- 가장 가까운 장애물 기준 속도 조절
```cpp
double distance = std::sqrt(obstacle.x * obstacle.x + obstacle.y * obstacle.y);
if (distance < min_distance) {
    min_distance = distance;
    obstacle_speed = obstacle.velocity;
}
target_speed = std::min(limit_speed, obstacle_speed);
```

### 4.2 데이터 동기화
- 뮤텍스(Mutex)를 이용한 스레드 안전성 보장
```cpp
mutex_vehicle_state_.lock();
ad_msgs::msg::VehicleOutput vehicle_state = i_vehicle_state_;
mutex_vehicle_state_.unlock();
```

## 5. 주요 제어 파라미터

### 5.1 PID 제어 파라미터
- `pid_kp_`: 비례 게인(Proportional Gain)
- `pid_ki_`: 적분 게인(Integral Gain)
- `pid_kd_`: 미분 게인(Derivative Gain)

### 5.2 Pure Pursuit 파라미터
- `param_pp_kd_`: 거리 게인(Distance Gain)
- `param_pp_kv_`: 속도 게인(Velocity Gain)
- `param_pp_kc_`: 곡률 게인(Curvature Gain)
- `param_wheel_base_`: 차량 축거(Wheelbase)

## 6. 시스템 제한사항

1. 차선 인식:
   - 3차 다항식 모델 사용으로 복잡한 곡선에서 정확도 제한
   - y좌표 기반 차선 분류로 급격한 곡선에서 오차 발생 가능

2. 제어:
   - 고정된 Look-ahead distance 사용으로 다양한 주행 상황 대응 제한
   - 단순 PID 제어로 인한 비선형 동역학 대응 한계

3. 안전:
   - 단순 거리 기반 장애물 회피로 복잡한 상황 대응 제한
   - 고정된 제어 주기로 인한 실시간성 제약
