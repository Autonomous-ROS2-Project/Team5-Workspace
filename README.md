# 🚗 Team 5 - Autonomous Driving System

## 👥 Team Members
**Department of Future Automotive Engineering (미래자동차공학과)**

| Role | Name | GitHub |
|------|------|--------|
| Team Leader | Jisang Yun (윤지상) | [@thejourneyofbabo] |
| Team Member | Byeongheon Lee (이병현) | [@byeongheon-github-id] |
| Team Member | Jonguk Baek (백종욱) | [@jonguk-github-id] |

## 📌 Project Overview
ROS2 기반의 자율주행 시스템 구현 프로젝트입니다. 차선 인식, 장애물 회피, 경로 계획 및 차량 제어를 통합적으로 구현하여 안전하고 효율적인 자율주행을 목표로 합니다.

## 🛠 Technical Stack
- **Framework**: ROS2 (Robot Operating System 2)
- **Language**: C++
- **Key Libraries**: 
  - Eigen (선형대수 연산)
  - ROS2 Navigation
  - Custom Message Types

## 🔍 Core Features

### 1. Lane Detection & Path Planning (차선 인식 & 경로 계획)
- 3차 다항식 피팅을 통한 차선 모델링
- 최소자승법(Least Squares Method) 기반 곡선 추정
- 중심선 기반 주행 경로 생성

### 2. Vehicle Control System (차량 제어 시스템)
#### Longitudinal Control (종방향 제어)
- PID 기반 속도 제어
- 장애물 기반 적응형 속도 조절

#### Lateral Control (횡방향 제어)
- Pure Pursuit 알고리즘 기반 조향 제어
- Look-ahead distance 기반 경로 추종

### 3. Safety Features (안전 기능)
- 실시간 장애물 감지 및 회피
- Thread-safe 데이터 처리
- 수동/자동 모드 전환 지원

## 🔧 System Architecture

```
[Sensors] → [Perception] → [Planning] → [Control] → [Vehicle]
    ↑           ↑            ↑            ↑
    └───────────────── [Safety Monitor] ──┘
```

### ROS2 Node Structure
- **Main Nodes**:
  - `autonomous_driving`: 핵심 제어 로직
  - `vehicle_simulator`: 차량 동역학 시뮬레이션
  - `scenario_runner`: 테스트 시나리오 실행
  - `evaluation`: 성능 평가

### Message Topics
- **Input**:
  - `/manual_input`
  - `vehicle_state`
  - `lane_points`
  - `obstacles`
  
- **Output**:
  - `vehicle_command`
  - `driving_way`
  - `poly_lanes`

## 💻 Installation & Setup

```bash
# Clone repository
git clone https://github.com/[organization]/team5-autonomous-driving.git

# Build project
cd team5-autonomous-driving
colcon build

# Source setup file
source install/setup.bash

# Run the system
ros2 launch team5_autonomous_driving autonomous_driving.launch.xml
```

## 🎮 Usage
1. **시뮬레이션 실행**:
   ```bash
   ros2 launch simulator simulation.launch.xml
   ```

2. **자율주행 시스템 실행**:
   ```bash
   ros2 launch autonomous_driving autonomous_driving.launch.xml
   ```

3. **테스트 시나리오 실행**:
   ```bash
   ros2 launch scenario_runner scenario_runner.launch.xml
   ```

## 🔍 Key Parameters

### PID Control
```yaml
pid_kp: 1.0  # Proportional gain
pid_ki: 0.1  # Integral gain
pid_kd: 0.05 # Derivative gain
```

### Pure Pursuit
```yaml
pp_kd: 1.0   # Distance gain
pp_kv: 0.5   # Velocity gain
pp_kc: 0.1   # Curvature gain
```

## 📊 Project Status
- [x] System Architecture Design
- [x] Basic Control System Implementation
- [x] Lane Detection Algorithm
- [x] Path Planning
- [ ] Advanced Safety Features
- [ ] Performance Optimization

## 🔎 Implementation Details

### Lane Detection
차선 인식 알고리즘은 3차 다항식 피팅을 사용하여 구현되었습니다:
```cpp
// 차선 피팅을 위한 행렬 구성
Eigen::MatrixXd H_left(lane_points_left_x.size(), 4);
H_left(i, 0) = 1.0;          // 상수항
H_left(i, 1) = x;            // 1차항
H_left(i, 2) = x * x;        // 2차항
H_left(i, 3) = x * x * x;    // 3차항
```

### Vehicle Control
PID 제어와 Pure Pursuit 알고리즘을 통한 통합 제어 시스템:
```cpp
// PID 속도 제어
computed_input = param_pid_kp_ * e +        // 비례항
                param_pid_ki_ * int_e +     // 적분항
                param_pid_kd_ * dev_e;      // 미분항

// Pure Pursuit 조향 제어
delta_radian = atan(2 * param_wheel_base_ * sin_alpha / x2);
```

## 📝 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🔗 Links
- [Project Wiki](wiki-link)
- [Issue Tracker](issues-link)
- [Project Board](project-board-link)

## 📞 Contact
- Jisang Yun - [email]
- Byeongheon Lee - [email]
- Jonguk Baek - [email]

## 🙏 Acknowledgments
- 미래자동차공학과 교수진
- ROS2 Community
- 프로젝트 지원팀

```
Note: GitHub IDs, 이메일 주소 등 개인정보는 각자 추가해주시기 바랍니다.
```
