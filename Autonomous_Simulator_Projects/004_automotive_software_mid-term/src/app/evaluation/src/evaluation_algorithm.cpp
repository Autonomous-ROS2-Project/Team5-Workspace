/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_algorithm.cpp
 * @brief     autonomous driving algorithm evaluation tool
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#include "evaluation_algorithm.hpp"

EvaluationAlgorithm::EvaluationAlgorithm(const EvaluationParams& param) {
    param_ = param;
}

EvaluationAlgorithm::~EvaluationAlgorithm() {}

void EvaluationAlgorithm::LoadLaneData() {
    SLanes ref_csv_lane;
    ref_csv_lane.ImportLaneCsvFile(param_.ref_csv_path);

    o_ref_lane_.frame_id = "world";
    o_ref_lane_.point.clear();
    o_ref_lane_.id = param_.lane_id;

    for (auto i_lane = 0; i_lane < ref_csv_lane.m_vecLanes.size(); i_lane++) {
        if (param_.lane_id == std::to_string(ref_csv_lane.m_vecLanes[i_lane].m_nLaneID)) {
            for (auto i_point = 0;
                    i_point < ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint.size();
                    i_point++) {
                
                interface::Point2D point;
                point.x = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtX_m;
                point.y = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtY_m;

                o_ref_lane_.point.push_back(point);
            }
        }
    }
}

void EvaluationAlgorithm::SetGoalPoint() {
    goal_point_.x = o_ref_lane_.point.back().x;
    goal_point_.y = o_ref_lane_.point.back().y;
}

void EvaluationAlgorithm::CalcErrors(const VehicleState& ego, const std::vector<ObstacleState>& obstacles,
                                     const double& limit_speed, const double& driving_time,
                                     const double& dt) {
    CalcOverSpeedPenalty(ego, dt, limit_speed);
    CalcCrossTrackError(ego, dt);
    bool is_collision = CheckCollision(ego, obstacles);
    CheckFailure(is_collision, driving_time);
}

void EvaluationAlgorithm::CalcOverSpeedPenalty(const VehicleState& ego, const double& dt, const double& limit_speed) {
    if (limit_speed > 0.1 && ego.velocity > limit_speed) {
        eval_speed_penalty_ += ego.velocity * dt *
                                (ego.velocity - limit_speed) / limit_speed;
        
    }
}

bool EvaluationAlgorithm::CheckCollision(const VehicleState& ego, const std::vector<ObstacleState>& obstacles) {
    std::vector<Point2D> ego_circles;
    
    Point2D ego_circle_1;
    ego_circle_1.x = ego.x;
    ego_circle_1.y = ego.y;
    ego_circles.push_back(ego_circle_1);

    Point2D ego_circle_2;
    ego_circle_2.x = ego_circle_1.x + 1.5*cos(ego.yaw);
    ego_circle_2.y = ego_circle_1.y + 1.5*sin(ego.yaw);
    ego_circles.push_back(ego_circle_2);

    Point2D ego_circle_3;
    ego_circle_3.x = ego_circle_2.x + 1.5*cos(ego.yaw);
    ego_circle_3.y = ego_circle_2.y + 1.5*sin(ego.yaw);
    ego_circles.push_back(ego_circle_3);

    double circle_radius = 1.0;

    for (int i=0; i<obstacles.size(); i++) {
        std::vector<Point2D> obs_circles;

        Point2D obs_circle_1;
        obs_circle_1.x = obstacles[i].x;
        obs_circle_1.y = obstacles[i].y;

        for(int j=0; j<ego_circles.size(); j++) {
            double dx = ego_circles[j].x - obs_circle_1.x;
            double dy = ego_circles[j].y - obs_circle_1.y;
            double dist = sqrt(dx*dx + dy*dy);

            if(dist < 2*circle_radius) {
                return true;
            }
        }    

        Point2D obs_circle_2;
        obs_circle_2.x = obs_circle_1.x + 1.5*cos(obstacles[i].yaw);
        obs_circle_2.y = obs_circle_1.y + 1.5*sin(obstacles[i].yaw);
        
        for(int j=0; j<ego_circles.size(); j++) {
            double dx = ego_circles[j].x - obs_circle_2.x;
            double dy = ego_circles[j].y - obs_circle_2.y;
            double dist = sqrt(dx*dx + dy*dy);

            if(dist < 2*circle_radius) {
                return true;
            }
        }    

        Point2D obs_circle_3;
        obs_circle_3.x = obs_circle_2.x + 1.5*cos(obstacles[i].yaw);
        obs_circle_3.y = obs_circle_2.y + 1.5*sin(obstacles[i].yaw);
        
        for(int j=0; j<ego_circles.size(); j++) {
            double dx = ego_circles[j].x - obs_circle_3.x;
            double dy = ego_circles[j].y - obs_circle_3.y;
            double dist = sqrt(dx*dx + dy*dy);

            if(dist < 2*circle_radius) {
                return true;
            }
        }  
    }

    return false;
}

void EvaluationAlgorithm::CalcCrossTrackError(const VehicleState& ego, const double& dt) {
    double min1_distance_sq = std::numeric_limits<double>::max();
    double min2_distance_sq = std::numeric_limits<double>::max();
    interface::Point2D min1_point;
    interface::Point2D min2_point;
    for (auto i = 0; i < o_ref_lane_.point.size(); i++) {
        double dx = ego.x - o_ref_lane_.point[i].x;
        double dy = ego.y - o_ref_lane_.point[i].y;
        double distance_sq = dx * dx + dy * dy;

        if (distance_sq < min1_distance_sq) {
            min2_distance_sq = min1_distance_sq;
            min2_point = min1_point;

            min1_distance_sq = distance_sq;
            min1_point = o_ref_lane_.point[i];
        } else if (distance_sq < min2_distance_sq) {
            min2_distance_sq = distance_sq;
            min2_point = o_ref_lane_.point[i];
        }
    }

    double a = (min2_point.y - min1_point.y) / (min2_point.x - min1_point.x);
    double b = -1.0;
    double c = -1.0 * a * min2_point.x + min2_point.y;

    eval_curr_cte_ = (a * ego.x + b * ego.y + c) / (pow(a * a + b * b, 0.5));

    if (fabs(eval_curr_cte_) > eval_max_cte_) {
        eval_max_cte_ = fabs(eval_curr_cte_);
    }

    eval_LAD_cte_ += dt * fabs(eval_curr_cte_);
}

void EvaluationAlgorithm::CheckFailure(const bool& is_collision,
                                       const double& driving_time) {
    // 1. Lane Departure
    if (fabs(eval_curr_cte_) > param_.eval_lane_departure){
        eval_is_lane_departure = true;
    }
    // 2. Collision
    if (is_collision == true){
        eval_is_collision_ = true;
    }

    // 3. Time Limit
    if (driving_time > param_.eval_time_limit){
        eval_is_retire_ = true;
    }
}

bool EvaluationAlgorithm::IsFinished(const VehicleState& ego, const double& gain) {
    double dx = ego.x - goal_point_.x;
    double dy = ego.y - goal_point_.y;
    double distance_sq = dx * dx + dy * dy;

    if (distance_sq <= gain * gain) {
        return true;
    }
    else {
        return false;
    }
}

std::string EvaluationAlgorithm::GetEvaluationInfo(const VehicleState& ego, 
                                                   const double& driving_time,
                                                   const double& limit_speed) {

    std::ostringstream evaluation_info;

    evaluation_info         
        << "\n\n"
        << "///////////////////////////////////////////////////"
        << "\n"
        << "/////////////// [Evaluation Result] ///////////////"
        << "\n"
        << "1. Limit Speed:\t\t" << limit_speed << " m/s"
        << "\n"
        << "   Vehicle Speed:\t" << ego.velocity << " m/s"
        << "\n"
        << "   ------------------------------------------------"
        << "\n"
        << "   Over Speed Penalty:\t" << eval_speed_penalty_ << " sec"
        << "\n"
        << "\n"
        << "==================================================="
        << "\n"
        << "2. Cross Track Error:\t\t" << eval_curr_cte_ << " m"
        << "\n"
        << "   Lane Departure Threshold:\t" << param_.eval_lane_departure << " m"
        << "\n"
        // << "   LAD Cross Track Error:\t" << eval_LAD_cte_ << " m"
        // << "\n"
        << "   Max Cross Track Error:\t" << eval_max_cte_ << " m"
        << "\n"
        << "\n"
        << "==================================================="
        // << "\n"
        // << "3. Spacing Error: " << eval_curr_spacing_error_ << " m"
        // << "\n"
        // << "   LAD Spacing Error: " << eval_LAD_spacing_error_ << " m"
        // << "\n"
        // << "   Collision: " << (eval_is_collision_ ? "TRUE" : "FALSE") << "\n"
        // << "\n"
        // << "==================================================="
        << "\n"
        << "3. Driving Time:\t" << driving_time << " sec"
        << "\n"
        << "   Time Limit:\t\t" << param_.eval_time_limit << " sec"
        << "\n"
        << "\n"
        << "==================================================="
        << "\n"
        << "4. Failure:"
        << "\n"
        << "   Time Limit:\t\t" << (eval_is_retire_ ? "FAIL" : "SUCCESS") << "\n"
        << "   Lane Departure:\t" << (eval_is_lane_departure ? "FAIL" : "SUCCESS") << "\n"
        << "   Collision:\t\t" << (eval_is_collision_ ? "FAIL" : "SUCCESS") << "\n"
        << "\n"
        << "\n"
        << "------------------- [Result] ----------------------"
        << "\n"
        << "Time Score:\t" << (driving_time + eval_speed_penalty_)
        << "\n" 
        << "Failure:\t" << ((eval_is_retire_
                           + eval_is_lane_departure
                           + eval_is_collision_) ? "FAIL" : "SUCCESS")
        << "\n"                            
        << "///////////////////////////////////////////////////";

    return evaluation_info.str();
}
