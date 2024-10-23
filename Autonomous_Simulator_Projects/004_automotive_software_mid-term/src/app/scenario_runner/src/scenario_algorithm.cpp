/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      scenario_algorithm.cpp
 * @brief     mission scenario generation tool
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#include "scenario_algorithm.hpp"

ScenarioAlgorithm::ScenarioAlgorithm(const ScenarioParams& param) {
    param_ = param;

    LoadLaneData();
    GenerateObstacles();
    SetObstacleTargetSpeed();
    SetEgoLimitSpeed();
}

ScenarioAlgorithm::~ScenarioAlgorithm() {}

void ScenarioAlgorithm::LoadLaneData() {
    SLanes ref_csv_lane;
    ref_csv_lane.ImportLaneCsvFile(param_.ref_csv_path);

    ref_lanes_.frame_id = "world";
    ref_lanes_.lane.clear();

    obstacle_ref_lanes_.lanes.clear();

    for (size_t i_lane = 0; i_lane < ref_csv_lane.m_vecLanes.size(); i_lane++) {
        double accum_s = 0.0;
        interface::Point2D prev_point; 
        prev_point.x = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[0].m_dPtX_m;
        prev_point.y = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[0].m_dPtY_m;
        
        
        interface::Lane ith_lane;
        interface::RefLane jth_lane; // obstacle ref lane

        for (size_t i_point = 0; i_point < ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint.size(); i_point++) {
            interface::Point2D point;
            point.x = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtX_m;
            point.y = ref_csv_lane.m_vecLanes[i_lane].m_vecLanePoint[i_point].m_dPtY_m;

            // calculate the distance
            double dx = point.x - prev_point.x;
            double dy = point.y - prev_point.y;
            double s = sqrt(dx*dx + dy*dy);
            accum_s += s;
 
            // Update obstalce ref lanepoint
            interface::RefLanePoint ref_point;
            ref_point.x = point.x;
            ref_point.y = point.y;
            ref_point.s = accum_s;

            ith_lane.point.push_back(point);
            jth_lane.points.push_back(ref_point);

            prev_point = point;
        }
        ref_lanes_.lane.push_back(ith_lane);
        obstacle_ref_lanes_.lanes.push_back(jth_lane);
    }

    center_id_ = ref_lanes_.lane.size() / 2;
}

void ScenarioAlgorithm::GenerateObstacles() {
    for (int i=0; i<param_.num_of_obstacles; i++) {
        ObstacleState obstacle;

        // find initial position of obstacle
        int init_idx = 0;
        for (int j=0; j<obstacle_ref_lanes_.lanes[i].points.size(); j++) {
            if(obstacle_ref_lanes_.lanes[i].points[j].s > param_.obstacle_init_s) {
                init_idx = j;
                break;
            }
        }

        double s0 = obstacle_ref_lanes_.lanes[i].points[init_idx].s;
        double s1 = obstacle_ref_lanes_.lanes[i].points[init_idx + 1].s;
        double ratio = (param_.obstacle_init_s - s0) / (s1 - s0);

        interface::RefLanePoint p0 = obstacle_ref_lanes_.lanes[i].points[init_idx];
        interface::RefLanePoint p1 = obstacle_ref_lanes_.lanes[i].points[init_idx + 1];

        interface::RefLanePoint obs_init_pose;
        obs_init_pose.x = p0.x + ratio * (p1.x - p0.x);
        obs_init_pose.y = p0.y + ratio * (p1.y - p0.y);

        double dx = p1.x - p0.x;
        double dy = p1.y - p0.y;
        double yaw = std::atan2(dy, dx);

        obstacle.id = "obstacle";
        obstacle.x = obs_init_pose.x;
        obstacle.y = obs_init_pose.y;
        obstacle.yaw = yaw;
        obstacle.velocity = 30.0/3.6;
        obstacle.s = param_.obstacle_init_s;

        obstacle_states_.push_back(obstacle);
    }
}

void ScenarioAlgorithm::SetObstacleTargetSpeed() {
    object_target_speed_.push_back(std::make_pair(param_.obstacle_init_s+50.0, 30.0/3.6));
    object_target_speed_.push_back(std::make_pair(object_target_speed_.back().first+50.0, 70.0/3.6));
    object_target_speed_.push_back(std::make_pair(object_target_speed_.back().first+50.0, 40.0/3.6));
    object_target_speed_.push_back(std::make_pair(object_target_speed_.back().first+1000.0, 100.0/3.6));
}

void ScenarioAlgorithm::SetEgoLimitSpeed() {
    ego_limit_speed_.push_back(std::make_pair(10000.0, 65.0/3.6));
}

void ScenarioAlgorithm::UpdateEgoState(const VehicleState& vehicle_state) {
    i_vehicle_state_ = vehicle_state;

    double dx = i_vehicle_state_.x - obstacle_states_.back().x;
    double dy = i_vehicle_state_.y - obstacle_states_.back().y;
    double dist = sqrt(dx*dx + dy*dy);

    if (dist > param_.obstacle_detection_distance) {
        is_ego_close_to_obstacle_ = false;
    }
    else {
        is_ego_close_to_obstacle_ = true;
    }
}

double ScenarioAlgorithm::GetEgoLimitSpeed() {
    double limit_speed = 0.0;

    limit_speed = ego_limit_speed_[0].second;

    return limit_speed;
}

void ScenarioAlgorithm::UpdateObstacleState(double dt) {
    if(is_ego_close_to_obstacle_ == true) {
        for (int i=0; i<obstacle_states_.size(); i++) {
            // Update obstacle velocity
            double target_speed = 0.0;
            for(int k=0; k<object_target_speed_.size(); k++) {
                if(obstacle_states_[i].s < object_target_speed_[k].first) {
                    target_speed = object_target_speed_[k].second;
                    break;
                }
            }

            if(target_speed > obstacle_states_[i].velocity) {
                obstacle_states_[i].velocity += 3.0 * dt;
            }
            else {
                obstacle_states_[i].velocity -= 3.0 * dt;
            }

            // Update obstacle traveling distance(s)
            obstacle_states_[i].s += obstacle_states_[i].velocity * dt;

            // Restrict the obstalce's traveling distance to max length of lane
            if(obstacle_states_[i].s >= obstacle_ref_lanes_.lanes[i].points.back().s) {
                obstacle_states_[i].s = obstacle_ref_lanes_.lanes[i].points.back().s;
                obstacle_states_[i].is_reach_target = true;
            }

            if(obstacle_states_[i].is_reach_target == false) {
                // Find the position of obstacle
                int idx = 0;
                for (int j=obstacle_ref_lanes_.lanes[i].points.size()-1; j>=0; j--) {
                    if(obstacle_states_[i].s >  obstacle_ref_lanes_.lanes[i].points[j].s) {
                        idx = j;
                        break;
                    }
                }

                double s0 = obstacle_ref_lanes_.lanes[i].points[idx].s;
                double s1 = obstacle_ref_lanes_.lanes[i].points[idx + 1].s;
                double ratio = (obstacle_states_[i].s - s0) / (s1 - s0);

                interface::RefLanePoint p0 = obstacle_ref_lanes_.lanes[i].points[idx];
                interface::RefLanePoint p1 = obstacle_ref_lanes_.lanes[i].points[idx + 1];

                interface::RefLanePoint obs_pose;
                obs_pose.x = p0.x + ratio * (p1.x - p0.x);
                obs_pose.y = p0.y + ratio * (p1.y - p0.y);

                double dx = p1.x - p0.x;
                double dy = p1.y - p0.y;
                double yaw = std::atan2(dy, dx);

                obstacle_states_[i].x = obs_pose.x;
                obstacle_states_[i].y = obs_pose.y;
                obstacle_states_[i].yaw = yaw;
            }
            else {
                obstacle_states_[i].x = obstacle_ref_lanes_.lanes[i].points.back().x;
                obstacle_states_[i].y = obstacle_ref_lanes_.lanes[i].points.back().y;
                obstacle_states_[i].s = obstacle_ref_lanes_.lanes[i].points.back().s;
            }
        }
    }
}

std::vector<ObstacleState> ScenarioAlgorithm::GetObstacleState() {
    return obstacle_states_;
}
