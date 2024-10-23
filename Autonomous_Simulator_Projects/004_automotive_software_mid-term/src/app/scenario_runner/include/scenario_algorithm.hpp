/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      scenario_algorithm.hpp
 * @brief     mission scenario generation tool
 * 
 * @date      2024-10-10 created by Seokhwan Jeong (shjeong00@hanyang.ac.kr)
 *            2024-10-18 updated by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */


#ifndef __SCENARIO_ALGORITHM_HPP__
#define __SCENARIO_ALGORITHM_HPP__
#pragma once

// STD Header
#include <string>
#include <vector>
#include <cmath>
#include <random>

// Interface Header
#include "interface_lane.hpp"
#include "interface_vehicle.hpp"
#include "interface_scenario.hpp"

using namespace interface;

typedef struct {
    std::string ref_csv_path;
    int num_of_lanes;
    int num_of_obstacles;
    double obstacle_init_s;
    double obstacle_detection_distance;
} ScenarioParams;

class ScenarioAlgorithm {
public:
    ScenarioAlgorithm(const ScenarioParams& params);
    ~ScenarioAlgorithm();
    
    void LoadLaneData();
    void GenerateObstacles();
    void SetObstacleTargetSpeed();
    void SetEgoLimitSpeed();
    void UpdateEgoState(const VehicleState& vehicle_state);
    void UpdateObstacleState(double dt);
    double GetEgoLimitSpeed();
    std::vector<ObstacleState> GetObstacleState();

private:
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Variables
    interface::Lanes ref_lanes_;
    interface::RefLanes obstacle_ref_lanes_;
    int center_id_ = 0;

    // Inputs
    VehicleState i_vehicle_state_;

    // Parameters
    ScenarioParams param_;

    std::vector<ObstacleState> obstacle_states_;
    std::vector<std::pair<double,double>> object_target_speed_;              // pair<s,v> [m/s]
    std::vector<std::pair<double,double>> ego_limit_speed_;

    bool is_ego_close_to_obstacle_ = false;
};

#endif // __SCENARIO_ALGORITHM_HPP__
