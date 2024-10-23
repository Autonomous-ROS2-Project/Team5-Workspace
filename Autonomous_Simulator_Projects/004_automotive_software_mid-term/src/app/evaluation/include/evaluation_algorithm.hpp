/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      evaluation_algorithm.cpp
 * @brief     autonomous driving algorithm evaluation tool
 * 
 * @date      2023-08-07 created by Yuseung Na (yuseungna@hanyang.ac.kr)
 */

#ifndef __EVALUATION_ALGORITHM_HPP__
#define __EVALUATION_ALGORITHM_HPP__
#pragma once

// STD Header
#include <string>
#include <math.h>

// Interface Header
#include "interface_lane.hpp"
#include "interface_vehicle.hpp"
#include "interface_scenario.hpp"

using namespace interface;

typedef struct {
    std::string ref_csv_path;
    std::string lane_id;

    double eval_time_limit;
    double eval_lane_departure;

} EvaluationParams;

class EvaluationAlgorithm {
    public:
        EvaluationAlgorithm(const EvaluationParams& params);
        ~EvaluationAlgorithm();
        
    public:
        void LoadLaneData();
        void SetGoalPoint();
        void CalcErrors(const VehicleState& ego, const std::vector<ObstacleState>& obstacles,
                        const double& limit_speed, const double& driving_time,
                        const double& dt);

        std::string GetEvaluationInfo(const VehicleState& ego, 
                                      const double& driving_time,
                                      const double& limit_speed);
        
        bool IsFinished(const VehicleState& ego, const double& gain);

        inline double GetSpeedPenalty() { return eval_speed_penalty_; }
        inline double GetCrossTrackError() { return eval_curr_cte_; }
        inline double GetLADCrossTrackError() { return eval_LAD_cte_; }
        inline double GetMaxCrossTrackError() { return eval_max_cte_; }
        inline bool IsCollision() { return eval_is_collision_; }
        inline bool IsLaneDeparture() { return eval_is_lane_departure; }
        inline bool IsRetire() { return eval_is_retire_; }

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Functions
        void CalcOverSpeedPenalty(const VehicleState& ego, const double& dt, const double& limit_speed);
        void CalcCrossTrackError(const VehicleState& ego, const double& dt);
        bool CheckCollision(const VehicleState& ego, const std::vector<ObstacleState>& obstacles);
        void CheckFailure(const bool& is_collision, const double& driving_time);
        
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Variables
        interface::Point2D goal_point_;

        // Outputs
        interface::Lane o_ref_lane_;
        
        // Evaluation results
        double eval_speed_penalty_ = 0.0;

        double eval_curr_cte_ = 0.0;
        double eval_LAD_cte_ = 0.0;
        double eval_max_cte_ = 0.0;

        double eval_curr_spacing_error_ = 0.0;
        double eval_LAD_spacing_error_ = 0.0;

        bool eval_is_lane_departure = false;
        bool eval_is_collision_ = false;
        bool eval_is_retire_ = false;

        EvaluationParams param_;
};


#endif // __EVALUATION_ALGORITHM_HPP__
