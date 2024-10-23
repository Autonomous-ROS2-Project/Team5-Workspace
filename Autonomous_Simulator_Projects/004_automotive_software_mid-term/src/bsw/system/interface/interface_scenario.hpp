/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      interface_scenario.hpp
 * @brief     scenario component structure
 * 
 * @date      2024-10-21 created by Seongjae Jeong (sjeong99@hanyang.ac.kr)
 */

#ifndef __INTERFACE_SCENARIO_HPP__
#define __INTERFACE_SCENARIO_HPP__
#pragma once

#include <vector>
#include <string>

namespace interface {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    typedef struct {
        double x{0.0};
        double y{0.0};
        double s{0.0};
    } RefLanePoint;
    
    typedef struct {
        std::vector<RefLanePoint> points;
    } RefLane;

    typedef struct {
        std::vector<RefLane> lanes;
    } RefLanes;

    typedef struct {
        std::string id;
        bool is_reach_target{false};
        int ref_lane_id{0};
        double x{0.0};
        double y{0.0};
        double s{0.0};
        double yaw{0.0};
        double velocity{0.0};
    } ObstacleState;
} // namespace interface

#endif // __INTERFACE_SCENARIO_HPP__