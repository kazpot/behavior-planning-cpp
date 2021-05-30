#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <vector>
#include <thread>

//impacts default behavior for most states
int SPEED_LIMIT = 10;

//all traffic in lane (besides ego) follow these speeds
std::vector<int> LANE_SPEEDS = {6, 7, 8, 9};

//Number of available "cells" which should have traffic
double TRAFFIC_DENSITY   = 0.15;

// At each timestep, ego can set acceleration to value between 
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
std::vector<int> GOAL = {1500, 0};

// These affect the visualization
int FRAMES_PER_SECOND = 4;
int AMOUNT_OF_ROAD_VISIBLE = 40;

int main()
{
    Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);
    road.update_width = AMOUNT_OF_ROAD_VISIBLE;
    road.PopulateTraffic();

    int goal_s = GOAL[0];
    int goal_lane = GOAL[1];

    int num_lanes = LANE_SPEEDS.size();
    std::vector<int> ego_config = {SPEED_LIMIT, num_lanes, goal_s, goal_lane, MAX_ACCEL};
     
    road.AddEgo(1,0, ego_config);
    int time_step = 0;
    
    while (road.GetEgo().s <= GOAL[0])
    {
        ++time_step;
        if (time_step > 150)
        {
            std::cout << "Taking too long to reach goal. Go faster!" << std::endl;
            break;
        }
        road.Advance();
        road.Display(time_step);
        std::this_thread::sleep_for(std::chrono::milliseconds((long)(1.0 / FRAMES_PER_SECOND * 1000)));
    }
    Vehicle ego = road.GetEgo();
    if (ego.lane == GOAL[1])
    {
        std::cout << "You got to the goal in " << time_step << " seconds!" << std::endl;
    }
    else
    {
        std::cout << "You missed the goal. You are in lane " << ego.lane << " instead of " << GOAL[1] << std::endl;
    }

    return 0;
}