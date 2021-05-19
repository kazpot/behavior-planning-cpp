#include "road.h"
#include "vehicle.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

//impacts default behavior for most states
int SPEED_LIMIT = 10;

//all traffic in lane (besides ego) follow these speeds
std::vector<int> LANE_SPEEDS = {6,7,8,9};

//Number of available "cells" which should have traffic
double TRAFFIC_DENSITY   = 0.15;

// At each timestep, ego can set acceleration to value between 
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

// s value and lane number of goal.
std::vector<int> GOAL = {300, 0};

// These affect the visualization
int FRAMES_PER_SECOND = 4;
int AMOUNT_OF_ROAD_VISIBLE = 40;

int main() {
 
    Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

    road.update_width = AMOUNT_OF_ROAD_VISIBLE;

    road.populate_traffic();

    int goal_s = 300;
    int goal_lane = 0;

    //configuration data: speed limit, num_lanes, goal_s, goal_lane, max_acceleration

    int num_lanes = LANE_SPEEDS.size();
    std::vector<int> ego_config = {SPEED_LIMIT, num_lanes, goal_s, goal_lane, MAX_ACCEL};
     
    road.add_ego(2,0, ego_config);
    int timestep = 0;
    
    while (road.get_ego().s <= GOAL[0]) {
        timestep++;
        if (timestep > 150) {
            std::cout << "Taking too long to reach goal. Go faster!" << std::endl;
            break;
        }
        road.advance();
        road.display(timestep);
        //time.sleep(float(1.0) / FRAMES_PER_SECOND);
    }
    Vehicle ego = road.get_ego();
    if (ego.lane == GOAL[1])
    {
        std::cout << "You got to the goal in " << timestep << " seconds!" << std::endl;
    }
    else
    {
        std::cout << "You missed the goal. You are in lane " << ego.lane << " instead of " << GOAL[1] << std::endl;
    }

    return 0;
}