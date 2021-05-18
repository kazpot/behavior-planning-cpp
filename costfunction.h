#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

// priority levels for costs
const double COLLISION  = pow(10,6);
const double DANGER     = pow(10,5);
const double REACH_GOAL = pow(10,5);
const double COMFORT    = pow(10,4);
const double EFFICIENCY = pow(10,2);

const double DESIRED_BUFFER = 1.5; // timesteps
const int PLANNING_HORIZON = 2;

const bool DEBUG = false;
// const bool DEBUG = true;

class Costfunction{
public:
    struct TrajectoryData{
        int proposed_lane;
        double avg_speed;
        int max_acceleration;
        int rms_acceleration;
        int closest_approach;
        int end_distance_to_goal;
        int end_lanes_from_goal;
        map<string, int> collides;
    };

    /**
    * Constructor
    */
    Costfunction();
  
    /** 
    * Destructor
    */
    virtual ~Costfunction();

    double change_lane_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions, Costfunction::TrajectoryData data);

    double distance_from_goal_lane(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions, Costfunction::TrajectoryData data);

    double inefficiency_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions, Costfunction::TrajectoryData data);

    double collision_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions, Costfunction::TrajectoryData data);

    double buffer_cost(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions, Costfunction::TrajectoryData data);

    double calculate_cost(Vehicle& vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions, bool verbose);

    Costfunction::TrajectoryData get_helper_data(Vehicle vehicle, vector<Vehicle::Snapshot> trajectory, map<int,vector<vector<int>>> predictions);

    bool check_collision(Vehicle::Snapshot snapshot, double s_previous, double s_now);

    vector<double>unpack_snapshot(Vehicle::Snapshot snapshot);

    map<int,vector<vector<int>>> filter_predictions_by_lane(map<int,vector<vector<int>>> predictions, int lane);
};

#endif