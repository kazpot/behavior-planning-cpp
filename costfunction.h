#ifndef COSTFUNCTION_H
#define COSTFUNCTION_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"


// priority levels for costs
const double COLLISION  = pow(10,6);
const double DANGER     = pow(10,5);
const double REACH_GOAL = pow(10,5);
const double COMFORT    = pow(10,4);
const double EFFICIENCY = pow(10,2);

const double DESIRED_BUFFER = 1.5; // timesteps
const int PLANNING_HORIZON = 2;

const bool DEBUG = false;

class Costfunction{
public:
    struct TrajectoryData
    {
        int proposed_lane;
        double avg_speed;
        int max_acceleration;
        int rms_acceleration;
        int closest_approach;
        int end_distance_to_goal;
        int end_lanes_from_goal;
        std::map<std::string, int> collides;
    };

    /**
    * Constructor
    */
    Costfunction();
  
    /** 
    * Destructor
    */
    virtual ~Costfunction();

    double ChangeLaneCost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data);

    double DistanceFromGoalLane(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data);

    double InefficiencyCost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data);

    double CollisionCost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data);

    double BufferCost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data);

    double CalculateCost(Vehicle& vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, bool verbose);

    Costfunction::TrajectoryData GetHelperData(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions);

    bool CheckCollision(Vehicle::Snapshot snapshot, double s_previous, double s_now);

    std::vector<double>UnpackSnapshot(Vehicle::Snapshot snapshot);

    std::map<int,std::vector<std::vector<int>>> FilterPredictionsByLane(std::map<int,std::vector<std::vector<int>>> predictions, int lane);
};

#endif