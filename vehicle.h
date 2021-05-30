#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <iterator>

class Vehicle
{
public:

  struct Collider
  {
      bool collision;
      int  time;
  };

  struct Snapshot
  {
      int lane;
      int s;
      int v;
      int a;
      std::string state;
  };

  int L = 1;
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  int lane;
  int s;
  int v;
  int a;
  int target_speed;
  int lanes_available;
  int max_acceleration;
  int goal_lane;
  int goal_s;
  std::string state;

  /**
  * Constructor
  */
  Vehicle(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void UpdateState(std::map<int, std::vector<std::vector<int>>>& predictions);

  void Configure(std::vector<int> road_data);

  std::string Display();

  void Increment(int dt);

  std::vector<int> StateAt(int t);

  bool CollidesWith(Vehicle other, int at_time);

  Collider WillCollideWith(Vehicle other, int timesteps);

  void RealizeState(std::map<int, std::vector<std::vector<int>>> predictions);

  void RealizeConstantSpeed();

  int MaxAccelForLane(std::map<int,std::vector<std::vector<int>>> predictions, int lane, int s);

  void RealizeKeepLane(std::map<int, std::vector<std::vector<int>>> predictions);

  void RealizeLaneChange(std::map<int,std::vector<std::vector<int>>> predictions, std::string direction);

  void RealizePrepLaneChange(std::map<int,std::vector<std::vector<int>>> predictions, std::string direction);

  std::vector<std::vector<int>> GeneratePredictions(int horizon);

  std::string GetNextState(std::map<int,std::vector<std::vector<int>>> predictions);

  std::vector<Vehicle::Snapshot> TrajectoryForState(std::string state, std::map<int,std::vector<std::vector<int>>> predictions, int horizon=5);

  void RestoreStateFromSnapshot(Vehicle::Snapshot);

  Vehicle::Snapshot snapshot();
};

#endif