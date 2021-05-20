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

  void update_state(std::map<int, std::vector<std::vector<int>>>& predictions);

  void configure(std::vector<int> road_data);

  std::string display();

  void increment(int dt);

  std::vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  Collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(std::map<int, std::vector<std::vector<int>>> predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(std::map<int,std::vector<std::vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(std::map<int, std::vector<std::vector<int> >> predictions);

  void realize_lane_change(std::map<int,std::vector<std::vector<int>>> predictions, std::string direction);

  void realize_prep_lane_change(std::map<int,std::vector<std::vector<int>>> predictions, std::string direction);

  std::vector<std::vector<int> > generate_predictions(int horizon);

  std::string get_next_state(std::map<int,std::vector<std::vector<int>>> predictions);

  std::vector<Vehicle::Snapshot> trajectory_for_state(std::string state, std::map<int,std::vector<std::vector<int>>> predictions, int horizon=5);

  void restore_state_from_snapshot(Vehicle::Snapshot);

  Vehicle::Snapshot snapshot();
};

#endif