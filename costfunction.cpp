#include <iostream>
#include "costfunction.h"
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>


/**
 * Initializes Costfunction
 */
Costfunction::Costfunction() {
}

Costfunction::~Costfunction() {}


double Costfunction::change_lane_cost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data){
    /*
    Penalizes lane changes AWAY from the goal lane and rewards
    lane changes TOWARDS the goal lane.
    */
    int proposed_lanes = data.end_lanes_from_goal;
    int cur_lanes = trajectory[0].lane;
    double cost = 0;
    if (proposed_lanes > cur_lanes){
        cost = COMFORT;
    }
    if (proposed_lanes < cur_lanes){
        cost = -1.0 * COMFORT;
    }
    if (cost != 0){
        std::cout << "!!cost for lane change is " << cost << "\n\n";
    }
    return cost;
}

double Costfunction::distance_from_goal_lane(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data){

    int distance = abs(data.end_distance_to_goal);
    distance = std::max(distance, 1);
    int time_to_goal = distance / data.avg_speed;
    int lanes = data.end_lanes_from_goal;
    double multiplier = (double)(5 * lanes / time_to_goal);
    double cost = multiplier * REACH_GOAL;
    return cost;
}

double Costfunction::inefficiency_cost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data){

    int speed = data.avg_speed;
    int target_speed = vehicle.target_speed;
    int diff = target_speed - speed;
    double pct = double(diff) / target_speed;
    double multiplier = pow(pct, 2);
    return multiplier * EFFICIENCY;
}


double Costfunction::collision_cost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data){

    if (data.collides.size() > 0){
        int time_til_collision = data.collides["at"];
        double exponent = (double)pow(time_til_collision,2);
        double multiplier = exp(-1 * exponent);
        return multiplier * COLLISION;
    }
    return 0.0;
}

double Costfunction::buffer_cost(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, Costfunction::TrajectoryData data)
{
    int closest = data.closest_approach;
    if (closest == 0)
    {
        return 10 * DANGER;
    }

    int timesteps_away = closest / data.avg_speed;
    if (timesteps_away > DESIRED_BUFFER)
    {
        return 0.0;
    }
    double multiplier = 1.0 - pow(timesteps_away / DESIRED_BUFFER, 2);
    return multiplier * DANGER;
}

double Costfunction::calculate_cost(Vehicle& vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions, bool verbose=false){

    TrajectoryData trajectory_data = TrajectoryData();
    trajectory_data = this->get_helper_data(vehicle, trajectory, predictions);

    double cost = 0.0;

    cost += this->distance_from_goal_lane(vehicle, trajectory, predictions, trajectory_data);
    cost += this->inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);
    cost += this->collision_cost(vehicle, trajectory, predictions, trajectory_data);
    cost += this->buffer_cost(vehicle, trajectory, predictions, trajectory_data);
    cost += this->change_lane_cost(vehicle, trajectory, predictions, trajectory_data);

    return cost;
}

Costfunction::TrajectoryData Costfunction::get_helper_data(Vehicle vehicle, std::vector<Vehicle::Snapshot> trajectory, std::map<int,std::vector<std::vector<int>>> predictions){
    std::vector<Vehicle::Snapshot> t = trajectory;
    Vehicle::Snapshot current_snapshot = t.front();
    Vehicle::Snapshot first = t.at(1);
    Vehicle::Snapshot last = t.back();
    int end_distance_to_goal = vehicle.goal_s - last.s;
    int end_lanes_from_goal = abs(vehicle.goal_lane - last.lane);
    double dt = (double)trajectory.size(); 
    int proposed_lane = first.lane;
    double avg_speed = (double)((last.s - current_snapshot.s) / dt);

    //initialize variables
    std::map<std::string, int> collides;
    std::vector<int> accels;
    int closest_approach = 999999;
    bool is_collided = false;
    Vehicle::Snapshot last_snap = trajectory.front();
    std::map<int,std::vector<std::vector<int>>> filtered = this->filter_predictions_by_lane(predictions, proposed_lane);

    for(int i = 1; i < PLANNING_HORIZON + 1; i++){
        Vehicle::Snapshot ss = trajectory.at(i);
        int lane = ss.lane;
        int s = ss.s;
        int v = ss.v;
        int a = ss.a;
        accels.push_back(a);

        for(auto item : filtered){
            int v_id = item.first;
            std::vector<std::vector<int>> v = item.second;
            std::vector<int> state = v.at(i);
            std::vector<int> last_state = v.at(i-1);
            bool vehicle_collides = this->check_collision(ss, last_state.at(1), state.at(1));
            if(vehicle_collides){
                collides["at"] = i;
            }
            int dist = abs(state.at(1) - s);
            if(dist < closest_approach){
                closest_approach = dist;
            }
        }
        last_snap = ss;
    }
    int max_accel = *max_element(accels.begin(), accels.end());
    
    std::vector<int> rms_accels;
    for(int i; i < accels.size(); i++){
        int a = accels.at(i);
        int rms = pow(a, 2);
        rms_accels.push_back(rms);
    }

    double sum = (double)accumulate(rms_accels.begin(), rms_accels.end(), 0);
    int num_accels = rms_accels.size();
    double rms_acceleration = (double)sum/num_accels;

    TrajectoryData trajectory_data = TrajectoryData();
    trajectory_data.proposed_lane = proposed_lane;
    trajectory_data.avg_speed = avg_speed;
    trajectory_data.max_acceleration = max_accel;
    trajectory_data.rms_acceleration = rms_acceleration;
    trajectory_data.closest_approach = closest_approach;
    trajectory_data.end_distance_to_goal = end_distance_to_goal;
    trajectory_data.end_lanes_from_goal = end_lanes_from_goal;
    trajectory_data.collides = collides;

    return trajectory_data;
}

bool Costfunction::check_collision(Vehicle::Snapshot snapshot, double s_previous, double s_now){
    int s = snapshot.s;
    int v = snapshot.v;
    int v_target = s_now - s_previous;
    if(s_previous < s){
        if(s_now >= s){
            return true;
        }else{
            return false;
        }
    }
    
    if(s_previous > s){
        if(s_now <= s){
            return true;
        }else{
            return false;
        }
    }

    if(s_previous == s){
        if(v_target > v){
            return false;
        }else{
            return true;
        }
    }
}

std::map<int,std::vector<std::vector<int>>> Costfunction::filter_predictions_by_lane(std::map<int,std::vector<std::vector<int>>> predictions, int lane){

    std::map<int,std::vector<std::vector<int>>> filtered;
    for(auto map : predictions){
        int lane = map.second.front().back();
        int v_id = map.first;
        if( lane == lane && v_id != -1){
            filtered[v_id] = map.second;
        }
    }
    return filtered;
}