#include <iostream>
#include "vehicle.h"
#include "costfunction.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>


/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {
    this->lane_ = lane;
    this->s_ = s;
    this->v_ = v;
    this->a_ = a;
    this->state_ = "CS";
    this->max_acceleration_ = -1;
}

Vehicle::~Vehicle() {}

void Vehicle::UpdateState(std::map<int, std::vector<std::vector<int>>> &predictions) {
    this->state_ = this->GetNextState(predictions);
}

std::string Vehicle::GetNextState(std::map<int,std::vector<std::vector<int>>> predictions){
    std::vector<std::string> states = {"KL", "LCL", "LCR"};

    if (this->lane_ == 0){
        auto result = find(states.begin(), states.end(), "LCL");
        states.erase(result);
    }

    if (this->lane_ == (this->lanes_available_ - 1)){
        auto result = find(states.begin(), states.end(), "LCR");
        states.erase(result);
    }

    double min_cost = std::numeric_limits<double>::infinity();
    int min_index = 0;
    Costfunction cost_function = Costfunction();
    for (int i = 0; i < states.size(); ++i){
        const std::map<int, std::vector<std::vector<int>>> &predictions_copy = predictions;
        std::string st = states.at(i);
        std::vector<Vehicle::Snapshot> trajectory = this->TrajectoryForState(st, predictions_copy);
        double cost = cost_function.CalculateCost(*this, trajectory, predictions, false);
        if(cost < min_cost){
            min_cost = cost;
            min_index = i;
        }
    }
    return states.at(min_index);
}

std::vector<Vehicle::Snapshot> Vehicle::TrajectoryForState(std::string st, std::map<int, std::vector<std::vector<int>>> predictions, int horizon)
{
    Vehicle::Snapshot snap = this->GetSnapshot();
    std::vector<Vehicle::Snapshot> trajectory;
    trajectory.push_back(snap);

    this->state_ = st;

    for (int i = 0; i < horizon; ++i)
    {
        this->RestoreStateFromSnapshot(snap);
        this->state_ = st;
        this->RealizeState(predictions);
        this->Increment(1);
        trajectory.push_back(this->GetSnapshot());
        
        // need to remove first prediction for each vehicle
        for(auto &map : predictions)
        {
            for(auto vehicle : map.second)
            {
                vehicle.erase(vehicle.begin());
            }
        }
    }

    //restore state from snapshot
    this->RestoreStateFromSnapshot(snap);

    return trajectory;
}

void Vehicle::RestoreStateFromSnapshot(Vehicle::Snapshot &snap){
    this->lane_ = snap.lane;
    this->s_ = snap.s;
    this->v_ = snap.v;
    this->a_ = snap.a;
    this->state_ = snap.state;
}

Vehicle::Snapshot Vehicle::GetSnapshot()
{
    Vehicle::Snapshot snapshot = {
            this->lane_,
            this->s_,
            this->v_,
            this->a_,
            this->state_
    };
    return snapshot;
}

void Vehicle::Configure(std::vector<int> road_data)
{
    this->target_speed_ = road_data[0];
    this->lanes_available_ = road_data[1];
    this->max_acceleration_ = road_data[2];
    this->goal_lane_ = road_data[3];
    this->goal_s_ = road_data[4];
}

std::string Vehicle::Display()
{
    std::ostringstream oss;
    oss << "s:    " << this->s_ << "\n";
    oss << "lane: " << this->lane_ << "\n";
    oss << "v:    " << this->v_ << "\n";
    oss << "a:    " << this->a_ << "\n";
    return oss.str();
}

void Vehicle::Increment(int dt = 1)
{
    this->s_ += this->v_ * dt;
    this->v_ += this->a_ * dt;
}

std::vector<int> Vehicle::StateAt(int t)
{
    // Predicts state of vehicle in t seconds (assuming constant acceleration)
    int s = this->s_ + this->v_ * t + this->a_ * t * t / 2;
    int v = this->v_ + this->a_ * t;
    return {this->lane_, s, v, this->a_};
}

bool Vehicle::CollidesWith(Vehicle other, int at_time)
{
    // Simple collision detection.
    std::vector<int> check1 = StateAt(at_time);
    std::vector<int> check2 = other.StateAt(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::Collider Vehicle::WillCollideWith(Vehicle other, int time_steps)
{
    Vehicle::Collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1; 

    for (int t = 0; t < time_steps + 1; ++t)
    {
        if (CollidesWith(other, t) )
        {
            collider_temp.collision = true;
            collider_temp.time = t; 
            return collider_temp;
        }
    }
    return collider_temp;
}

void Vehicle::RealizeState(std::map<int,std::vector<std::vector<int>>> predictions)
{
    // Given a state, realize it by adjusting acceleration and lane.
    // Note - lane changes happen instantaneously.
    std::string state = this->state_;
    if(state == "CS")
    {
        RealizeConstantSpeed();
    }
    else if(state == "KL")
    {
        RealizeKeepLane(predictions);
    }
    else if(state == "LCL")
    {
        RealizeLaneChange(predictions, "L");
    }
    else if(state == "LCR")
    {
        RealizeLaneChange(predictions, "R");
    }
}

void Vehicle::RealizeConstantSpeed()
{
    this->a_ = 0;
}

int Vehicle::MaxAccelForLane(std::map<int, std::vector<std::vector<int>>> predictions, int lane, int s)
{
    int delta_v_til_target = this->target_speed_ - this->v_;
    int max_acc = std::min(this->max_acceleration_, delta_v_til_target);

    auto it = predictions.begin();
    std::vector<std::vector<std::vector<int>>> in_front;
    while(it != predictions.end())
    {
        std::vector<std::vector<int>> v = it->second;
        if((v[0][0] == lane) && (v[0][1] > s))
        {
            in_front.push_back(v);
        }
        ++it;
    }
    
    if (!in_front.empty())
    {
        int min_s = 1000;
        std::vector<std::vector<int>> leading = {};
        for(auto &i : in_front)
        {
            if((i[0][1] - s) < min_s)
            {
                min_s = i[0][1] - s;
                leading = i;
            }
        }
        
        int next_pos = leading[1][1];
        int my_next = s + this->v_;
        int separation_next = next_pos - my_next;
        int available_room = separation_next - this->preferred_buffer_;
        max_acc = std::min(max_acc, available_room);
    }
    return max_acc;
}

void Vehicle::RealizeKeepLane(std::map<int,std::vector< std::vector<int>>> predictions)
{
    this->a_ = MaxAccelForLane(predictions, this->lane_, this->s_);
}

void Vehicle::RealizeLaneChange(std::map<int,std::vector< std::vector<int>>> predictions, std::string direction)
{
    int delta = -1;
    if (direction == "L")
    {
        delta = 1;
    }
    this->lane_ += delta;
    this->a_ = MaxAccelForLane(predictions, this->lane_, this->s_);
}

void Vehicle::RealizePrepLaneChange(std::map<int, std::vector<std::vector<int>>> predictions, std::string direction)
{
    int delta = -1;
    if (direction == "L")
    {
        delta = 1;
    }

    int lane = this->lane_ + delta;
    auto it = predictions.begin();
    std::vector<std::vector<std::vector<int>>> at_behind;
    while(it != predictions.end())
    {
        std::vector<std::vector<int>> v = it->second;
        if((v[0][0] == lane) && (v[0][1] <= this->s_))
        {
            at_behind.push_back(v);

        }
        ++it;
    }

    if(!at_behind.empty())
    {
        int max_s = -1000;
        std::vector<std::vector<int>> nearest_behind = {};
        for(auto &i : at_behind)
        {
            if((i[0][1]) > max_s)
            {
                max_s = i[0][1];
                nearest_behind = i;
            }
        }
        int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
        int delta_v = this->v_ - target_vel;
        int delta_s = this->s_ - nearest_behind[0][1];
        if(delta_v != 0)
        {
            int time = -2 * delta_s / delta_v;
            int a;
            if (time == 0)
            {
                a = this->a_;
            }
            else
            {
                a = delta_v/time;
            }

            if(a > this->max_acceleration_)
            {
                a = this->max_acceleration_;
            }

            if(a < -this->max_acceleration_)
            {
                a = -this->max_acceleration_;
            }
            this->a_ = a;
        }
        else
        {
            int my_min_acc = std::max(-this->max_acceleration_, -delta_s);
            this->a_ = my_min_acc;
        }
    }
}

std::vector<std::vector<int>> Vehicle::GeneratePredictions(int horizon = 10)
{
    std::vector<std::vector<int>> predictions;
    for( int i = 0; i < horizon; ++i)
    {
      std::vector<int> check1 = StateAt(i);

      // Predicts s position until 9 time_step ahead
      // lane_s = {lane, s}
      std::vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
    }
    return predictions;
}