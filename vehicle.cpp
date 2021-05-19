#include <iostream>
#include "vehicle.h"
#include "costfunction.h"
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>


/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;

}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(std::map<int,std::vector < std::vector<int> > > predictions) {
    /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    state = this->get_next_state(predictions);
    this->state = state;
}

std::string Vehicle::get_next_state(std::map<int,std::vector<std::vector<int>>> predictions){
    std::vector<std::string> states = {"KL", "LCL", "LCR"};
    if(this->lane==0){
        std::vector<std::string>::iterator result = find(states.begin(), states.end(), "LCL");
        states.erase(result);
    }
    if(this->lane == (this->lanes_available -1)){
        std::vector<std::string>::iterator result = find(states.begin(), states.end(), "LCR");
        states.erase(result);
    }

    double min_cost = std::numeric_limits<double>::infinity();
    int min_index = 0;
    Costfunction costf = Costfunction();
    for(int i=0; i < states.size(); i++){
        std::map<int,std::vector<std::vector<int>>> predictions_copy = predictions;
        std::string state = states.at(i);
        std::vector<Vehicle::Snapshot> trajectory = this->trajectory_for_state(state,predictions_copy);
        double cost = costf.calculate_cost(*this, trajectory, predictions, false);
        if(cost < min_cost){
            min_cost = cost;
            min_index = i;
        }
    }
    std::string state = states.at(min_index);
    return state;

    /*
    states = ["KL", "LCL", "LCR"]
    if  self.lane == 0:
      states.remove("LCL")
    if self.lane == (self.lanes_available -1):
      states.remove("LCR")

    costs = []
    for state in states:
      predictions_copy = deepcopy(predictions)
      trajectory = self._trajectory_for_state(state,predictions_copy)
      cost = calculate_cost(self, trajectory, predictions)
      costs.append({"state": state, "cost" : cost})

    best = min(costs, key=lambda s: s['cost'])
    return best['state']
    */
}

std::vector<Vehicle::Snapshot> Vehicle::trajectory_for_state(std::string state, std::map<int,std::vector<std::vector<int>>> predictions, int horizon){
    Vehicle::Snapshot snap = this->snapshot();
    std::vector<Vehicle::Snapshot> trajectory;
    trajectory.push_back(snap);

    this->state = state;

    for(int i = 0; i < horizon; i++){
        this->restore_state_from_snapshot(snap);
        this->state = state;
        this->realize_state(predictions);
        this->increment(1);
        trajectory.push_back(this->snapshot());
        
        //need to remove first prediction for each vehicle
        for(auto map : predictions){
            for(auto v : map.second){
                v.erase(v.begin());
            }
        }
    }

    //restore state from snapshot
    this->restore_state_from_snapshot(snap);

    return trajectory;
}

void Vehicle::restore_state_from_snapshot(Vehicle::Snapshot snap){
    this->lane = snap.lane;
    this->s = snap.s;
    this->v = snap.v;
    this->a = snap.a;
    this->state = snap.state;
}

Vehicle::Snapshot Vehicle::snapshot(){
    Vehicle::Snapshot snapshot = {lane=this->lane, s=this->s, v=this->v, a=this->a, state=this->state};
    return snapshot;
}

void Vehicle::configure(std::vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    max_acceleration = road_data[2];
    goal_lane = road_data[3];
    goal_s = road_data[4];
}

std::string Vehicle::display() {

    std::ostringstream oss;
    
    oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {

    this->s += this->v * dt;
    this->v += this->a * dt;
}

std::vector<int> Vehicle::state_at(int t) {

    /*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

    /*
    Simple collision detection.
    */
    std::vector<int> check1 = state_at(at_time);
    std::vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1; 

    for (int t = 0; t < timesteps+1; t++)
    {
        if( collides_with(other, t) )
        {
            collider_temp.collision = true;
            collider_temp.time = t; 
            return collider_temp;
        }
    }

    return collider_temp;
}

void Vehicle::realize_state(std::map<int,std::vector<std::vector<int>>> predictions) {
   
    /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    std::string state = this->state;
    if(state.compare("CS") == 0)
    {
        realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
        realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
        realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
        realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
        realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
        realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
    a = 0;
}

int Vehicle::_max_accel_for_lane(std::map<int,std::vector<std::vector<int> > > predictions, int lane, int s) {

    int delta_v_til_target = target_speed - v;
    int max_acc = std::min(max_acceleration, delta_v_til_target);

    std::map<int, std::vector<std::vector<int> > >::iterator it = predictions.begin();
    std::vector<std::vector<std::vector<int> > > in_front;
    while(it != predictions.end())
    {
       
        int v_id = it->first;
        
        std::vector<std::vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
            in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
        int min_s = 1000;
        std::vector<std::vector<int>> leading = {};
        for(int i = 0; i < in_front.size(); i++)
        {
            if((in_front[i][0][1]-s) < min_s)
            {
                min_s = (in_front[i][0][1]-s);
                leading = in_front[i];
            }
        }
        
        int next_pos = leading[1][1];
        int my_next = s + this->v;
        int separation_next = next_pos - my_next;
        int available_room = separation_next - preferred_buffer;
        max_acc = std::min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(std::map<int,std::vector< std::vector<int> > > predictions) {
    this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(std::map<int,std::vector< std::vector<int> > > predictions, std::string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
        delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(std::map<int,std::vector<std::vector<int> > > predictions, std::string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
        delta = 1;
    }
    int lane = this->lane + delta;

    std::map<int, std::vector<std::vector<int> > >::iterator it = predictions.begin();
    std::vector<std::vector<std::vector<int> > > at_behind;
    while(it != predictions.end())
    {
        int v_id = it->first;
        std::vector<std::vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
            at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

        int max_s = -1000;
        std::vector<std::vector<int> > nearest_behind = {};
        for(int i = 0; i < at_behind.size(); i++)
        {
            if((at_behind[i][0][1]) > max_s)
            {
                max_s = at_behind[i][0][1];
                nearest_behind = at_behind[i];
            }
        }
        int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
        int delta_v = this->v - target_vel;
        int delta_s = this->s - nearest_behind[0][1];
        if(delta_v != 0)
        {

            int time = -2 * delta_s/delta_v;
            int a;
            if (time == 0)
            {
                a = this->a;
            }
            else
            {
                a = delta_v/time;
            }
            if(a > this->max_acceleration)
            {
                a = this->max_acceleration;
            }
            if(a < -this->max_acceleration)
            {
                a = -this->max_acceleration;
            }
            this->a = a;
        }
        else
        {
            int my_min_acc = std::max(-this->max_acceleration,-delta_s);
            this->a = my_min_acc;
        }

    }

}

std::vector<std::vector<int>> Vehicle::generate_predictions(int horizon = 10) {

    std::vector<std::vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      std::vector<int> check1 = state_at(i);
      std::vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
    }
    return predictions;

}