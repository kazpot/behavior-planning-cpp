#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

class Road {
public:
    int update_width = 70;
    std::string ego_rep = " *** ";
    int ego_key = -1;
    int num_lanes;
    std::vector<int> lane_speeds;
    int speed_limit;
    double density;
    int camera_center;
    std::map<int, Vehicle> vehicles;
    int vehicles_added = 0;

    /**
    * Constructor
    */
    Road(int speed_limit, double traffic_density, std::vector<int> lane_speeds);

    /**
    * Destructor
    */
    virtual ~Road();

    Vehicle GetEgo();

    void PopulateTraffic();

    void Advance();

    void Display(int timestep);

    void AddEgo(int lane_num, int s, std::vector<int> config_data);

    void Cull();

};

#endif
