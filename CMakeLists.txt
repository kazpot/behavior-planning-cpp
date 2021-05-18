cmake_minimum_required(VERSION 3.19)
project(Behavior_Planning_Example)

set(CMAKE_CXX_STANDARD 11)

add_executable(Behavior_Planning_Example main.cpp road.cpp road.h costfunction.cpp costfunction.h vehicle.cpp vehicle.h)