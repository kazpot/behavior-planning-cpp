# Behavior Planner
This is a behavior planner and cost functions for highway driving. 
It will navigate through the traffic to the goal in as little as possible. 

5 vehicle states
* "KL" - Keep Lane
* "LCL" / "LCR"- Lane Change Left / Right
* "PLCL" / "PLCR" - Prepare Lane Change Left / Right

# Build

```bash
$ mkdir build
$ cmake ..
$ make
```