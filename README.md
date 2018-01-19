# CoopMultiRobotsPlanning
This paper develops an effective, cooperative, and probabilistically-complete multi-robot motion planner.

The approach takes into account geometric and differential constraints imposed by the obstacles and the robot dynamics by using sampling to expand a motion tree in the composite state space of all the robots. Scalability and efficiency is achieved by using solutions to a simplified problem representation that does not take dynamics into account to guide the motion-tree expansion. The heuristic solutions are obtained by constructing roadmaps over low-dimensional configuration spaces and relying on cooperative multi-agent graph search to effectively find graph routes. Experimental results with second-order vehicle models operating in complex environments, where cooperation among the robots is required to find solutions, demonstrate significant improvements over related work

![webinterface](/ScenePics/case3_bumpy.jpg?raw=true "webinterface")

## Prerequisites
- OPENGL library
- GLUT library
- BOOST library
- Pybullet library

## Getting Started
```
cmake -DCMAKE_BUILD_TYPE="Release"
make
./bin/Runner GRunCoopPlanner data/case1/case1.txt data/case1/query/query0.txt 3
```
- Press "r" until solved.


## License

Please contact me via lesun90@gmail.com if you want to use this code for your work.
