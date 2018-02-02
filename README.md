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

### Video Demonstration

[Video playlist](goo.gl/iuFF4Q)

<a href="http://www.youtube.com/watch?feature=player_embedded&v=ExR0GADeCvk
" target="_blank"><img src="http://img.youtube.com/vi/ExR0GADeCvk/0.jpg" 
alt="" width="240" height="180" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=kKqS2OkRLQM
" target="_blank"><img src="http://img.youtube.com/vi/kKqS2OkRLQM/0.jpg" 
alt="" width="240" height="180" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=x_AYG531AI
" target="_blank"><img src="http://img.youtube.com/vi/x_AYG531AI/0.jpg" 
alt="" width="240" height="180" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=bTrl3N1obVU
" target="_blank"><img src="http://img.youtube.com/vi/bTrl3N1obVU/0.jpg" 
alt="" width="240" height="180" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=vzleSt7PRU
" target="_blank"><img src="http://img.youtube.com/vi/vzleSt7PRU/0.jpg" 
alt="" width="240" height="180" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=G6mwNYJW07M
" target="_blank"><img src="http://img.youtube.com/vi/G6mwNYJW07M/0.jpg" 
alt="" width="240" height="180" border="10" /></a>

## License

Please contact me via lesun90@gmail.com if you want to use this code for your work.
