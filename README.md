# CoopMultiRobotsPlanning
This paper develops an effective, cooperative, and probabilistically-complete multi-robot motion planner.

The approach takes into account geometric and differential constraints imposed by the obstacles and the robot

dynamics by using sampling to expand a motion tree in the composite state space of all the robots. Scalability and efficiency is achieved by using solutions to a simplified problem representation that does not take dynamics into account to guide the motion-tree expansion. The heuristic solutions are obtained by constructing roadmaps over low-dimensional configuration

spaces and relying on cooperative multi-agent graph search to effectively find graph routes. Experimental results with second-order vehicle models operating in complex environments, where cooperation among the robots is required to find solutions, demonstrate significant improvements over related work.

## Getting Started
![webinterface](/pics/webinterface.png?raw=true "webinterface")
### Hardware Setup
- A temperature sensor is connected to arduindo.

- Arduino connected to Raspberry PI via cable.

- A switch button connected to Raspberry PI (pull down).

- A led is connected to indicate status of switch button (optional).

![schematic](/pics/schematic.jpg?raw=true "schematic")
### Prerequisites
```
//install nodejs
sudo apt-get update
sudo apt-get dist-upgrade
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs

```

<!---//install socket and express
npm init
npm install socket.io express --save
//install serialport
npm install serialport
sudo npm install serialport --unsafe-perm --build-from-source
//install Raspberry module for nodejs
npm install onoff
-->

### Running
On the server sie (Raspberry Pi)

```
cd /path_to_project_folder
node app.js
```

On the client side (must connect to the same network of Raspberry), run the following
code to get the Raspberry IP address

```
//Look for pi ip address
nmap -sn 192.168.1.0/24
```

Open web browser, type in that address with the port number (port number defined in app.js), eg:
```
192.168.1.9:5000
```

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments
