# AGV Reciever Software
The Receiver Software exists individually on each AGV in our possession, and is the component responsible for sending periodic status updates (AGV Update Rate) to the Waypoint Server, and translating Waypoint Server received tasks and commands into ROS navigation processes through the Simple Commander API.

There are three main organizational entities that comprise the Receiver Software: Internal Memory (`memory.py`), the ROS Location Sampling Node (`collector_ros_node.py`), and the Action Loop (`system.py`). The following documentation contains information on how to start developing on the Receiver Software, and how to run the Receiver Software.

## Getting Started

The following instructions you are using either a Linux or Unix based operating system.

If you are using a Windows operating system, I would highly recommend using WSL2 (Windows Subsystem Linux), as that is the development environment I used while devloping the Waypoint Server. Many [tutorials](https://pureinfotech.com/install-windows-subsystem-linux-2-windows-10/) exist online for installing WSL2.

### Install Git
The [documentation](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) is the best place to start for Git! Pick the installation method that corresponds to your operating system family.

### Clone the Repository Locally

```bash
# navigate to the directory you desire to clone this repository into
git clone https://github.com/APSCL/agv_receiver_system.git
```

### Download ROS Galactic

Follow the instructions provided by the [ROS Galatic Workspace Repository](https://github.com/APSCL/ros_galactic_ws) to install ROS onto your machine.

### (WSL2 Only) Download XLaunch 

WSL2 has no inherent way to connect graphics to a visual display. This is where XLaunch comes in. Download [here](https://sourceforge.net/projects/xming/)

### Dependencies
Because ROS Galactic creates a virtual environment in of itself which we are required to use when running Reciever Software scripts, our team was not able to find a way (as of 2022) to virtualize our development environment. As such, we develop all package depencies directly to the user's root environment.

#### SQLAlchemy
Our way of sharing persitent information between the scripts that comprise Receiver Software
```bash
pip install SQLAlchemy
```

-= Continue adding dependencies to this list as development continues =-

## Running Receiver Software for Simulations

The Receiver Software (and Waypoint Server) can be developed without the presence of physical AGVs with the use of simulated AGVs. Two such simulation environments are the [Turtlesim](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) and [Gazebo](http://classic.gazebosim.org/tutorials?tut=ros2_overview&cat=connect_ros). Below we cover step by step instructions for how to run both environments.

### Turtlesim ROS2
Note, for Turtlesim - one can start as many seperate instances as needed. To enusre that each simulated AGV is a seperate entity from the other, differentiate their `ROS_DOMAIN_ID`s.

#### (WSL2 Only) Enable Graphic Display
Open XLaunch (download it first not already) 

Upon launching, in order to get WSL2 functioning with XLaunch, configure it as such:
- For the **Display Settings** tab, select "Multiple Windows", and set the Display Number as 0
- For the **Client Startup** tab, select "Start no Client"
- For the **Display Settings** tab, unselect "Native opengl" and select "Disable access control

**On the terminal tab used to initialize the simulated AGV**, set the following environment variables:
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
export QT_DEBUG_PLUGINS=1
```

#### Process to Simulate TurtleSim AGV
**Before Running Receiver Software, always be sure to set the correct variables in `config.py`**

1. Open a terminal tab. Enter the following bash commands:
```bash
source /opt/ros/galactic/setup.bash
ROS_DOMAIN_ID=1 ros2 run turtlesim turtlesim_node
```
Remember to set `ROS_DOMIN_ID` differently for each simulated Turtlesim AGV
2. (Optional) Open a terminal tab. Enter the following bash commands: 
```bash
source /opt/ros/galactic/setup.bash
ROS_DOMAIN_ID=1 ros2 run turtlesim turtle_teleop_key
```
3. Open a terminal tab. Enter the following bash commands:
```bash
# navigate to agv_receiver_software repo
cd src 
./run_simulation_system.sh -d 1 -s collector_ros_node.py -w 1
```
The `d` argument indicates the `ROS_DOMAIN_ID` the `collector_ros_node.py` script is being run for. Keep it consistent with the `ROS_DOMAIN_ID` used in previous steps. The `w` argument is used to indicate whether the Waypoint Server will be interfaced with during script execution. **If the Waypoint Server is not running when starting Receiver Software set `-w 0`.**

4. Open a terminal tab. Enter the following bash commands:
```bash
# navigate to agv_receiver_software repo
cd src 
./run_simulation_system.sh -d 1 -s system.py -w 1
```
The `d` argument indicates the `ROS_DOMAIN_ID` the `system.py` script is being run for. Keep it consistent with the `ROS_DOMAIN_ID` used in previous steps. The `w` argument is used to indicate whether the Waypoint Server will be interfaced with during script execution. **If the Waypoint Server is not running when starting Receiver Software set `-w 0`.**

### Gazebo ROS2
**To be added** (it's similar to running the Live Demo)

## Running Receiver Software for Live System Demos
**To be added** (the Receiver Software is heavily integrated into launch instructions for the entire Mutiple AGV Controller system. The 2022 team will craft this documentation upon demonstrating system launch to the 2023 team)

## Resources for Learning Receiver Software
- [ROS2 Publisher and Subscriber Nodes with rclpy Tutorial](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Nav2 Simple Commander API Documentation](https://navigation.ros.org/commander_api/index.html#overview)
- [Brief SQLAlchemy Video Tutorial](https://www.youtube.com/watch?v=NuDSWGOcvtg&ab_channel=DataEngineeringTeam)
- [(2021-2022) Team Final Report](https://docs.google.com/document/d/1PWzXRaNC-ERTDzknO0ewaxypTz_3unFS2LD6ZZ-UfPw/edit?usp=sharing)-Read about Receiver Software

## TODOS
- Coordinate Standardization

## Contact Information
- Ethan Ohman: ethanjohman@utexas.edu