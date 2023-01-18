#!/bin/bash
source /opt/ros/galactic/setup.bash

# indicate to the scripts run through 'run_simulation_system' that it is being run in test mode
export TESTING=1

while getopts 'd:s:w:' flag
do
    case "${flag}" in
        d) 
          ros_domain_id=${OPTARG};;
        s)
          python_script_name=${OPTARG};;
        w)
          use_waypoint_server=${OPTARG};;
        ?) 
          echo "script usage: $(basename \$0) [-d ROS_DOMAIN_ID] [is SCRIPT_NAME (system.py|collector_ros_node.py)]" >&2
          exit 1
          ;;
    esac
done

# set the ROS_DOMAIN_ID to be 1 by default
if [ -z $ros_domain_id ];
then
  ros_domain_id=1;
fi

# set the USE_WAYPOINT_SERVER variable to be 1 by default
if [ -z $use_waypoint_server ];
then
  use_waypoint_server=1;
fi

export ROS_DOMAIN_ID=$ros_domain_id
export USE_WAYPOINT_SERVER=$use_waypoint_server
python $python_script_name