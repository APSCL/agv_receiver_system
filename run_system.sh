source /opt/ros/galactic/setup.bash
# set environment variable for ROS_DOMAIN_ID to be i or something (for different parts!)

while getopts 'd:s:' flag
do
    case "${flag}" in
        d) 
          ros_domain_id=${OPTARG};;
        s)
          python_script_name=${OPTARG};
          echo $python_script_name
          if ["$python_script_name" != "system.py" && "$python_script_name" != "collector_ros_node.py"]; 
          then
            echo "script usage: $(basename \$0) [-d ROS_DOMAIN_ID] [is SCRIPT_NAME (system.py|collector_ros_node.py)]" >&2
            exit 1
          fi
          ;;
        ?) 
          echo "script usage: $(basename \$0) [-d ROS_DOMAIN_ID] [is SCRIPT_NAME (system.py|collector_ros_node.py)]" >&2
          exit 1
          ;;
    esac
done

if [ -z $ros_domain_id ];
then
  ros_domain_id=1;
fi

export ROS_DOMAIN_ID=$ros_domain_id
python $python_script_name
