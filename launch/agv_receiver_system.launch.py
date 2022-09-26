from re import I
import subprocess

from launch import LaunchDescription

def generate_launch_description():
    subprocess.Popen(['./src/run_system.sh', '-s', './src/collector_ros_node.py', '>', '/dev/null', '2>', '/dev/null'])
    subprocess.Popen(['./src/run_system.sh', '-s', './src/system.py', '>', '/dev/null', '2>', '/dev/null'])

    return LaunchDescription()

