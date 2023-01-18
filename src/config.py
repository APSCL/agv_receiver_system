from core import AGVDriveTrainType

# the LAN-based IP of the machine the Waypoint Server is currently running on
WAYPOINT_SERVER_IP = "192.168.0.100"
# the PORT through which the Waypoint Server is being hosted on
WAYPOINT_SERVER_PORT = "5000"
# the manualy set ID to distingish each AGV in the Multiple AGV Controller System
AGV_ID = 1
# the drive train type of the AGV housing the Receiver Software
AGV_DRIVE_TRAIN = AGVDriveTrainType.ACKERMANN

# the period in seconds by which the AGV samples pertinent informaton from its ROS Stack (ie - location, direction, etc)
SAMPLE_PERIOD_SECONDS = 2.5

# TODO: Implement some form of coordinate standardization 
COORDINATE_STANDARIZATION_ENABLED = False

# the depth of which to subscribe to node history. This is a recommend configuration from ROS Docs, it's unlikely that it needs to be changed
QOS_PROFILE = 10

# the period at which an iteration of the action loop (defined in system.py) runs at. Decrease for shorter intervals between AGV to Waypoint Server communication
INTER_ITERATION_PERIOD_SECONDS = 5

# internal database name definitions
DATABASE_BASENAME = "sqlite:///states/robot1.db"
TESTING_DATABASE_NAME = "sqlite:///tests/testing.db"
