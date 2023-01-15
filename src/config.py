WAYPOINT_SERVER_IP = "192.168.0.100"
WAYPOINT_SERVER_PORT = "5000"
TEST_WAYPOINT_SERVER_IP = "10.19.4.19"
AGV_ID = 1

# how often to recieve updates from turtle bot, and therefore write to in memeory DB
SAMPLE_PERIOD_SECONDS = 2.5

COORDINATE_STANDARIZATION_ENABLED = False

# depth of which to subscribe to node history
QOS_PROFILE = 10

# for system controller
INTER_ITERATION_PERIOD_SECONDS = 5

# internal database name
DATABASE_BASENAME = "sqlite:///states/robot1.db"
TESTING_DATABASE_NAME = "sqlite:///tests/testing.db"
