import json

import requests

from config import TEST_WAYPOINT_SERVER_IP, WAYPOINT_SERVER_IP


class WaypointServerCommunicator:
    def __init__(self, test_mode=True):
        self.ip = TEST_WAYPOINT_SERVER_IP if test_mode else WAYPOINT_SERVER_IP
    
    def register_agv(self):
        pass

 
    def get_task(self):
        pass

    def send_internal_state_update(self):
        pass
