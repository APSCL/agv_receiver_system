import json
from http import HTTPStatus

import requests

from config import TEST_WAYPOINT_SERVER_IP, WAYPOINT_SERVER_IP, WAYPOINT_SERVER_PORT
from core import AGVState
from memory import Memory


class WaypointServerCommunicator:
    @classmethod
    def register_agv(cls):
        print("Attempting to register AGV to the waypoint server...")
        agv = Memory.get_agv()
        registration_payload = {
            "id": agv.id,
            "x": agv.x,
            "y": agv.y,
            "theta": agv.theta,
            "status": agv.status,
        }
        response = requests.post(
            url=f"http://{WAYPOINT_SERVER_IP}:{WAYPOINT_SERVER_PORT}/agvs/",
            data=registration_payload,
        )

        print(response.status_code, response.json())

    @classmethod
    def get_task(self):
        pass

    @classmethod
    def send_internal_state_update(self):
        print("Sending Waypoint Server an internal update...")
        pass


class MockWaypointServerCommunicator:
    # used for testing!
    @classmethod
    def register_agv(cls):
        print("Attempting to register AGV to the waypoint server...")
        return True

    @classmethod
    def get_task(cls):
        waypoints = [(1, 1, 1), (5.5, 5.5, 2), (11, 1, 3)]
        return HTTPStatus.OK, waypoints

    @classmethod
    def send_internal_state_update(cls):
        print("Sending Waypoint Server an internal update...")
        pass


class WaypointJSONParser:
    @classmethod
    def no_tasks_available(cls, task_json):
        if task_json.get("task") == None:
            return True
        return False

    @classmethod
    def process_new_task(cls, task_json):
        pass


class MockWaypointJSONParser:
    @classmethod
    def no_tasks_available(cls, task_json):
        return False

    @classmethod
    def process_new_task(cls, task_json):
        # waypoints must be a [(x, y, waypoint_order_int)]
        # parse and respond to output
        # - Create task with waypoints in Internal State
        # - Assign the Task to the AGV
        # - Change and update the internal state of the AGV to BUSY
        Memory.create_task(task_json)
        task = Memory.get_next_task()
        Memory.update_agv_state(status=AGVState.BUSY)
        Memory.update_agv_current_task(current_task_id=task.id)
