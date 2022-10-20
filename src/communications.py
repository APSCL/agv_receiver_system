import json
from http import HTTPStatus

import requests

from config import TEST_WAYPOINT_SERVER_IP, WAYPOINT_SERVER_IP, WAYPOINT_SERVER_PORT
from core import AGVState, CommandTypes
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
            "status": str(agv.status),
        }
        response = requests.post(
            url=f"http://{WAYPOINT_SERVER_IP}:{WAYPOINT_SERVER_PORT}/agvs/",
            data=registration_payload,
        )
        if response.ok:
            return response.status_code, response.json()
        return response.status_code, response.text

    @classmethod
    def get_task(self):
        agv = Memory.get_agv()
        agv_update_json = {
            "id": agv.id,
            "x": agv.x,
            "y": agv.y,
            "theta": agv.theta,
            "status": str(agv.status),
            "current_task_id": None,
            "current_waypoint_order": None,
        }
        response = requests.get(
            url=f"http://{WAYPOINT_SERVER_IP}:{WAYPOINT_SERVER_PORT}/agv_request_handlers/request_task/",
            json=agv_update_json,
        )
        if response.ok:
            return response.status_code, response.json()
        return response.status_code, response.text

    @classmethod
    def send_internal_state_update(self):
        agv = Memory.get_agv()
        next_waypoint = Memory.get_next_task_waypoint()
        registration_payload = {
            "id": agv.id,
            "x": agv.x,
            "y": agv.y,
            "theta": agv.theta,
            "status": str(agv.status),
            "current_task_id": agv.current_task.id if agv.current_task is not None else None,
            "current_waypoint_order": next_waypoint.order if next_waypoint is not None else None,
        }
        response = requests.post(
            url=f"http://{WAYPOINT_SERVER_IP}:{WAYPOINT_SERVER_PORT}/agv_request_handlers/update_state/",
            json=registration_payload,
        )

        if response.ok:
            return response.status_code, response.json()
        return response.status_code, response.text


class MockWaypointServerCommunicator:
    # Potentially used for testing
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
    def no_tasks_available(cls, status_code):
        if HTTPStatus.ACCEPTED == status_code:
            return True
        return False

    @classmethod
    def no_tasks_available(cls, status_code):
        if HTTPStatus.OK != status_code:
            return True
        return False

    @classmethod
    def process_new_task(cls, task_json):
        task_id, waypoints = task_json.get("id"), task_json.get("waypoints")
        parsed_waypoints = []
        for waypoint in waypoints:
            parsed_waypoints.append((waypoint.get("x"), waypoint.get("y"), waypoint.get("theta"), waypoint.get("order")))

        # it's important to create it with the task id from the waypoint server, so it can be identified later!
        Memory.create_task(parsed_waypoints, id=task_id)
        task = Memory.get_next_task()
        Memory.update_agv_state(status=AGVState.BUSY)
        Memory.update_agv_current_task(current_task_id=task.id)

    @classmethod
    def process_command(cls, response_json):
        command_json = response_json.get("command", None)
        if command_json is None:
            return False

        command_type = command_json.get("type", None)
        if command_type is None:
            return False

        command_type_to_processing_function = {
            str(CommandTypes.CANCEL_AGV): cls.process_cancel_task,
            str(CommandTypes.CANCEL_TASK): cls.process_cancel_task,
            str(CommandTypes.STOP_AGV): cls.process_stop_agv,
            str(CommandTypes.START_AGV): cls.process_start_agv,
        }
        processing_function = command_type_to_processing_function.get(command_type, None)
        if processing_function is None:
            return False
        processing_function()
        return True
    
    @classmethod
    def process_cancel_task(cls):
        # remove the current task from the database (because it may have to exist within the AGV's internal database again)
        Memory.delete_task(current_task=True)
        # set the AGV to recieve a new task
        Memory.update_agv_state(status=AGVState.READY)
        # set the AGV to no longer possess a "current_task"
        Memory.update_agv_current_task(current_task_id=None)


    @classmethod
    def process_stop_agv(cls):
        # remove the current task from the database (because it may have to exist within the AGV's internal database again)
        Memory.delete_task(current_task=True)
        # set the AGV to no longer possess a "current_task"
        Memory.update_agv_current_task(current_task_id=None)
        # set AGV into the locked "STOPPED" state cycle
        Memory.update_agv_state(status=AGVState.STOPPED)

    @classmethod
    def process_start_agv(cls):
        # free AGV from "STOPPED" state cycle, and prime the AGV to receive tasks again!
        Memory.update_agv_state(status=AGVState.READY)



class MockWaypointJSONParser:
    @classmethod
    def no_tasks_available(cls, task_json):
        return False

    @classmethod
    def process_new_task(cls, task_json):
        # waypoints must be a [(x, y, waypoint_order_int)]
        Memory.create_task(task_json)
        task = Memory.get_next_task()
        Memory.update_agv_state(status=AGVState.BUSY)
        Memory.update_agv_current_task(current_task_id=task.id)

    @classmethod 
    def process_command(cls, response_json):
        return False
