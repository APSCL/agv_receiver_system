import os
import unittest

import rclpy

from core import AGVState, TaskStatus
from memory import Memory


class TestMemory(unittest.TestCase):
    test_waypoints = [(1,1,1,1), (2,2,2,2)] 

    def setUp(self):
        Memory.wipe_database()
    
    def test_create_agv_state(self):
        success, debug_message = Memory.create_agv_state(id=1)
        agv = Memory.get_agv()
        expected_agv_state = {
            "x":0.0,
            "y":0.0,
            "theta":0.0,
            "status":AGVState.READY,
            "current_task":None,
        }
        self.assertEqual(success, True)
        for key, value in expected_agv_state.items():
            self.assertEqual(getattr(agv, key), value)
        
        success, debug_message = Memory.create_agv_state()
        self.assertEqual(success, False)

    def test_get_next_task(self):
        # create complete task
        Memory.create_task(self.test_waypoints, id=1, status=TaskStatus.COMPLETE)
        # create two incomplete tasks
        Memory.create_task(self.test_waypoints, id=2, status=TaskStatus.INCOMPLETE)
        Memory.create_task(self.test_waypoints, id=3, status=TaskStatus.INCOMPLETE)
        task = Memory.get_next_task()
        expected_task_params = {
            "id":2,
            "status":TaskStatus.INCOMPLETE,
        }
        for key, value in expected_task_params.items():
            self.assertEqual(getattr(task, key), value)

    def test_get_next_task_waypoint(self):
        Memory.create_agv_state(id=1)
        # test no next waypoint is returned if it is not registered to the agv
        waypoint = Memory.get_next_task_waypoint()  
        self.assertEqual(waypoint, None)
        # test no next waypoint is returned if it has no waypoints l0l
        Memory.create_task([], id=1, status=TaskStatus.INCOMPLETE)
        Memory.update_agv_current_task(current_task_id=1)
        waypoint = Memory.get_next_task_waypoint() 
        self.assertEqual(waypoint, None)
        # test a waypoint is sucessfully returned
        Memory.create_task(self.test_waypoints, id=2, status=TaskStatus.INCOMPLETE)
        Memory.update_agv_current_task(current_task_id=2)
        waypoint = Memory.get_next_task_waypoint() 
        expected_waypoint_params = {
            "x":1.0,
            "y":1.0,
            "theta":1.0,
            "order":1
        }
        for key, value in expected_waypoint_params.items():
            self.assertEqual(getattr(waypoint, key), value)

        Memory.update_waypoint(id=waypoint.id, visited=True)
        waypoint = Memory.get_next_task_waypoint()  
        expected_waypoint_params = {
            "x":2.0,
            "y":2.0,
            "theta":2.0,
            "order":2
        }
        for key, value in expected_waypoint_params.items():
            self.assertEqual(getattr(waypoint, key), value)
    
    def test_update_agv_current_task(self):
        Memory.create_agv_state(id=1)
        task_id = 1
        Memory.create_task([], id=task_id, status=TaskStatus.INCOMPLETE)

        invalid_task_id = -1
        success, debug_message = Memory.update_agv_current_task(current_task_id=invalid_task_id)
        self.assertEqual(success, False)

        success, debug_message = Memory.update_agv_current_task(current_task_id=task_id)
        self.assertEqual(success, True)

        agv = Memory.get_agv()
        self.assertEqual(agv.current_task.id, task_id)



    def test_update_task(self):
        task_id = 1
        Memory.create_task([], id=task_id, status=TaskStatus.INCOMPLETE)

        success, debug_message = Memory.update_task(id=None, status=TaskStatus.IN_PROGRESS)
        self.assertEqual(success, False)
        invalid_task_id = -1
        success, debug_message = Memory.update_task(id=invalid_task_id, status=TaskStatus.IN_PROGRESS)
        self.assertEqual(success, False)
        success, debug_message = Memory.update_task(id=task_id, status="WRONG VALUE")
        self.assertEqual(success, False)

        success, debug_message = Memory.update_task(id=task_id, status=TaskStatus.IN_PROGRESS)
        self.assertEqual(success, True)
        task = Memory.get_all_tasks()[0]
        self.assertEqual(task.status, TaskStatus.IN_PROGRESS)
        
        
    def test_update_waypoint(self):
        task_id, waypoint_id = 1, 1
        Memory.create_task([self.test_waypoints[0]], id=task_id, status=TaskStatus.INCOMPLETE)

        success, debug_message = Memory.update_waypoint(id=None, visited=True)
        self.assertEqual(success, False)
        invalid_task_id = -1
        success, debug_message = Memory.update_waypoint(id=invalid_task_id, visited=True)
        self.assertEqual(success, False)

        success, debug_message = Memory.update_waypoint(id=waypoint_id, visited=True)
        self.assertEqual(success, True)
        waypoint = Memory.get_all_waypoints()[0]
        self.assertEqual(waypoint.visited, True)

    def test_update_agv_state(self):
        Memory.create_agv_state(id=1)
        update_params = {
            "x":1.0,
            "y":2.0,
            "theta":3.0,
            "status":AGVState.READY,
        }
        Memory.update_agv_state(**update_params)
        agv = Memory.get_agv()
        for key, value in update_params.items():
            self.assertEqual(getattr(agv, key), value)

    def tearDown(self):
        Memory.wipe_database()

if __name__ == "__main__":
    unittest.main()
