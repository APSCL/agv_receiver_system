import rclpy

from core import TaskStatus
from memory import Memory
from ros_turtlesim_nav import TurtleSimNavigationPublisher

# For now, this will serve as a space where I can test operations of the reciever software in a safe, clean environment


def main():
    # print("Testing Begins...")
    # Memory.wipe_database()
    # # test agv retrieval
    # print("Testing AGV Creation/Retrieval...")
    # Memory.create_agv_state()
    # agv = Memory.get_agv()
    # print(agv)
    # print(agv.current_task)

    # # test create new task
    # print("Testing Task Creation/Retrieval...")
    # waypoints = [(1, 2, 1), (1, 2, 2), (1, 2, 3)]
    # Memory.create_task(waypoints)
    # Memory.create_task(waypoints, status=TaskStatus.IN_PROGRESS)
    # next_task = Memory.get_next_task()
    # all_tasks = Memory.get_all_tasks()
    # print(next_task)
    # print(next_task.waypoints)
    # print(all_tasks)

    # # test adding a task
    # print("Testing Add Task to AGV...")
    # next_task = Memory.get_next_task()
    # next_task_id = next_task.id
    # Memory.update_agv_state(current_task_id=next_task_id)
    # agv = Memory.get_agv()
    # print(agv.current_task.waypoints[0].id)

    # Test Navigating to a waypoint [WORKS]
    rclpy.init()
    nav = TurtleSimNavigationPublisher()
    agv = Memory.get_agv()
    print(agv)
    x, y, theta = agv.x, agv.y, agv.theta
    goal_x = 10.0
    goal_y = 10.0
    nav.navigate_to(x, y, theta, goal_x, goal_y)


if __name__ == "__main__":
    main()
