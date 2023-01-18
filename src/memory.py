import os
from enum import Enum

from sqlalchemy import (
    Boolean,
    Column,
    DateTime,
    Enum,
    Float,
    ForeignKey,
    Integer,
    create_engine,
)
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship, scoped_session, sessionmaker

from config import DATABASE_BASENAME, TESTING_DATABASE_NAME
from core import AGVDriveTrainType, AGVState, TaskStatus

Base = declarative_base()


class AGV(Base):
    """
    This model serves as an internal representation for each running and registered AGV (Autonomous Guided Vehicle)
    in the Multiple AGV System.

    id : A unique identifier for the AGV housing the Receiver Software, this is set manually in config.py
    drive_train_type : the drive train model the AGV possesses
    x : the current x-coordinate position of the AGV
    y : the current y-coordinate position of the AGV
    theta : the current rotational direction of the AGV (in radians)
    status : the current part of the navigation process the AGV is currently in (can be READY, BUSY, DONE, STOPPED)
    current_task : denotes the Task the AGV is currently executing
    """

    __tablename__ = "agv"
    id = Column(Integer, primary_key=True)
    drive_train_type = Column(Enum(AGVDriveTrainType), default=AGVDriveTrainType.ACKERMANN, nullable=False)
    x = Column(Float)
    y = Column(Float)
    theta = Column(Float)
    status = Column(Enum(AGVState), default=AGVState.READY, nullable=False)
    # lazy = joined is eager loading and crucial to ensuring that we have access to objects outside of sessions
    current_task = relationship("Task", backref="assigned_agv", uselist=False, lazy="joined")

    def __init__(self, id=None, drive_train_type=None, x=None, y=None, theta=None, status=None, current_task=None):
        if id is not None:
            self.id = id
        self.drive_train_type = drive_train_type if drive_train_type is not None else AGVDriveTrainType.ACKERMANN
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.theta = theta if theta is not None else 0.0
        if current_task is not None:
            self.current_task = current_task
        if status is not None:
            self.status = status
        else:
            self.status = None

    def __repr__(self):
        return f"AGV | ID:{self.id} | X:{self.x} Y:{self.y} 0: {self.theta}| STATUS: {str(self.status)}"


class Task(Base):
    """
    This model servers as the internal representation for user created tasks. In contrast to the Waypoint Server's definition of Tasks,
    the Receiver Software does not need to have information related to the priority or drive train assignment associated with the task.
    The Receiver Software will simply execute all tasks provided to it on a "first-come-first-serve" basis

    id : A unique identifier for each Task
    waypoints : relational link to the waypoints (locations on a 2D plane) associated with this task
    status : indicates which stage of processing a Task is in (ie - INCOMPLETE, IN_PROGRESS, COMPLETE)
    agv_id : indicates which AGV this task is assigned to
    """

    __tablename__ = "task"
    id = Column(Integer, primary_key=True)
    waypoints = relationship("Waypoint", backref="task", lazy="joined")
    status = Column(Enum(TaskStatus), default=TaskStatus.INCOMPLETE, nullable=False)
    agv_id = Column(Integer, ForeignKey("agv.id"))

    @property
    def num_waypoints(self):
        if not self.waypoints:
            return 0
        return len(self.waypoints)

    def __init__(self, id=None, status=None):
        if id is not None:
            self.id = id
        if status is not None:
            self.status = status
        else:
            self.status = TaskStatus.INCOMPLETE

    def __repr__(self):
        return f"TASK | ID:{self.id} | STATUS: {str(self.status)} | NUM_WAYPOINTS:{self.num_waypoints}"


class Waypoint(Base):
    """
    This model servers as the internal representation for user created waypoints (effectively coordinates)

    id : A unique identifier for each Waypoint
    x : the x-coordinate position of the Waypoint
    y : the y-coordinate position of the Waypoint
    theta : the rotational direction of the Waypoint (in radians)
    order : indicates when this Waypoint should be visited (in comparison to other waypoints in the same Task)
    visited : indicates whether the Waypoint was navigated to by the AGV
    task_id : indicates which Task this waypoint belongs to
    """

    __tablename__ = "waypoint"
    id = Column(Integer, primary_key=True)
    order = Column(Integer)
    visited = Column(Boolean)
    x = Column(Float)
    y = Column(Float)
    theta = Column(Float)
    task_id = Column(Integer, ForeignKey("task.id"))

    def __init__(self, id=None, x=None, y=None, theta=None, order=None, visited=None):
        if id is not None:
            self.id = id
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.theta = theta if theta is not None else 0.0
        self.order = order if order is not None else 0
        self.visited = visited if visited is not None else False

    def __repr__(self):
        return f"WAYPOINT | ID:{self.id} | TASK:{self.task_id} | X:{self.x} Y:{self.y} 0:{self.theta} | ORDER:{self.order} | VISITED:{self.visited}"


def thread_safe_db_access(func):
    """
    The following function serves as a wrapper to all Receiver Software internal database functions. The wrapper make internal
    database accesses atomic and concurrent, allowing for multiple scripts to interface with the database. 
    The inspiration for this solution is provided by the stack overflow post below:

    https://stackoverflow.com/questions/25156264/sqlalchemy-using-decorator-to-provide-thread-safe-session-for-multiple-function
    """

    ros_domain_id = (
        os.environ.get("ROS_DOMAIN_ID", None)
        if os.environ.get("ROS_DOMAIN_ID", None) is not None
        else 1
    )
    agv_memory_db_name = f"{DATABASE_BASENAME}_{ros_domain_id}" if not os.environ.get("TESTING", False) else f"{TESTING_DATABASE_NAME}_{ros_domain_id}"
    engine = create_engine(agv_memory_db_name)
    Base.metadata.create_all(bind=engine)
    session_factory = sessionmaker(bind=engine)
    Session = scoped_session(session_factory)

    def inner(*args, **kwargs):
        session = Session()
        result = None
        try:
            result = func(*args, **kwargs, session=session)
            session.commit()
        except:
            session.rollback()
        finally:
            Session.remove()
        return result

    return inner


class Memory:
    """
    Memory is the interface for the Receiver Software's internal memory. By implementing this as a 
    database, we are able to update and share information across python scripts.
    """
    
    @classmethod
    @thread_safe_db_access
    def wipe_database(cls, session=None):
        session.query(AGV).delete()
        session.query(Waypoint).delete()
        session.query(Task).delete()

    @classmethod
    @thread_safe_db_access
    def create_agv_state(cls, id=None, session=None):
        """
        Creates the internal definition of the AGV housing the Receiver Software. Note, there will only ever
        exist one AGV database entry for each AGV in the Multiple AGV Controller System.
        """

        agv = session.query(AGV).first()
        if agv is not None:
            return False, "AGV definition already exists within internal memory"
        if id is not None:
            agv = AGV(id=id)
        else:
            agv = AGV()
        session.add(agv)
        return True, "AGV Creation Success"

    @classmethod
    @thread_safe_db_access
    def create_task(cls, waypoints, id=None, status=TaskStatus.INCOMPLETE, session=None):
        """
        'waypoints' will be in the format [(x, y, theta, order), ... ]. I was too lazy to add python type descriptions.
        If any future teams want to kill time in the lab, add types to all the funcitons in the Receiver Software with Python Typing:

        https://docs.python.org/3/library/typing.html
        """

        if id is not None:
            task = Task(id=id, status=status)
        else:
            task = Task(status=status)
        session.add(task)
        for waypoint in waypoints:
            x, y, theta, order = waypoint
            task.waypoints.append(Waypoint(x=x, y=y, theta=theta, order=order, visited=False))
            session.merge(task)
        return True, "Task Creation Success"

    @classmethod
    @thread_safe_db_access
    def delete_task(cls, id=None, current_task=False, session=None):
        if id is not None or current_task is True:
            agv = session.query(AGV).first()
            id = agv.current_task.id if agv.current_task is not None else None
        if id is None:
            return
        task_to_delete = session.query(Task).filter_by(id=id).first()
        if task_to_delete is None:
            return
        for waypoint in task_to_delete.waypoints:
            session.delete(waypoint)
        session.delete(task_to_delete)

    @classmethod
    @thread_safe_db_access
    def get_agv(cls, session=None):
        agv = session.query(AGV).first()
        current_task = agv.current_task

        session.expunge(agv)
        if current_task is not None:
            session.expunge(current_task)
            task_waypoints = session.query(Waypoint).filter_by(task_id=current_task.id).all()
            cls._expunge_query_statement(session=session, query=task_waypoints)
        return agv

    @classmethod
    @thread_safe_db_access
    def get_next_task(cls, session=None):
        task = (
            session.query(Task)
            .filter_by(status=TaskStatus.INCOMPLETE)
            .order_by(Task.id.asc())
            .first()
        )
        task_waypoints = session.query(Waypoint).filter_by(task_id=task.id).all()
        session.expunge(task)
        cls._expunge_query_statement(session=session, query=task_waypoints)
        return task

    @classmethod
    @thread_safe_db_access
    def get_all_tasks(cls, session=None):
        tasks = session.query(Task).all()
        cls._expunge_query_statement(session=session, query=tasks)
        return tasks

    @classmethod
    @thread_safe_db_access
    def get_next_task_waypoint(cls, session=None):
        agv = session.query(AGV).first()
        current_task = agv.current_task
        if current_task is None:
            return None
        unvisited_ordered_waypoints = (
            session.query(Waypoint)
            .filter_by(task_id=current_task.id, visited=False)
            .order_by(Waypoint.order.asc())
            .all()
        )
        if len(unvisited_ordered_waypoints) == 0:
            return None
        next_waypoint = unvisited_ordered_waypoints[0]
        session.expunge(next_waypoint)
        return next_waypoint

    @classmethod
    @thread_safe_db_access
    def get_all_waypoints(cls, session=None):
        waypoints = session.query(Waypoint).all()
        cls._expunge_query_statement(session=session, query=waypoints)
        return waypoints
        
    @classmethod
    @thread_safe_db_access
    def update_agv_state(
        cls,
        id=None,
        x=None,
        y=None,
        theta=None,
        status=None,
        session=None,
    ):
        agv = session.query(AGV).first()
        if id is not None:
            agv.id = id
        if x is not None:
            agv.x = x
        if y is not None:
            agv.y = y
        if theta is not None:
            agv.theta = theta
        if status is not None:
            agv.status = status

    @classmethod
    @thread_safe_db_access
    def update_agv_current_task(cls, current_task_id=None, session=None):
        agv = session.query(AGV).first()
        if current_task_id is not None:
            task = session.query(Task).filter_by(id=current_task_id).first()
            if task is None:
                return False, "Task with provided id does not exist!"
            agv.current_task = task
            session.merge(agv)
            return True, None
        
        agv.current_task = None
        session.merge(agv)
        return True, None

    @classmethod
    @thread_safe_db_access
    def update_task(cls, id=None, status=None, session=None):
        if id is None:
            return False, "Task ID not provided"
        current_task = session.query(Task).filter_by(id=id).first()
        if current_task is None:
            return False, "Invalid Task ID provided"
        if type(status) is not TaskStatus:
            return False, "[status] provided must be of type: TaskStatus"
        current_task.status = status
        return True, None

    @classmethod
    @thread_safe_db_access
    def update_waypoint(cls, id=None, visited=False, session=None):
        if id is None:
            return False, "Waypoint ID not provided"
        waypoint = session.query(Waypoint).filter_by(id=id).first()
        if waypoint is None:
            return False, "Invalid Waypoint ID provided"
        waypoint.visited = visited
        return True, None

    @classmethod
    def _expunge_query_statement(cls, session=None, query=None):
        """
        Allows objects (a query) referenced in a SQLAlchemy client database connection to be used outside of said connection.
        """
        if session is None or query is None:
            print(f"session: {session} | query: {query} must both be defined!")
            return
        if type(query) is not list:
            print("query must be a list, not a model/object")
            return
        for obj in query:
            session.expunge(obj)
