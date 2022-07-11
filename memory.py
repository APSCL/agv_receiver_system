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

from config import DATABASE_BASENAME
from core import AGVState, MemoryAccessMessages, TaskStatus

Base = declarative_base()


class AGV(Base):
    __tablename__ = "agv"
    # WILL BE THE ROS_DOMAIN_ID
    id = Column(Integer, primary_key=True)
    x = Column(Float)
    y = Column(Float)
    theta = Column(Float)
    status = Column(Enum(AGVState), default=AGVState.READY, nullable=False)
    # lazy = joined is eager loading and crucial to ensuring that we have access to objects outside of sessions
    current_task = relationship("Task", backref="assigned_agv", uselist=False, lazy="joined")

    def __init__(self, id=None, x=None, y=None, theta=None, status=None, current_task=None):
        if id is not None:
            self.id = id
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.theta = theta if theta is not None else 0.0
        if current_task is not None:
            self.current_task = current_task
        # else:
        #     self.current_task = None
        if status is not None:
            self.status = status
        else:
            self.status = None

    def __repr__(self):
        return f"AGV | ID:{self.id} | X:{self.x} Y:{self.y} THETA: {self.theta}| STATUS: {str(self.status)}"


class Task(Base):
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
    __tablename__ = "waypoint"
    id = Column(Integer, primary_key=True)
    order = Column(Integer)
    visited = Column(Boolean)
    x = Column(Float)
    y = Column(Float)
    task_id = Column(Integer, ForeignKey("task.id"))

    def __init__(self, id=None, x=None, y=None, order=None, visited=None):
        if id is not None:
            self.id = id
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.order = order if order is not None else 0
        self.visited = visited if visited is not None else False

    def __repr__(self):
        return f"WAYPOINT | ID:{self.id} | TASK:{self.task_id} | X:{self.x} Y:{self.y} | ORDER:{self.order} | VISITED:{self.visited}"


# https://stackoverflow.com/questions/25156264/sqlalchemy-using-decorator-to-provide-thread-safe-session-for-multiple-function
def thread_safe_db_access(func):

    ros_domain_id = (
        os.environ.get("ROS_DOMAIN_ID", None)
        if os.environ.get("ROS_DOMAIN_ID", None) is not None
        else 1
    )
    agv_memory_db_name = f"{DATABASE_BASENAME}_{ros_domain_id}"
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


# TODO: Figure out how to create sessions and then close them!!
# TODO: Description - two different instances
# TODO: First Draft - EDIT LATER (maybe make each funtion a dynamic memory query instead!! (create filter statemetns with kwarsg etc))
# TODO: Document what @thread_safe_db_access does
class Memory:
    # Comment, internal methods that work INSIDE of sessions will have the '-' suffix
    @classmethod
    @thread_safe_db_access
    def wipe_database(cls, session=None):
        session.query(AGV).delete()
        session.query(Waypoint).delete()
        session.query(Task).delete()

    # Creation Methods
    @classmethod
    @thread_safe_db_access
    def create_agv_state(cls, id=None, session=None):
        # ensure there only exists one definition of the AGV at a time!
        agv = session.query(AGV).first()
        if agv is not None:
            return
        if id is not None:
            agv = AGV(id=id)
        else:
            agv = AGV()
        session.add(agv)

    @classmethod
    @thread_safe_db_access
    def create_task(cls, waypoints, id=None, status=TaskStatus.INCOMPLETE, session=None):
        # for now, waypoints will be [(x,y,order)], before I assign them to a serialzer!
        if id is not None:
            task = Task(id=id, status=status)
        else:
            task = Task(status=status)
        session.add(task)
        for waypoint in waypoints:
            x, y, order = waypoint
            task.waypoints.append(Waypoint(x=x, y=y, order=order, visited=False))
            session.merge(task)

    # Retrieval Methods - expose objects outside of sessions (OUTSIDE, NOT INTERNAL GET)
    @classmethod
    @thread_safe_db_access
    def get_agv(cls, session=None):
        agv = session.query(AGV).first()
        current_task = agv.current_task

        # expose agv and its current task (with associated waypoints) object for external session access
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

        # expose task waypoints
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
            print(f"Task registered to AGV with id: {current_task.id} could not be found")
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
                print(f"Task with id:{current_task_id} does not exist!")
                return
            agv.current_task = task
            session.merge(agv)
        else:
            agv.current_task = None
            session.merge(agv)

    @classmethod
    @thread_safe_db_access
    def update_task(cls, id=None, status=None, session=None):
        if id is None:
            print("Task ID not provided")
            return
        current_task = session.query(Task).filter_by(id=id).first()
        if current_task is None:
            print("Invalid Task ID provided")
            return
        if type(status) is not TaskStatus:
            print("[status] provided must be of type: TaskStatus")
            return
        current_task.status = status

    @classmethod
    @thread_safe_db_access
    def update_waypoint(cls, id=None, visited=False, session=None):
        if id is None:
            print("Waypoint ID not provided")
            return
        waypoint = session.query(Waypoint).filter_by(id=id).first()
        if waypoint is None:
            print("Invalid Waypoint ID provided")
            return
        waypoint.visited = visited

    @classmethod
    def _expunge_query_statement(cls, session=None, query=None):
        """
        Bread and butter for exposing objects for external use in memory!
        """
        if session is None or query is None:
            print(f"session: {session} | query: {query} must both be defined!")
            return
        if type(query) is not list:
            print("query must be a list, not a model/object")
            return
        for obj in query:
            session.expunge(obj)
