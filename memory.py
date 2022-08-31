from enum import Enum

import pandas as pd
from sqlaclhemy.orm import relationship, sessionmaker
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

from core import AGVState, BaseEnum, TaskStatus

Base = declarative_base()

class AGV(Base):
    __tablename__ = "agv"
    id = Column(Integer, primary_key=True)
    x = Column(Float)
    y = Column(Float)
    status = Column(
        Enum(AGVState), 
        default=AGVState.READY,
        nullable=False
    )
    current_task = relationship(
        "Task",
        backref="assigned_agv",
        uselist=False
    )

    def __init__(self, x=None, y=None, status=None, current_task=None):
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        if current_task is not None:
            self.current_task = current_task
        if status is not None:
            self.status = status

    def __repr__(self):
        return f"AGV | ID:{self.id} | X:{self.x} Y:{self.y} | STATUS: {str(self.status)}"    

class Task(Base):
    __tablename__ = "task"
    id = Column(Integer, primary_key=True)
    waypoints = relationship(
        "Waypoint", 
        backref="task"
    )
    status = Column(
        Enum(TaskStatus), 
        default=TaskStatus.IN_PROGRESS, 
        nullable=False
    )
    agv_id = Column(Integer, ForeignKey('agv.id'))

    @property
    def num_waypoints(self):
        if not self.waypoints:
            return 0
        return len(self.waypoints)

    def __init__(self, status=None):
        if status is not None:
            self.status = status

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

    def __init__(self, x=None, y=None, order=None, visited=None):
        self.x = x if x is not None else 0.0
        self.y = y if y is not None else 0.0
        self.order = order if order is not None else 0
        self.visited = visited if visited is not None else False

    def __repr__(self):
        return f"WAYPOINT | ID:{self.id} | TASK:{self.task_id} | X:{self.x} Y:{self.y} | ORDER:{self.order} | VISITED:{self.visited}"   

class AGVMemory(BaseEnum):
    IN_MEMORY = "sqlite:///:memory:"
    SHARED_MEMORY = "sqlite:///state.db"

# TODO: Consider Renaming!
# TODO: Description - two different instances 
class InternalState:
    def __init__(self, db_name=str(AGVMemory.SHARED_MEMORY), wipe_memory=False):
        engine = create_engine(db_name)
        Base.metadata.create_all(bind=engine)
        Session = sessionmaker(bind=engine)
        self.db_name = db_name
        self.session = Session()

        # create internal representation of AGV
        if wipe_memory:
            Base.metadata.drop_all(bind=engine)
            Base.metadata.create_all(bind=engine)
            self.create_agv_state()

        self.agv = self.get_agv()

    def update_memory(self):
        self.session.commit()

    def create_agv_state(self):
        agv = AGV()
        self.session.add(agv)
        self.update_memory()

    def current_task_exists(self):
        current_task_id = self.get_agv_state().current_task
        if current_task_id is None:
            return False
        return True

    def get_agv(self):
        return AGV.query.first()

    def get_current_task_waypoints(self, only_unvisited=False):
        if not self.current_task_exists():
            return []
        current_task_id = self.agv.current_task
        filter_kwargs = {"task_id":current_task_id}
        if only_unvisited:
            filter_kwargs["visited"] = False
        waypoints =  Waypoint.query.filter_by(**filter_kwargs).order_by(Waypoint.order.asc()).all()
        return waypoints

    def get_current_task_current_waypoint(self):
        if not self.current_task_exists():
            return None
        waypoints = self.get_current_task_waypoints(only_unvisited=True)
        if waypoints:
            return waypoints[0]
        return None

    def update_agv_state(self, x=None, y=None, status=None, current_task=None):
        if x is not None:
            self.agv.x = x
        if y is not None:
            self.agv.y = y
        if status is not None:
            self.agv.status = status
        if current_task is not None:
            self.agv.current_task = current_task

        self.update_memory()

    def update_agv_current_task_status(self, status=None):
        task = self.agv.current_task
        if task is None:
            return 
        if type(status) is not TaskStatus:
            return
        task.status = status
        
        self.update_memory()

    def update_current_waypoint(self, visited=False):
        current_waypoint = self.get_current_task_current_waypoint()
        if current_waypoint is None:
            return
        current_waypoint.visited = visited

        self.update_memory()

    def close(self):
        self.session.close()
