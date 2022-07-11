# build in memory data base to track:
# - current state (database field)
#   - AGV ID, Status, current task, current pose, currr
# - pose for each 5 second time-stamp : ()
# - current task
# - visited waypoints (point to waypoints)
import enum as Enum
import json
from datetime import datetime

import pandas as pd
from sqlaclhemy.orm import relationship, sessionmaker
from sqlalchemy import Column, DateTime, Integer, String, Text, create_engine
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()


class AGVState(Base):
    __tablename__ = "agv_state"
    id = Column(Integer, primary_key=True)
