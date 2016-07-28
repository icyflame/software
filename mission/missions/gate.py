import shm

from mission.framework.combinators import Sequential
from mission.framework.movement import Depth, Heading
from mission.framework.position import MoveX
from mission.missions.start import WaitForUnkill

gate = Sequential(WaitForUnkill(), Depth(1.0), MoveX(7))
