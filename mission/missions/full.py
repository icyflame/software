'''
   ___       __                 __     ___  ___ _______
  / _ \___  / /  ___  ___ __ __/ /    |_  |/ _ <  / __/
 / , _/ _ \/ _ \/ _ \(_-</ // / _ \  / __// // / / _ \
/_/|_|\___/_.__/\___/___/\_,_/_.__/ /____/\___/_/\___/

'''

import shm

from mission.opt_aux.aux import *
from mission.framework.combinators import Sequential
from mission.missions.opt import Opt
from mission.missions.gate import gate
from mission.missions.aslam_buoys import AllBuoys as Buoys
from mission.missions.navigate import full as Navigation
from mission.missions.pipe import OptimizablePipe as Pipe
from mission.missions.random_pinger import RandomPinger

PipeToBuoys = OptimizableTask(
  name = 'PipeToBuoys',
  cls = lambda: Pipe(grp = shm.buoys_pipe_results),
  maxTries = 1,
  noProgressKillTime = 30
)

Buoys = OptimizableTask(
  name = 'Buoys',
  cls = Buoys,
  maxTries = 1,
  noProgressKillTime = 120
)

PipeToNavigation = OptimizableTask(
  name = 'PipeToNavigation',
  cls = lambda: Pipe(grp = shm.navigate_pipe_results),
  maxTries = 1,
  noProgressKillTime = 30
)

Navigation = OptimizableTask(
  name = 'Navigation',
  cls = Navigation,
  maxTries = 1,
  noProgressKillTime = 90
)

FirstPinger = OptimizableTask(
  name = 'FirstPinger',
  cls = RandomPinger,
  maxTries = 3,
  noProgressKillTime = 120
)

SecondPinger = OptimizableTask(
  name = 'SecondPinger',
  cls = RandomPinger,
  maxTries = 3,
  noProgressKillTime = 120
)

restrictions = [
  TopologicalRestriction(beforeTask = 'PipeToBuoys', afterTask = 'Buoys'),
  TopologicalRestriction(beforeTask = 'PipeToNavigation', afterTask = 'Navigation')
]

tasks = [
  PipeToBuoys,
  Buoys,
  PipeToNavigation,
  Navigation,
  FirstPinger,
  SecondPinger
]

Full = Opt(tasks = tasks, restrictions = restrictions)

FullSwitched = Sequential(
  gate,
  Full
)

defaultOrdering = tasks

for task in tasks:
  globals()[task.name] = Opt(tasks = [task], restrictions = [])
  
  globals()['After' + task.name] = Opt(
    tasks = defaultOrdering[defaultOrdering.index(task) + 1:],
    restrictions = restrictions,
    alreadyFinishedTasks = [task.name for task in defaultOrdering[:defaultOrdering.index(task) + 1]]
  )
