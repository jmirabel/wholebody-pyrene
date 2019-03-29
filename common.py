import numpy as np
from numpy.linalg import norm
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import newProblem, ProblemSolver
from hpp.gepetto.manipulation import ViewerFactory
from hpp import Transform
import CORBA, sys, numpy as np

# Generate a random valid configuration
def generateRandomConfig (robot, cg):
    res = False
    while not res:
        q = robot.shootRandomConfig ()
        res, q1, err = cg.applyNodeConstraints ('static stability', q)
        if res: res, msg = robot.isConfigValid (q1)
    return q1

# Write path in yaml file
# pid id of path to write,
# filename yaml file
# dt difference between consecutive sample times
# Note velocity is computed analytically
def writeTrajInYaml (ps, pid, filename, dt):
    with open (filename, 'w') as f:
        T = ps.pathLength (pid)
        t = 0
        finished = 0
        while finished < 2:
            q = ps.configAtParam (pid, t)
            v = ps.derivativeAtParam (pid, 1, t)
            a = ps.derivativeAtParam (pid, 2, t)
            f.write ('time: {0}\n'.format (t))
            f.write ('  - q: {0}\n'.format (q))
            f.write ('  - v: {0}\n'.format (v))
            f.write ('  - a: {0}\n'.format (a))
            t += dt
            if t > T:
                t = T
                finished += 1

# Compute velocity by finite difference along path
#  pid id of path,
#  t time along path
# Note: quaternion part is irrelevant
def finiteDifference (ps, pid, t):
    dt = 1e-4
    if t+dt > ps.pathLength (pid): dt = -dt
    q0 = np.array (ps.configAtParam (pid, t))
    q1 = np.array (ps.configAtParam (pid, t+dt))
    v = (q1 - q0) / dt
    return v

# Compare velocity provided by method derivativeAtParam with finite difference
# pid id of path,
# dt sampling of times where velocity is checked.
def checkVelocity (ps, pid, dt):
    T = ps.pathLength (pid)
    t = 0
    M = 0
    finished = 0
    while finished < 2:
        v0 = np.array (ps.derivativeAtParam (pid, 1, t))
        v1 = finiteDifference (pid, t)
        err = norm (v1 [:3] - v0 [:3])
        if err > M: M = err
        err = norm (v1 [7:] - v0 [6:])
        if err > M: M = err
        t += dt
        if t > T:
            t = T
            finished += 1
    return M

class Box (object):
  def __init__ (self, name, vf) :
    self.name = name
    self.handles = [ name + "/" + h for h in self.__class__.handles ]
    self.contacts = [ name + "/" + h for h in self.__class__.contacts ]
    vf.loadObjectModel (self.__class__, name)

  rootJointType = 'freeflyer'
  packageName = 'gerard_bauzil'
  urdfName = 'plank_of_wood1'
  urdfSuffix = ""
  srdfSuffix = ""
  handles = ["handle1", "handle2", "handle3", "handle4"]
  contacts = [ "front_surface", "rear_surface", ]

class Table (object):
  def __init__ (self, name, vf) :
    self.name = name
    self.handles = [ name + "/" + h for h in self.__class__.handles ]
    self.contacts = [ name + "/" + h for h in self.__class__.contacts ]
    vf.loadObjectModel (self.__class__, name)

  rootJointType = 'anchor'
  packageName = 'gerard_bauzil'
  urdfName = 'table_140_70_73'
  urdfSuffix = ""
  srdfSuffix = ""
  pose = "pose"
  handles = []
  contacts = [ "top", ]

newProblem()

Robot.packageName = 'talos_data'
Robot.urdfName = 'talos'
Robot.urdfSuffix = '_full_v2'
Robot.srdfSuffix= ''

robot = Robot ('dev', 'talos', rootJointType = "freeflyer")
robot. leftAnkle = "talos/leg_left_6_joint"
robot.rightAnkle = "talos/leg_right_6_joint"

robot.setJointBounds ("talos/root_joint", [-1, 1, -1, 1, 0.5, 1.5])

ps = ProblemSolver (robot)
ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
ps.setErrorThreshold (1e-3)
ps.setMaxIterProjection (40)

ps.addPathOptimizer("RandomShortcut")
ps.addPathOptimizer("SimpleTimeParameterization")

vf = ViewerFactory (ps)

