#/usr/bin/env python
from math import sqrt
from hpp.corbaserver.manipulation import ProblemSolver, Constraints, \
    ConstraintGraph, ConstraintGraphFactory, Rule
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from hpp import Transform
import CORBA, sys, numpy as np
from common import generateRandomConfig, writeTrajInYaml, robot, ps, \
    Box, Table, vf

objects = list ()
objects.append (Box (name = 'box', vf = vf))
robot.setJointBounds ("box/root_joint", [-1, 1, -1, 1, 0, 2])

# Loaded as an object to get the visual tags at the right position.
table = Table (name = 'table', vf = vf)

q_init = [0.6000022397992977, -0.6781023282014177, 1.0227731536350761, -0.026501735815611785, 0.026519221784192294, 0.7066117724333544, 0.7066075232640208, -7.425303327690214e-06, -3.104979264370574e-05, -0.5336970777582547, 0.7998833793330343, -0.3411868362719986, -0.001701417910656863, -7.425312934111446e-06, -3.104992049186117e-05, -0.5337106744153133, 0.799914920807006, -0.341204781088912, -0.0017014177824483088, 0.0, 0.5558260964981129, 0.28504014455571497, 0.19401590944614103, 0.003941049852447548, -0.5228672465592601, -6.095997813993427e-05, 0.001283145908800703, 0.10028131572105346, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.28502261804473694, -0.1940282332221976, -0.0038866982587039012, -0.5226794809165282, 0.00011731350732173029, -0.00127685849617846, 0.10046668823166613, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6504553908999274, -5.56003086594079e-06, 0.6, -0.11, 0.8375, 0, sqrt(2)/2, 0, sqrt(2)/2]

# Gaze constraint
ps.createPositionConstraint ("gaze", "talos/rgbd_optical_joint",
                             "box/root_joint", (0,0,0), (0,0,0),
                             (True, True, False))

# Constraint of constant yaw of the waist
ps.createOrientationConstraint ("waist orientation", "", "talos/root_joint",
                                (0,0,0,1), [True, True, True])
ps.setConstantRightHandSide ("waist orientation", False)
# Create static stability constraints
ps.addPartialCom ("talos", ["talos/root_joint"])
ps.createStaticStabilityConstraints ("balance", q_init, "talos", ProblemSolver.FIXED_ON_THE_GROUND)
footPlacement = [ "balance/pose-left-foot", "balance/pose-right-foot" ]
footPlacementComplement = [ ]
quasiStaticConstraints = footPlacement + ['balance/relative-com']

# Lock grippers in open positions
left_gripper_lock = []
right_gripper_lock = []
other_lock = ["talos/torso_1_joint"]
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith ("talos/gripper_right"):
        ps.createLockedJoint(n, n, q_init[r:r+s])
        right_gripper_lock.append(n)
    elif n.startswith ("talos/gripper_left"):
        ps.createLockedJoint(n, n, q_init[r:r+s])
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, q_init[r:r+s])

cg = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory (cg)
factory.setGrippers ([ "talos/left_gripper", "talos/right_gripper"])
factory.setObjects ([ obj.name for obj in objects],
                    [ obj.handles for obj in objects ],
                    [ obj.contacts for obj in objects])
factory.environmentContacts (table.contacts)
factory.generate ()
for edgename, edgeid in cg.edges.iteritems():
        cg.addConstraints (edge = edgename, constraints = Constraints
                           (numConstraints = ["waist orientation"]))
cg.setConstraints (graph=True,
                   constraints = Constraints \
                   (numConstraints = quasiStaticConstraints + ['gaze'],
                    lockedJoints = left_gripper_lock + right_gripper_lock +\
                    other_lock))
cg.initialize ()

q_goal = q_init [::]
q_goal [-3] *= -1
# Project initial configuration on initial node
res, q_init, err = cg.applyNodeConstraints ('free', q_init)
if not res: raise RuntimeError ('Failed to project initial configuration')
res, q_goal, err = cg.applyNodeConstraints ('free', q_goal)
if not res: raise RuntimeError ('Failed to project goal configuration')
ps.setInitialConfig(q_init)
ps.resetGoalConfigs ()
ps.addGoalConfig(q_goal)
ps.setParameter ('SimpleTimeParameterization/order', 2)
ps.setParameter ('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter ('SimpleTimeParameterization/safety', 0.5)
ps.solve()
pid = ps.numberPaths () - 1
if pid >= 0: writeTrajInYaml (ps, pid, '/tmp/traj.yaml', .1)

# To display the resulting paths
# run gepetto-gui in another terminal,
#
# v = vf.createViewer ()
# pp = PathPlayer (v)
# pp (i) displays the i-th path in the viewer.
# each call to solve generates 3 paths
#  - result of RRT,
#  - optimization via random shortcut,
#  - time parameterization.

# to discretize a give path, use method
#  ps.configAtParam (i, t)
# where
#   - i is the path id,
#   - t is a number between 0 and
#   - ps.pathLength (i)

