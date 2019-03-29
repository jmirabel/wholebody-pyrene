#/usr/bin/env python
from hpp.corbaserver.manipulation import ProblemSolver, Constraints, \
    ConstraintGraph, Rule
from hpp.gepetto import PathPlayer
from common import generateRandomConfig, writeTrajInYaml, robot, ps, vf

half_sitting = [
        -0.74,0,1.0192720229567027,0,0,0,1, # root_joint
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_left
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_right
        0, 0.006761, # torso
        0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, # arm_left
        0, 0, 0, 0, 0, 0, 0, # gripper_left
        -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, # arm_right
        0, 0, 0, 0, 0, 0, 0, # gripper_right
        0, 0, # head
        ]

# Create waist orientation constraint
robot.setCurrentConfig (half_sitting)
waistPose = robot.getJointPosition ("talos/root_joint")
ps.createOrientationConstraint ('waist orientation', '', 'talos/root_joint',
                                waistPose [3:], 3 * [True,])
# Create static stability constraints
ps.addPartialCom ("talos", ["talos/root_joint"])
ps.createStaticStabilityConstraints ("balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND)
footPlacement = [ "balance/pose-left-foot", "balance/pose-right-foot" ]
footPlacementComplement = [ ]
quasiStaticConstraints = footPlacement + ['balance/relative-com', 'waist orientation']

# Lock grippers in open positions
left_gripper_lock = []
right_gripper_lock = []
other_lock = ["talos/torso_1_joint"]
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith ("talos/gripper_right"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        right_gripper_lock.append(n)
    elif n.startswith ("talos/gripper_left"):
        ps.createLockedJoint(n, n, half_sitting[r:r+s])
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r:r+s])

cg = ConstraintGraph (robot, 'graph')
cg.createNode (['static stability'])
cg.createEdge (nodeFrom = 'static stability', nodeTo = 'static stability',
               name = 'move', weight = 1, isInNode = 'static stability')
cg.setConstraints (node = 'static stability', constraints =
                   Constraints (numConstraints = quasiStaticConstraints,
                                lockedJoints = left_gripper_lock +
                                right_gripper_lock))
cg.initialize ()
q_init = half_sitting [::]
q_goal = generateRandomConfig (robot, cg)
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
