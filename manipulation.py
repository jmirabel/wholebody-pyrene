#/usr/bin/env python
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import newProblem, ProblemSolver, \
    Constraints, ConstraintGraph, Rule
from hpp.gepetto import PathPlayer
from hpp.gepetto.manipulation import ViewerFactory
from hpp import Transform
import CORBA, sys, numpy as np
from common import generateRandomConfig, writeTrajInYaml

newProblem()

Robot.packageName = "talos_data"
Robot.urdfName = "talos"
Robot.urdfSuffix = '_full'
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

qq = [-0.7671778026566639, 0.0073267002287253635, 1.0035168727631776, -0.003341673452654457, -0.021566597515109524, 0.0002183620894239602, 0.9997618053357284, -0.00020053128587844821, -0.0021365695604276275, -0.4415951773681094, 0.9659230706255528, -0.48119003672520416, 0.007109157982067145, -0.00020095991543181877, -0.002126639473414498, -0.4382848597339842, 0.9589221865248464, -0.4774994711722908, 0.007099218648561522, 0.0, 0.025235347910697536, -0.06985947194357875, 0.0, -0.12446173084176845, -1.5808415926365578, 0.014333078135875619, -0.0806417043955706, -0.37124401660668394, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.25955282922987977, -0.1618313202464181, 0.002447883426630002, -0.5149037074691503, -0.00010703274362664899, 0.0008742582163227642, 0.10168585913285667, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.785398163397, 0.32250041955468695, -0.2569883469655496, 0.18577095561452217, 1.164388709412583, 0.0694401264431558, 0.5475575114527793, -0.11842286843715424, 0.8254301089264399]

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

# ps.addNumericalConstraints ("balance + locked grippers", footPlacement +
#                             ['balance/relative-com'])

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

# ps.addLockedJointConstraints ("balance + locked grippers",
#                               left_gripper_lock + right_gripper_lock)

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
