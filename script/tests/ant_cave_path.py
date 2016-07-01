#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a ant-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 6 end-effectors


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'ant_trunk'
urdfNameRoms = ['LFFootSphere','LMFootSphere','LBFootSphere','RFFootSphere','RMFootSphere','RBFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-6, 0, -2, 2, 0.001, 2.4])
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
filterRange = 0.3
rbprmBuilder.setNormalFilter('LFFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('LMFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('LBFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RFFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RMFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('RBFootSphere', [0,0,1], filterRange)
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
rbprmBuilder.setNumberFilterMatch(4)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ('hpp-rbprm-corba', "cave", "cave")
addLight (r, [-3,0,8,1,0,0,0], "li");
addLight (r, [-1,15,0,1,0,0,0], "li2");

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [0.8, -8.3, -4.55, 1, 0, 0, 0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [0.1, 14.2, -2.8, 1, 0, 0, 0]; r(q22)

rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(5)
rbprmBuilder.setMaxLandingVelocity(10)
ps.clearRoadmap();
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])

rbprmBuilder.rotateAlongPath (solutionPathId)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

V0list = rbprmBuilder.getsubPathsV0Vimp("V0",solutionPathId)
Vimplist = rbprmBuilder.getsubPathsV0Vimp("Vimp",solutionPathId)

print("Verify that all RB-waypoints are valid: ")
pathWaypoints = ps.getWaypoints(solutionPathId)
for i in range(1,len(pathWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))


plotConeWaypoints (rbprmBuilder, solutionPathId, r, "cone_wp_group", "friction_cone_2")
plotCone (q11, rbprmBuilder, r, "cone_11", "friction_cone2"); plotCone (q22, rbprmBuilder, r, "cone_21", "friction_cone2")



# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("ant_test_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

rob = rbprmBuilder.client.basic.robot
r(q11)

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[1] = -11
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


