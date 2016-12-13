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
base_joint_xyz_limits = [-4.5, 6.8, -8.8, 14.3, -6.9, 3.2]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", [-4.5, 6.8, -8.8, 14.3, -6.9, 3.2])
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
rbprmBuilder.setAffordanceFilter('LFFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('LMFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('LBFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RFFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RMFootSphere', affordanceType)
rbprmBuilder.setAffordanceFilter('RBFootSphere', affordanceType)
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "cave"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [-3,0,8,1,0,0,0], "li");
addLight (r, [-1,15,0,1,0,0,0], "li2");

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [0.99, -5.4, -3.08, 1, 0, 0, 0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [1.15, 12.28, -1.51, 1, 0, 0, 0]; r(q22)

rbprmBuilder.isConfigValid(q22)

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance
rbprmBuilder.setNumberFilterMatch(3)
ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(8)
rbprmBuilder.setMaxLandingVelocity(11)
ps.clearRoadmap();

waypoints = [q11,[0.1988379215748521, 1.8186672873534242, -5.292452719944538, 0.912813944840402, 0.177395575286985, 0.3678122140630282, -0.0039607019019601555, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6700830141738334, -0.3267718988214978, 0.6664899701093056, 0.16741496846337983], [3.6397593607277514, 1.5112407775121763, -2.9527686860172633, 0.8306097558483304, -0.3087155298944409, -0.021082246226291255, 0.4629661909747033, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.32087194472144126, 0.4933235273592653, 0.8085005209920971, 0.9406367354708023], [-0.9192356007650062, 5.388406216073009, -2.4986747122562813, 0.7724304651394216, -0.1943192726962285, 0.46102638113198297, -0.3912107778214364, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8642612316730184, -0.06052072592908628, 0.49938939231831275, -1.2402983236382978], [2.7607200549147444, 8.122970264523747, -2.7542783837549423, 0.4519150149288567, -0.2112528996274209, -0.26985650890048574, 0.8236033610208862, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.5918816128139278, -0.25357274086982506, 0.7650993540060979, 2.1002160041817346], [-2.683560646503113, 9.738073491923686, -2.2024921249453624, 0.1358664531052861, 0.5155435900327132, 0.2664172688061241, 0.8029800449442416, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9003367689077387, 0.2877653429619063, 0.32647329131613434, 2.289886689181118],q22] # limVelocity = 4.5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()

ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True) # <- rotate after jump
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

rbprmBuilder.rotateAlongPath (solutionPathId)
orientedpathIdBis = ps.numberPaths () - 1
#pp(orientedpathIdBis)

V0list = rbprmBuilder.getsubPathsV0Vimp("V0",solutionPathId)
Vimplist = rbprmBuilder.getsubPathsV0Vimp("Vimp",solutionPathId)

print("-- Verify that all RB-waypoints are valid (solution path): ")
pathWaypoints = ps.getWaypoints(solutionPathId)
for i in range(1,len(pathWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriWaypoints = ps.getWaypoints(orientedpathId)
for i in range(1,len(pathOriWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))

print("-- Verify that all RB-waypoints are valid (oriented path): ")
pathOriBisWaypoints = ps.getWaypoints(orientedpathIdBis)
for i in range(1,len(pathOriBisWaypoints)-1):
    if(not(rbprmBuilder.isConfigValid(pathOriBisWaypoints[i])[0])):
        print('problem with waypoints number: ' + str(i))


#plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group_planning", "friction_cone2")
#plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")


"""
# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("ant_cave_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

rob = rbprmBuilder.client.basic.robot
r(q11)

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[1] = -11
rbprmBuilder.setCurrentConfig (qAway); r(qAway)
"""
