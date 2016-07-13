#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors

#blender/urdf_to_blender.py -p rbprmBuilder/ -i /local/mcampana/devel/hpp/src/animals_description/urdf/skeleton.urdf -o skeleton_blend.py


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'frog_trunk'
urdfNameRoms = ['FrogLFootSphere','FrogRFootSphere','FrogLHandSphere','FrogRHandSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-4.5, 3.6, -6.3, 4.9, -0.15, 2.7] # etang # all rocks
#base_joint_xyz_limits = [-2.3, 1.5, -3.9, 3.7, -0.15, 2.7]# etang juste middle

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
filterRange = -1
rbprmBuilder.setNormalFilter('FrogLFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('FrogRFootSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('FrogLHandSphere', [0,0,1], filterRange)
rbprmBuilder.setNormalFilter('FrogRHandSphere', [0,0,1], filterRange)
rbprmBuilder.setContactSize (0.03,0.08)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation
rbprmBuilder.setNumberFilterMatch(4)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
r.loadObstacleModel ('hpp-rbprm-corba', "etang_envir", "etang_envir")
addLight (r, [0,0,6,1,0,0,0], "li");
plotFrame (r, "frameGroupName", [0,0,0], 0.5)

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [-1.44, 2.78, -0.11, 1, 0, 0, 0]; r(q11) # etang
#q11[0:7] = [-3.2, 0, 0.05, 1, 0, 0, 0]; r(q11) # groundcrouch

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [-0.35, -3.45, 0.035, 1, 0, 0, 0]; r(q22) # etang

rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(5)
rbprmBuilder.setMaxLandingVelocity(9)
ps.clearRoadmap();

waypoints = [q11,[0.7568593392134035, 2.5001776108010154, -0.016627106595112656, 0.7669881208375589, 0.021052203660743424, -0.05214600253234064, 0.639192319755951, 0.0, 0.0, 0.0, -0.05307791519517043, -0.09895622889977937, 0.9936751982818474, 1.3913848017526247], [0.1874940972479516, 0.14929621880323268, 0.004508710268820125, 0.9452909290642296, 0.037779238914688505, 0.009565348237867363, 0.32389240906356687, 0.0, 0.0, 0.0, 0.04255689125452609, -0.06522845633742118, 0.9969624664402306, 0.65995381762137], [-0.6038954269369067, -2.0266558500396945, 0.04205823660655854, 0.5282008965036383, -0.0006806265025383335, 0.004078508951508424, -0.8491093660094966, 0.0, 0.0, 0.0, 0.005464396845288899, -0.006207185242508982, 0.9999658050245931, -2.028633313001989],q22] # limVelocity = 5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs ()


ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True,False,True)
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


plotConeWaypoints (ps, solutionPathId, r, "cone_rb_wp_group", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_22", "friction_cone2")



# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("frog_spond_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()


# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -6.5; qAway[1] = -2
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


