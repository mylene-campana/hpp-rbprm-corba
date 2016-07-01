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

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
#rbprmBuilder.setJointBounds ("base_joint_xyz", [-4, 5, -2, 2, -0.1, 2.7]) # etang
rbprmBuilder.setJointBounds ("base_joint_xyz", [-2, 4, -1.5, 1.5, -0.1, 2.7])
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
rbprmBuilder.setNumberFilterMatch(2)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
#r.loadObstacleModel ('hpp-rbprm-corba', "etang_envir", "etang_envir")
r.loadObstacleModel ('hpp-rbprm-corba', "groundcrouch", "planning")
addLight (r, [0,0,6,1,0,0,0], "li");

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
#q11[0:7] = [-1.4, 2.85, -0.125, 1, 0, 0, 0]; r(q11) # etang
q11[0:7] = [-3.2, 0, 0.05, 1, 0, 0, 0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
#q22[0:7] = [-0.3, -3.4, 0.02, 1, 0, 0, 0]; r(q22) # etang
q22[0:7] = [3, 0, 0.05, 1, 0, 0, 0]; r(q22)

rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner") # "PRMplanner"
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(4.3)#(8)
rbprmBuilder.setMaxLandingVelocity(8)
ps.clearRoadmap();
ps.setInitialConfig (q11); ps.addGoalConfig (q22)

t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

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

plotConeWaypoints (rbprmBuilder, solutionPathId, r, "cone_wp_group", "friction_cone2")
plotCone (q11, rbprmBuilder, r, "cone_11", "friction_cone2"); plotCone (q22, rbprmBuilder, r, "cone_21", "friction_cone2")



# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("frog_spond_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

rob = rbprmBuilder.client.basic.robot
r(q11)

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -6.5; qAway[1] = -2
rbprmBuilder.setCurrentConfig (qAway); r(qAway)


