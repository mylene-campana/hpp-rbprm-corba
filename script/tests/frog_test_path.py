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
#base_joint_xyz_limits = [-2, 4, -1.5, 1.5, -0.1, 2.7] # groundcrouch
#base_joint_xyz_limits = [-4.5, 3.6, -6.3, 4.9, -0.15, 2.7] # etang # all rocks
base_joint_xyz_limits = [-2.3, 1.5, -3.9, 3.7, -0.15, 2.7]# etang juste middle

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
#r.loadObstacleModel ('hpp-rbprm-corba', "groundcrouch", "planning")
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
#q22[0:7] = [3, 0, 0.05, 1, 0, 0, 0]; r(q22) # groundcrouch

rbprmBuilder.isConfigValid(q22)

ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
rbprmBuilder.setFrictionCoef(1.2)
rbprmBuilder.setMaxTakeoffVelocity(5)
rbprmBuilder.setMaxLandingVelocity(8)
ps.clearRoadmap();

"""waypoints = [q11,[-3.236940676716824, 3.587263164594118, -0.031030685272660577, -0.40254762220279544, 0.0026270768545143975, 0.03291138096925263, 0.9148034495610369, 0.0, 0.0, 0.0, 0.28093839855197145, 0.19251431480903344, 0.9402190461869309, -2.3134272354646073], [-3.5213747621473828, 1.8634612763516036, -0.027148527383034082, 0.998854034171292, 0.013001822884624054, 0.03785955712161055, 0.026234041929668217, 0.0, 0.0, 0.0, 0.07631452345915303, -0.023987428061840725, 0.9967952130724647, 0.0535686617290101], [-3.5344398096208267, 1.5014551468965347, -0.026143838101702774, 0.7116951702770786, 0.21568045121392423, 0.302645961298539, -0.5961353451005243, 0.0, 0.0, 0.0, -0.09301730275174097, 0.173100163376574, 0.9805019708434013, -1.4241429490876314], [-3.2865415076817235, -0.2813711637813963, -0.030100726341916892, 0.6685609523853411, 0.012831182468145972, -0.006184693398673308, -0.7435209232225003, 0.0, 0.0, 0.0, 0.21362369909331763, -0.09979801342057044, 0.9718051613893576, -1.676723716790911], [-3.2904158948336417, -1.1740914013129748, -0.053038737477100154, -0.3869268469228203, -0.005799025383538642, -0.0474805127087669, 0.9208689306018952, 0.0, 0.0, 0.0, 0.0260626855384192, -0.09193425513885975, 0.9954239444350274, -2.348011625362226], [-3.3478766626551044, -2.9809591056314635, -0.049262346356701306, 0.7516607799814529, -0.03605971861309553, 0.00802573560139602, -0.658514507128943, 0.0, 0.0, 0.0, 0.05955695702356245, 0.043639225789461174, 0.9972705685232017, -1.4377322584575263], [-2.852307202027884, -4.497664685916467, -0.04563354250209943, 0.5804172458560064, 0.017443100435285915, -0.08802070838155023, -0.8093601879609243, 0.0, 0.0, 0.0, 0.16140240785278995, 0.3423074556927834, 0.9256213418652655, -1.903412516834896], [-1.7816477653763647, -4.688098651606694, -0.04906760277055558, 0.9939334656961017, 0.029509920317280617, 0.04051005676766457, -0.09789977360997562, 0.0, 0.0, 0.0, 0.07475057320064095, -0.0665936455197043, 0.9949761998070967, -0.1941583427418596],q22] # limVelocity = 4.5
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); ps.solve (); ps.resetGoalConfigs()"""

ps.setInitialConfig (q11); ps.addGoalConfig (q22)


t = ps.solve ()
solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp(solutionPathId)

rbprmBuilder.rotateAlongPath (solutionPathId,True)
orientedpathId = ps.numberPaths () - 1
#pp(orientedpathId)

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


plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group", "friction_cone2")
plotCone (q11, ps, r, "cone_11", "friction_cone2"); plotCone (q22, ps, r, "cone_21", "friction_cone2")



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


