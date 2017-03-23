#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.
# The script launches a skeleton-robot and a groundcrouch environment.
# It defines init and final configs, and solve them with RBPRM.
# Range Of Motions are spheres linked to the 4 end-effectors


from hpp.corbaserver import Client
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.gepetto import Viewer, PathPlayer
import math
from viewer_library import *

rootJointType = 'freeflyer'
packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
urdfName = 'lamp_trunk'
urdfNameRoms = ['LampFootSphere']
urdfSuffix = ""
srdfSuffix = ""
ecsSize = 4
base_joint_xyz_limits = [-7.6, 8.7, -4.2, 3.1, -0.7, 10.]

rbprmBuilder = Builder () # RBPRM
rbprmBuilder.loadModel(urdfName, urdfNameRoms, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
rbprmBuilder.setJointBounds ("base_joint_xyz", base_joint_xyz_limits)
rbprmBuilder.boundSO3([-3.14,3.14,-3.14,3.14,-3.14,3.14])
rbprmBuilder.setFilter(urdfNameRoms)
affordanceType = ['Support']
rbprmBuilder.setAffordanceFilter('LampFootSphere', affordanceType)
rbprmBuilder.setContactSize (0.03,0.03)
rbprmBuilder.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
rbprmBuilder.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

ps = ProblemSolver (rbprmBuilder)
r = Viewer (ps); gui = r.client.gui
r(rbprmBuilder.getCurrentConfig ())

pp = PathPlayer (rbprmBuilder.client.basic, r)
obstacleName = "high_plateforms"
r.loadObstacleModel (packageName, obstacleName, obstacleName+"_obst")
addLight (r, [0,0,6,1,0,0,0], "li");

from hpp.corbaserver.affordance.affordance import AffordanceTool
afftool = AffordanceTool ()
afftool.setAffordanceConfig('Support', [2.5, 0.001, 0.1]) # error, angle and area   # default (0.3,0.3,0.05)
afftool.setNeighbouringTriangleMargin ('Support', 0.01)
afftool.loadObstacleModel (packageName, obstacleName, obstacleName+"_affordance", r)
afftool.visualiseAffordances('Support', r, [0.25, 0.5, 0.5])

ps.client.problem.selectPathValidation("RbprmPathValidation",0.05) # also configValidation; call after loading obstacles for affordance

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [7.3, 2, 7.8, math.sqrt(2)/2.0, 0, 0, -math.sqrt(2)/2.0]; r(q11)

rbprmBuilder.isConfigValid(q11)

q22 = q11[::]
q22[0:7] = [-6.3, 0, 0.27, 0, 0, 0, -1]; r(q22)

rbprmBuilder.isConfigValid(q22)


ps.selectPathPlanner("BallisticPlanner")
ps.client.problem.selectConFigurationShooter("RbprmShooter")
ps.selectSteeringMethod("SteeringParabola")
rbprmBuilder.setFullOrientationMode(True) # RB-shooter follow obstacle-normal orientation
frictionCoef = 1.; rbprmBuilder.setFrictionCoef(frictionCoef)
rbprmBuilder.setMaxTakeoffVelocity(8)
rbprmBuilder.setMaxLandingVelocity(9)
ps.clearRoadmap();


waypoints = [q11, [6.70395813792722, 2.059510746288534, 7.735575725114418, 0.6655992578417569, -0.014731814778191304, -0.003355764319008728, 0.746156378006433, -0.02645166359316184, 0.01460312006662316, 0.9995434249583526, 1.6845803154134376], [7.93, -0.01, 9.6, 0.6904413066201038, -0.1526132435692385, -0.690441306620104, -0.1526132435692385, -0.9068367915491051, 0.4214819491899088, 0.0, -0.16459098358604524], [5.86, -0.46, 8.4, 0.0572014556837969, 0.7047893255914526, -0.0572014556837969, 0.7047893255914528, 0.9869119738706186, -0.1612599014969303, 0.0, -3.022288231683334], [7.38, 0.61, 6.23, -0.42476825815207947, -0.44618445120090244, -0.09071529987927915, 0.7824717865462053, -0.6211875294856867, -0.5210143098295263, 0.5853803397487238, -1.9622816921083248], [5.91862079453954, -0.15223220421913342, 5.694772667847127, 0.05720131874475821, 0.7047893367055581, -0.05720131874475821, 0.7047893367055582, 0.9869120365354422, -0.16125951798720534, 0.0, 0.3344176002996635], [8.052491028021553, -1.4169642122470576, 5.823801643153638, 0.7070320196344704, 0.010282179321618222, -0.7070320196344704, 0.010282179321618222, -0.9995771071535923, -0.029079320048030082, 0.0, -0.7731504658431239], [5.8762828168389225, -1.7026854591130354, 5.075815122501059, -0.16449873145525473, 0.687706454346338, 0.16449873145525473, 0.6877064543463381, 0.891760669398448, 0.45250735741425463, 0.0, 2.462905946128218], [7.732313286122967, -0.7299990550343898, 5.044619090950858, 0.660313632790753, 0.2529543562595409, -0.660313632790753, 0.2529543562595409, -0.7440563745972852, -0.668116839647935, 0.0, -2.3981909882431856], [5.555, -1.075, 4.779, -0.16449873145525473, 0.687706454346338, 0.16449873145525473, 0.6877064543463381, 0.891760669398448, 0.45250735741425463, -0.0, 2.909693512808919], [7.265, -0.175, 3.7, 0.6603135694731884, 0.25295452154404496, -0.6603135694731884, 0.25295452154404496, -0.744056040121693, -0.6681172121405234, 0.0, 0.5814454586651089], [6.05, -2.29, 0.25, 0.05969563672257324, -0.032126710349116495, 0.2560399285069903, 0.9642861921901432, -0.03138975326303659, 0.4976271842774687, 0.8668228589845586, 3.0423286733146426], [1.44, -1.59, -0.2, 0.795220776333782, -0.2336416068001339, 0.1732336372807046, 0.5320015257247352, 0.026922592470073572, 0.5559144385673098, 0.8308034129726979, 1.1162387069938193], [-1.83, 1.45, -0.152, -0.15175502880932354, -0.07856877072626303, -0.2643865509282215, 0.9491559993941587, -0.06890406289389353, -0.5257345740754558, 0.8478533999096857, -2.8855156140797025], q22]
#q = waypoints[4]; conesWP = rbprmBuilder.getContactCones (q); plotContactCones (conesWP, ps, r, "qConesWPtest", "friction_cone2"); qAway = q11 [::]; qAway[0] = -6.5; r(qAway); conesWP
for i in range(0,len(waypoints)-1):
    ps.setInitialConfig (waypoints[i]); ps.addGoalConfig (waypoints[i+1]); t = ps.solve (); t; ps.resetGoalConfigs(); pp.displayPath(ps.numberPaths () - 1, [0.0, 0.0, 0.8, 1.0])

#ps.setInitialConfig (waypoints[12]); ps.addGoalConfig (waypoints[13]);t = ps.solve (); t; ps.resetGoalConfigs()   # DEBUG !!!!!!
#ps.setInitialConfig (waypoints[13]); ps.addGoalConfig (waypoints[14]);t = ps.solve (); t; ps.resetGoalConfigs()   # DEBUG !!!!!!
#ps.setInitialConfig (waypoints[12]); ps.addGoalConfig (waypoints[14])
#ps.setInitialConfig (waypoints[13]); ps.addGoalConfig (waypoints[14])


ps.setInitialConfig (q11); ps.addGoalConfig (q22)

print("start solve")
t = ps.solve (); t


solutionPathId = ps.numberPaths () - 1
pp.displayPath(solutionPathId, [0.0, 0.0, 0.8, 1.0])
#pp.speed = 3; pp(solutionPathId)

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


#plotConeWaypoints (ps, solutionPathId, r, "cone_wp_group_planning", "friction_cone1")  # gui.writeNodeFile('cone_wp_group_planning','conesWP_COM_lampPlateforms.dae')
#plotCone (q11, ps, r, "cone_11", "friction_cone1"); plotCone (q22, ps, r, "cone_21", "friction_cone1")
# r.client.gui.writeNodeFile('cone_11','Cone_start_mu1_lampPlateforms.dae'); r.client.gui.writeNodeFile('cone_21','Cone_goal_mu1_lampPlateforms.dae')

# Move RB-robot away in viewer
qAway = q11 [::]; qAway[0] = -7; rbprmBuilder.setCurrentConfig (qAway); r(qAway)

"""
r.startCapture ("problem_afforance_plateforms","png"); r.stopCapture()
# Write data to log file
pfr = rbprmBuilder.getResultValues ()
if isinstance(t, list):
    timeSec = t[0]* 3600000 + t[1] * 60000 + t[2] * 1000 + t[3]
f = open('log.txt', 'a')
f.write("ant_plateforms_path\n")
f.write("path computation: " + str(timeSec) + "\n")
f.write("parabola fail results: " + str(pfr) + "\n" + "\n")
f.close()

##### Blender

pathSamples = plotSampleSubPath (ps.client.problem, r, ps.numberPaths () - 1, 70, "sampledPath", [1,0,0,1])
writePathSamples (pathSamples, 'lampPlateforms_path.txt')

q_goal = ps.configAtParam(solutionPathId,ps.pathLength(solutionPathId))
pathToYamlFile (ps, r, "lampTrunkPlateforms_frames.yaml", "lamp_trunk", orientedpathIdBis, q_goal, 0.01)

plotFrame (r, "framy", [0,0,0], 1)

# plot contact-cones of waypoints
coneGroupName = "ConesWP_contactCones"; r.client.gui.createGroup (coneGroupName)
for i in range(0,len(waypoints)):
    q = waypoints[i]; conesWP = rbprmBuilder.getContactCones (q); coneName = "ConesWP"+str(i); contactConeName = coneName+"_contactCones_0"; plotContactCones (conesWP, ps, r, coneName, "friction_cone1"); r.client.gui.addToGroup (contactConeName, coneGroupName)


#r.client.gui.writeNodeFile(coneGroupName,'ConesWP_contactCones1_lampPlateforms.dae')
"""

