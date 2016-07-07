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
r.loadObstacleModel ('hpp-rbprm-corba', "rock", "rock")
addLight (r, [-3,0,8,1,0,0,0], "li");
addLight (r, [-1,15,0,1,0,0,0], "li2");

# Configs : [x, y, z, q1, q2, q3, q4, dir.x, dir.y, dir.z, theta]
q11 = rbprmBuilder.getCurrentConfig ()
q11[(len(q11)-4):]=[0,0,1,0] # set normal for init / goal config
q11[0:7] = [-0.15, 0,0.55, 1, 0, 0, 0]; r(q11)

rbprmBuilder.isConfigValid(q11)
#r.startCapture("antROM","png") ; r.stopCapture()




q_away = q11[::]
q_away[0:7] = [0,0,0,0,0,0,0]
r(q_away)



########################################### 


from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer
import numpy as np
from viewer_library import *


packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "ant"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [-6, 0, -2, 2, -0.3, 2.4])
fullBody.client.basic.robot.setDimensionExtraConfigSpace(ecsSize)
fullBody.client.basic.robot.setExtraConfigSpaceBounds([0,0,0,0,0,0,-3.14,3.14])

psf = ProblemSolver(fullBody)
rr = Viewer(psf)

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_6_DOF"
x = 0.03 # contact surface width
y = 0.03 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,-1]
heuristic = "EFORT_Normal"
lfLegId = 'lffoot'
lfLeg = 'LFThigh_rx'
lffoot = 'LFFootSphere'
fullBody.addLimb(lfLegId,lfLeg,lffoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

lmLegId = 'lmfoot'
lmLeg = 'LMThigh_rx'
lmfoot = 'LMFootSphere'
fullBody.addLimb(lmLegId,lmLeg,lmfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

lbLegId = 'lbfoot'
lbLeg = 'LBThigh_rx'
lbfoot = 'LBFootSphere'
fullBody.addLimb(lbLegId,lbLeg,lbfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

rfLegId = 'rffoot'
rfLeg = 'RFThigh_rx'
rffoot = 'RFFootSphere'
fullBody.addLimb(rfLegId,rfLeg,rffoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)


rmLegId = 'rmfoot'
rmLeg = 'RMThigh_rx'
rmfoot = 'RMFootSphere'
fullBody.addLimb(rmLegId,rmLeg,rmfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)

rbLegId = 'rbfoot'
rbLeg = 'RBThigh_rx'
rbfoot = 'RBFootSphere'
fullBody.addLimb(rbLegId,rbLeg,rbfoot,[0,0,0],[0,0,1], x, y, nbSamples, heuristic, 0.01,cType)
print("Limbs added to fullbody")

confsize = len(q11)-ecsSize
q_0 = fullBody.getCurrentConfig(); rr(q_0)
q_init = fullBody.getCurrentConfig(); 
q_init[0:confsize] = q11[0:confsize]


q_init_test = fullBody.generateContacts(q_init, [0,0,-1], True); rr (q_init_test)


#rr.startCapture("antFULL","png") ; rr.stopCapture()

