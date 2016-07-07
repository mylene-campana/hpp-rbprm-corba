#/usr/bin/env python
# author: Mylene Campana (mcampana@laas.fr)
# Script which goes with hpp-rbprm-corba package.

from hpp.corbaserver.rbprm.rbprmbuilder import Builder

from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.tools.plot_analytics  import plotOctreeValues
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
import numpy as np
from viewer_library import *



packageName = 'hpp-rbprm-corba'
meshPackageName = 'hpp-rbprm-corba'
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])



q_jump= fullBody.getCurrentConfig()
q_jump  = [0.0,0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0,0, 0.0, 0.0, 0, -0.7, -1.5, -0.4, -0.5, -1.2, -0.4, 0.1, -0.7, 1.5,0.4, -0.5, 1.2, -0.4, -0.1, -1.2, -0.3, -0.2, 2.2, -0.9,0, 0.0, -1.2, 0.3, 0.2, 2.2, -0.9, 0, 0.0]


# q_jump[0:7] = q_init_test[0:7] ; rr(q_jump)


q_lfeet = q_jump[27:34]
q_rfeet = q_jump[34:41]
q_larm = q_jump[13:20]
q_rarm = q_jump[20:27]

fullBody.addRefConfigAnalysisWeight(q_lfeet,"RefPoseLFeet",[1.,1.,1.,5.,1.,1.,1.])
fullBody.addRefConfigAnalysisWeight(q_rfeet,"RefPoseRFeet",[1.,1.,1.,5.,1.,1.,1.])
fullBody.addRefConfigAnalysis(q_larm,"RefPoseLArm")
fullBody.addRefConfigAnalysis(q_rarm,"RefPoseRArm")




#~ AFTER loading obstacles
nbSamples = 50000
cType = "_6_DOF"
x = 0.03 # contact surface width
y = 0.08 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_rx'
rfoot = 'SpidermanRFootSphere'
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,-0.01],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LThigh_rx'
lfoot = 'SpidermanLFootSphere'
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,-0.01],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Legs added to fullbody")

rarmId = 'rhand'
rLeg = 'RHumerus_rx'
rfoot = 'SpidermanRHandSphere'
rarmx = x; rarmy = y
fullBody.addLimb(rarmId,rLeg,rfoot,[0,0,0],[-1,0,0], x, y, nbSamples, "manipulability", 0.01,"_3_DOF")

larmId = 'lhand'
lLeg = 'LHumerus_rx'
lfoot = 'SpidermanLHandSphere'
larmx = x; larmy = y
fullBody.addLimb(larmId,lLeg,lfoot,[0,0,0],[-1,0,0], x, y, nbSamples, "manipulability", 0.01,"_3_DOF")

print("Arms added to fullbody")

def runallLLeg(lid, dbName):

	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "RefPoseLFeet", True)
	fullBody.saveLimbDatabase(lid, dbName)

def runallRLeg(lid, dbName):

	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "RefPoseRFeet", True)
	fullBody.saveLimbDatabase(lid, dbName)

def runallLArm(lid, dbName):

	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "RefPoseLArm", True)
	fullBody.saveLimbDatabase(lid, dbName)



def runallRArm(lid, dbName):

	fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
	fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
	fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
	fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
	fullBody.runLimbSampleAnalysis(lid, "RefPoseRArm", True)
	fullBody.saveLimbDatabase(lid, dbName)



print("Run all legs : ")
runallLLeg(lLegId, './Spiderman_lleg.db')
runallRLeg(rLegId, './Spiderman_rleg.db')
print("Run all arms : ")
runallLArm(larmId, './Spiderman_larm.db')
runallRArm(rarmId, './Spiderman_rarm.db')



##plotOctreeValues(fullBody, "isotropy", lLegId)

