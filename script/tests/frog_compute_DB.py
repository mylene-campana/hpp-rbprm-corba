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
urdfName = "frog"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])



flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.2, 0.2, -0.2, 0.1, -0.1, -0.4, -0.2, 0.2, -0.5, 0.2, -0.2, 0.2, 0.1, 0.1, 0.4, -0.2, -0.2, -0.3, -0.6, -1.2, -0.1, 0.4, 1, -0.2, 0.1, -1.5, 0.3, -0.6, 1.2, 0.1, 0.4, -1, 0.2, 0.1, 1.5]

"""from hpp.gepetto import Viewer
psf = ProblemSolver( fullBody ); r = Viewer (psf)
r(flexion)"""

q_lfeet = flexion[28:37]
q_larm = flexion[10:19]
q_rfeet = flexion[37:46]
q_rarm = flexion[19:28]

legWheights = [1.,3.,4.,1.,3.,4.,1.,1.,3.]

fullBody.addRefConfigAnalysisWeight(q_lfeet,"RefPoseLFeet",legWheights)
fullBody.addRefConfigAnalysisWeight(q_rfeet,"RefPoseRFeet",legWheights)
fullBody.addRefConfigAnalysis(q_larm,"RefPoseLArm")
fullBody.addRefConfigAnalysis(q_rarm,"RefPoseRArm")

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_3_DOF"
x = 0.02 # contact surface width
y = 0.03 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_rx'
rfoot = 'FrogRFootSphere'
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LThigh_rx'
lfoot = 'FrogLFootSphere'
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Legs added to fullbody")

rarmId = 'rhand'
rArm = 'RHumerus_rx'
rHand = 'FrogRHandSphere'
rarmx = x; rarmy = y
fullBody.addLimb(rarmId,rArm,rHand,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

larmId = 'lhand'
lArm = 'LHumerus_rx'
lHand = 'FrogLHandSphere'
larmx = x; larmy = y
fullBody.addLimb(larmId,lArm,lHand,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

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



runallLLeg(lLegId, './Frog_lleg.db')
runallRLeg(rLegId, './Frog_rleg.db')
runallLArm(larmId, './Frog_larm.db')
runallRArm(rarmId, './Frog_rarm.db')



##plotOctreeValues(fullBody, "isotropy", lLegId)

"""
# flexion
legWheights = [1.,5.,10.,1.,5.,10.,1.,1.,5.]
q = q_0; q[2] = 0.5
q = flexion
q [fullBody.rankInConfiguration ['LThigh_rx']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['LThigh_ry']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['LThigh_rz']] = -1.2; rr(q)
q [fullBody.rankInConfiguration ['LShank_rx']] = -0.1; rr(q)
q [fullBody.rankInConfiguration ['LShank_ry']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['LShank_rz']] = 1; rr(q)
q [fullBody.rankInConfiguration ['LFoot_rx']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['LFoot_ry']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['LFoot_rz']] = -1.5; rr(q)

q [fullBody.rankInConfiguration ['RThigh_rx']] = 0.3; rr(q)
q [fullBody.rankInConfiguration ['RThigh_ry']] = -0.6; rr(q)
q [fullBody.rankInConfiguration ['RThigh_rz']] = 1.2; rr(q)
q [fullBody.rankInConfiguration ['RShank_rx']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['RShank_ry']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['RShank_rz']] = -1; rr(q)
q [fullBody.rankInConfiguration ['RFoot_rx']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RFoot_ry']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['RFoot_rz']] = 1.5; rr(q)


q [fullBody.rankInConfiguration ['LHumerus_rx']] = 0.5; rr(q)
q [fullBody.rankInConfiguration ['LHumerus_ry']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LHumerus_rz']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['LForearm_rx']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['LForearm_ry']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['LForearm_rz']] = -0.1; rr(q)
q [fullBody.rankInConfiguration ['LHand_rx']] = -0.4; rr(q)
q [fullBody.rankInConfiguration ['LHand_ry']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['LHand_rz']] = 0.2; rr(q)

q [fullBody.rankInConfiguration ['RHumerus_rx']] = -0.5; rr(q)
q [fullBody.rankInConfiguration ['RHumerus_ry']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RHumerus_rz']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['RForearm_rx']] = 0.2; rr(q)
q [fullBody.rankInConfiguration ['RForearm_ry']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['RForearm_rz']] = 0.1; rr(q)
q [fullBody.rankInConfiguration ['RHand_rx']] = 0.4; rr(q)
q [fullBody.rankInConfiguration ['RHand_ry']] = -0.2; rr(q)
q [fullBody.rankInConfiguration ['RHand_rz']] = -0.2; rr(q)

fullBody.isConfigValid(q)
"""
