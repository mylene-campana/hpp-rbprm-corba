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
urdfName = "kangaroo"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])
q_0 = fullBody.getCurrentConfig()


flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, -0.1, 0.0, 0.0, 0.2, 0.0, 0.0, 0.2, 0.0, 0.0, -0.4, 0.0, -0.4, 0.0, -0.1, 0.0, -0.4, 0.0, -0.4, 0.0, -0.1, 0.0, -0.3, 0.0, 0.3, 0.1, -0.6, 0.0, -0.3, 0.0, 0.3, 0.1, -0.6]
#extending = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0, 0.0, -0.1, 0.0, 0, 0.0, 0, 0.0, -0.1, 0.0, 0.3, 0.0, -0.3, 0.8, -0.8, 0.0, 0.3, 0.0, -0.3, 0.8, -0.8]

"""
from hpp.gepetto import Viewer
psf = ProblemSolver( fullBody ); r = Viewer (psf)
r(q_0)
"""


q_lfeet = flexion[31:37]
q_rfeet = flexion[37:43]
q_larm = flexion[19:25]
q_rarm = flexion[25:31]

legWheights = [1.,3.,1.,3.,1.,3.,5.]
fullBody.addRefConfigAnalysisWeight(q_lfeet,"RefPoseLFeet",legWheights)
fullBody.addRefConfigAnalysisWeight(q_rfeet,"RefPoseRFeet",legWheights)
fullBody.addRefConfigAnalysis(q_larm,"RefPoseLArm")
fullBody.addRefConfigAnalysis(q_rarm,"RefPoseRArm")

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_6_DOF"
x = 0.04 # contact surface width
y = 0.06 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_rx'
rfoot = 'KangarooRFootSphere'
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LThigh_rx'
lfoot = 'KangarooLFootSphere'
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Legs added to fullbody")

rarmId = 'rhand'
fullBody.addLimb(rarmId,'RArm_rx','KangarooRHandSphere',[0,0,0],[0,-1,0], x, y, nbSamples, "manipulability", 0.01,"_3_DOF")

larmId = 'lhand'
fullBody.addLimb(larmId,'LArm_rx','KangarooLHandSphere',[0,0,0],[0,1,0], x, y, nbSamples, "manipulability", 0.01,"_3_DOF")

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

runallLLeg(lLegId, './kangaroo_lleg_6DOF.db')
runallRLeg(rLegId, './kangaroo_rleg_6DOF.db')
runallLArm(larmId, './kangaroo_larm_3DOF.db')
runallRArm(rarmId, './kangaroo_rarm_3DOF.db')


##plotOctreeValues(fullBody, "isotropy", lLegId)

"""
# q_contact (ref config for contact sampling)
q = q_0 [::]
q [fullBody.rankInConfiguration ['RThigh_ry']] = 0.3; r(q)
q [fullBody.rankInConfiguration ['RShank_ry']] = -0.3; r(q)
q [fullBody.rankInConfiguration ['RFoot_ry']] = 0.8; r(q)
q [fullBody.rankInConfiguration ['RFootToe_ry']] = -0.8; r(q)

q [fullBody.rankInConfiguration ['RArm_ry']] = 0; r(q)
q [fullBody.rankInConfiguration ['RForearm_ry']] = 0; r(q)
q [fullBody.rankInConfiguration ['RHand_ry']] = -0.1; r(q)

q [fullBody.rankInConfiguration ['LThigh_ry']] = 0.3; r(q)
q [fullBody.rankInConfiguration ['LShank_ry']] = -0.3; r(q)
q [fullBody.rankInConfiguration ['LFoot_ry']] = 0.8; r(q)
q [fullBody.rankInConfiguration ['LFootToe_ry']] = -0.8; r(q)

q [fullBody.rankInConfiguration ['LArm_ry']] = 0; r(q)
q [fullBody.rankInConfiguration ['LForearm_ry']] = 0; r(q)
q [fullBody.rankInConfiguration ['LHand_ry']] = -0.1; r(q)

q [fullBody.rankInConfiguration ['Head_ry']] = 0.0; r(q)
q [fullBody.rankInConfiguration ['Torso_ry']] = 0.0; r(q)
q [fullBody.rankInConfiguration ['Tailbase_ry']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['Tail_ry']] = 0.2; r(q)

fullBody.isConfigValid(q)

fullBody.rankInConfiguration ['RThigh_rx']
fullBody.rankInConfiguration ['RFootToe_ry']

fullBody.rankInConfiguration ['LThigh_rx']
fullBody.rankInConfiguration ['LFootToe_ry']

fullBody.rankInConfiguration ['RArm_rx']
fullBody.rankInConfiguration ['RHand_ry']

fullBody.rankInConfiguration ['LArm_rx']
fullBody.rankInConfiguration ['LHand_ry']


"""
