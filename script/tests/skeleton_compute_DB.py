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
urdfName = "skeleton"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])
q_0 = fullBody.getCurrentConfig()


flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.1, 0.2, -1.8, -1.2, 0.0, 0.0, 0.0, -0.5, -0.1, 0.2, -1.8, 1.2, 0.0, 0.0, 0.0, 0.1, 0.2, -1.1, 2.2, -1.2, 0.1, -0.1, -0.2, -1.1, 2.2, -1.2, -0.1]


"""
from hpp.gepetto import Viewer
psf = ProblemSolver( fullBody ); r = Viewer (psf)
r(q_0)
"""

q_lfeet = flexion[47:53]
q_rfeet = flexion[53:59]
q_larm = flexion[31:38]
q_rarm = flexion[39:46]

fullBody.addRefConfigAnalysisWeight(q_lfeet,"RefPoseLFeet",[1.,1.,1.,5.,1.,1.])
fullBody.addRefConfigAnalysisWeight(q_rfeet,"RefPoseRFeet",[1.,1.,1.,5.,1.,1.])
fullBody.addRefConfigAnalysis(q_larm,"RefPoseLArm")
fullBody.addRefConfigAnalysis(q_rarm,"RefPoseRArm")

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_3_DOF"
x = 0.03 # contact surface width
y = 0.08 # contact surface length
# By default, all offset are set to [0,0,0], leg normals [0,0,1] and hand normals [1,0,0]

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RHip_J1'
rfoot = 'RFootSphere'
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lLegId = 'lfoot'
lLeg = 'LHip_J1'
lfoot = 'LFootSphere'
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

print("Legs added to fullbody")

rarmId = 'rhand'
fullBody.addLimb(rarmId,'RShoulder_J1','RHandSphere',[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

larmId = 'lhand'
fullBody.addLimb(larmId,'LShoulder_J1','LHandSphere',[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

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

runallLLeg(lLegId, './skeleton_lleg.db')
runallRLeg(rLegId, './skeleton_rleg.db')
runallLArm(larmId, './skeleton_larm.db')
runallRArm(rarmId, './skeleton_rarm.db')


##plotOctreeValues(fullBody, "isotropy", lLegId)

"""
# flexion + amrs
q = q_0
q [fullBody.rankInConfiguration ['RHip_J1']] = -0.1; r(q)
q [fullBody.rankInConfiguration ['RHip_J2']] = -0.2; r(q)
q [fullBody.rankInConfiguration ['RThigh']] = -1.1; r(q)
q [fullBody.rankInConfiguration ['RShank']] = 2.2; r(q)
q [fullBody.rankInConfiguration ['RAnkle_J1']] = -1.2; r(q)
q [fullBody.rankInConfiguration ['RFoot']] = -0.1; r(q)

q [fullBody.rankInConfiguration ['LHip_J1']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['LHip_J2']] = 0.2; r(q)
q [fullBody.rankInConfiguration ['LThigh']] = -1.1; r(q)
q [fullBody.rankInConfiguration ['LShank']] = 2.2; r(q)
q [fullBody.rankInConfiguration ['LAnkle_J1']] = -1.2; r(q)
q [fullBody.rankInConfiguration ['LFoot']] = 0.1; r(q)

q [fullBody.rankInConfiguration ['LShoulder_J1']] = 0.6; r(q)
q [fullBody.rankInConfiguration ['LShoulder_J2']] = 0.1; r(q)
q [fullBody.rankInConfiguration ['LHumerus']] = 0.2; r(q)
q [fullBody.rankInConfiguration ['LElbow_J1']] = -1.4; r(q)
q [fullBody.rankInConfiguration ['LForearm']] = -1.2; r(q)

q [fullBody.rankInConfiguration ['RShoulder_J1']] = -0.6; r(q)
q [fullBody.rankInConfiguration ['RShoulder_J2']] = -0.1; r(q)
q [fullBody.rankInConfiguration ['RHumerus']] = 0.2; r(q)
q [fullBody.rankInConfiguration ['RElbow_J1']] = -1.4; r(q)
q [fullBody.rankInConfiguration ['RForearm']] = 1.2; r(q)

[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, -0.1, 0.3, -1.8, -1.2, 0, 0, 0.0, -0.2, 0.1, 0.3, -1.8, 1.2, 0.0, 0.0, 0.0, 0.1, 0.2, -1.1, 2.2, -1.2, 0.0, -0.1, -0.2, -1.1, 2.2, -1.2, -0.1]
fullBody.isConfigValid(q)

fullBody.rankInConfiguration ['RShoulder_J1']
fullBody.rankInConfiguration ['RHand']

fullBody.rankInConfiguration ['LShoulder_J1']
fullBody.rankInConfiguration ['LHand']

fullBody.rankInConfiguration ['RHip_J1']
fullBody.rankInConfiguration ['RFoot']

fullBody.rankInConfiguration ['LHip_J1']
fullBody.rankInConfiguration ['LFoot']
"""
