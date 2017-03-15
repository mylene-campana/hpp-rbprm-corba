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
urdfName = "ant"
urdfSuffix = ""
srdfSuffix = ""


fullBody = FullBody ()

fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("base_joint_xyz", [0,0,0,0,0,0])
q_0 = fullBody.getCurrentConfig()


flexion = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.4, 0.0, 1.0, -0.5, -0.3, 0.1, -0.2, 0.0, -1.1, 0.0, 0.0, 1.1, 0.0, 0.0, 0.0, 0.0, 0.6, -1.1, 0.0, -0.6, 1.1, -0.6, -0.1, 0.0, 0.0, 0.0, 1.0, 0.4, 0.0, -1.0, 0.5, -0.3, -0.1, 0.2, 0.0, 1.1, 0.0, 0.0, -1.1, 0.0, 0.0, 0.0, 0.0, 0.6, 1.1, 0.0, -0.5, -1.1, 0.6, 0.1, 0.0, 0.0]

"""
from hpp.gepetto import Viewer
psf = ProblemSolver( fullBody ); rr = Viewer (psf)
rr(q_0)
"""

q_LFfeet = flexion[22:31]
q_LMfeet = flexion[31:40]
q_LBfeet = flexion[13:22]
q_RFfeet = flexion[49:58]
q_RMfeet = flexion[58:67]
q_RBfeet = flexion[40:49]

fullBody.addRefConfigAnalysisWeight(q_LFfeet,"RefPoseLFFeet",[1., 5., 1., 1., 5., 1., 1., 1., 1.])
fullBody.addRefConfigAnalysisWeight(q_LMfeet,"RefPoseLMFeet",[2., 5., 1, 2., 5., 2., 1., 1., 1.])
fullBody.addRefConfigAnalysisWeight(q_LBfeet,"RefPoseLBFeet",[1., 5., 2., 1., 5., 2., 1., 1., 1.])
fullBody.addRefConfigAnalysisWeight(q_RFfeet,"RefPoseRFFeet",[1., 5., 1., 1., 5., 1., 1., 1., 1.])
fullBody.addRefConfigAnalysisWeight(q_RMfeet,"RefPoseRMFeet",[2., 5., 1, 2., 5., 2., 1., 1., 1.])
fullBody.addRefConfigAnalysisWeight(q_RBfeet,"RefPoseRBFeet",[1., 5., 2., 1., 5., 1., 1., 1., 1.])

#~ AFTER loading obstacles
nbSamples = 50000
cType = "_6_DOF"
x = 0.03 # contact surface width
y = 0.03 # contact surface length

lfLegId = 'LFFoot'
lfLeg = 'LFThigh_rx'
lffoot = 'LFFootSphere'
fullBody.addLimb(lfLegId,lfLeg,lffoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lmLegId = 'LMFoot'
lmLeg = 'LMThigh_rx'
lmfoot = 'LMFootSphere'
fullBody.addLimb(lmLegId,lmLeg,lmfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

lbLegId = 'LBFoot'
lbLeg = 'LBThigh_rx'
lbfoot = 'LBFootSphere'
fullBody.addLimb(lbLegId,lbLeg,lbfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

rfLegId = 'RFFoot'
rfLeg = 'RFThigh_rx'
rffoot = 'RFFootSphere'
fullBody.addLimb(rfLegId,rfLeg,rffoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

rmLegId = 'RMFoot'
rmLeg = 'RMThigh_rx'
rmfoot = 'RMFootSphere'
fullBody.addLimb(rmLegId,rmLeg,rmfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)

rbLegId = 'RBFoot'
rbLeg = 'RBThigh_rx'
rbfoot = 'RBFootSphere'
fullBody.addLimb(rbLegId,rbLeg,rbfoot,[0,0,0],[0,0,1], x, y, nbSamples, "manipulability", 0.01,cType)
print("Limbs added to fullbody")


def runallLFLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLFFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallLMLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLMFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallLBLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseLBFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallRFLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseRFFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallRMLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseRMFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

def runallRBLeg(lid, dbName):
    fullBody.runLimbSampleAnalysis(lid, "minimumSingularValue", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulabilityTr", False)
    fullBody.runLimbSampleAnalysis(lid, "jointLimitsDistance", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropyTr", False)
    fullBody.runLimbSampleAnalysis(lid, "isotropy", False)
    fullBody.runLimbSampleAnalysis(lid, "manipulability", False)
    fullBody.runLimbSampleAnalysis(lid, "RefPoseRBFeet", True)
    fullBody.saveLimbDatabase(lid, dbName)

# 50k samples

runallLFLeg(lfLegId, './ant_LFleg_6DOF.db') # 6DOF
runallLMLeg(lmLegId, './ant_LMleg_6DOF.db') # 6DOF
runallLBLeg(lbLegId, './ant_LBleg_6DOF.db') # 6DOF
runallRFLeg(rfLegId, './ant_RFleg_6DOF.db') # 6DOF
runallRMLeg(rmLegId, './ant_RMleg_6DOF.db') # 6DOF
runallRBLeg(rbLegId, './ant_RBleg_6DOF.db') # 6DOF


# 10k samples
"""
runallLFLeg(lfLegId, './ant_LFleg_6DOF_low.db') # 6DOF
runallLMLeg(lmLegId, './ant_LMleg_6DOF_low.db') # 6DOF
runallLBLeg(lbLegId, './ant_LBleg_6DOF_low.db') # 6DOF
runallRFLeg(rfLegId, './ant_RFleg_6DOF_low.db') # 6DOF
runallRMLeg(rmLegId, './ant_RMleg_6DOF_low.db') # 6DOF
runallRBLeg(rbLegId, './ant_RBleg_6DOF_low.db') # 6DOF
"""

# 50k samples
"""
runallLFLeg(lfLegId, './ant_LFleg_3DOF.db') # 3DOF
runallLMLeg(lmLegId, './ant_LMleg_3DOF.db') # 3DOF
runallLBLeg(lbLegId, './ant_LBleg_3DOF.db') # 3DOF
runallRFLeg(rfLegId, './ant_RFleg_3DOF.db') # 3DOF
runallRMLeg(rmLegId, './ant_RMleg_3DOF.db') # 3DOF
runallRBLeg(rbLegId, './ant_RBleg_3DOF.db') # 3DOF
"""

# 10k samples
"""
runallLFLeg(lfLegId, './ant_LFleg_3DOF_low.db') # 3DOF
runallLMLeg(lmLegId, './ant_LMleg_3DOF_low.db') # 3DOF
runallLBLeg(lbLegId, './ant_LBleg_3DOF_low.db') # 3DOF
runallRFLeg(rfLegId, './ant_RFleg_3DOF_low.db') # 3DOF
runallRMLeg(rmLegId, './ant_RMleg_3DOF_low.db') # 3DOF
runallRBLeg(rbLegId, './ant_RBleg_3DOF_low.db') # 3DOF
"""

##plotOctreeValues(fullBody, "isotropy", lLegId)


"""
# flexion
q = q_0 [::]
q [fullBody.rankInConfiguration ['LFThigh_rx']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LFThigh_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LFThigh_rz']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LFShank_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LFShank_rz']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LFFoot_rx']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LFFoot_ry']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LFFoot_rz']] = 0. ; rr(q)

q [fullBody.rankInConfiguration ['LMThigh_rx']] = 0.6 ; rr(q)
q [fullBody.rankInConfiguration ['LMThigh_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LMThigh_rz']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LMShank_rx']] = -0.6 ; rr(q)
q [fullBody.rankInConfiguration ['LMShank_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['LMShank_rz']] = -0.6 ; rr(q)
q [fullBody.rankInConfiguration ['LMFoot_rx']] = -0.1 ; rr(q)
q [fullBody.rankInConfiguration ['LMFoot_ry']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LMFoot_rz']] = 0. ; rr(q)

q [fullBody.rankInConfiguration ['LBThigh_rx']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['LBThigh_ry']] = -1. ; rr(q)
q [fullBody.rankInConfiguration ['LBThigh_rz']] = -0.4 ; rr(q)
q [fullBody.rankInConfiguration ['LBShank_ry']] = 1. ; rr(q)
q [fullBody.rankInConfiguration ['LBShank_rz']] = -0.5 ; rr(q)
q [fullBody.rankInConfiguration ['LBFoot_rx']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['LBFoot_ry']] = 0.1 ; rr(q)
q [fullBody.rankInConfiguration ['LBFoot_rz']] = -0.2 ; rr(q)

q [fullBody.rankInConfiguration ['RFThigh_rx']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RFThigh_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RFThigh_rz']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RFShank_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RFShank_rz']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RFFoot_rx']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RFFoot_ry']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RFFoot_rz']] = 0. ; rr(q)

q [fullBody.rankInConfiguration ['RMThigh_rx']] = 0.6 ; rr(q)
q [fullBody.rankInConfiguration ['RMThigh_ry']] = 1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RMThigh_rz']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RMShank_rx']] = -0.5 ; rr(q)
q [fullBody.rankInConfiguration ['RMShank_ry']] = -1.1 ; rr(q)
q [fullBody.rankInConfiguration ['RMShank_rz']] = 0.6 ; rr(q)
q [fullBody.rankInConfiguration ['RMFoot_rx']] = 0.1 ; rr(q)
q [fullBody.rankInConfiguration ['RMFoot_ry']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RMFoot_rz']] = 0. ; rr(q)

q [fullBody.rankInConfiguration ['RBThigh_rx']] = 0. ; rr(q)
q [fullBody.rankInConfiguration ['RBThigh_ry']] = 1. ; rr(q)
q [fullBody.rankInConfiguration ['RBThigh_rz']] = 0.4 ; rr(q)
q [fullBody.rankInConfiguration ['RBShank_ry']] = -1. ; rr(q)
q [fullBody.rankInConfiguration ['RBShank_rz']] = 0.5 ; rr(q)
q [fullBody.rankInConfiguration ['RBFoot_rx']] = -0.3; rr(q)
q [fullBody.rankInConfiguration ['RBFoot_ry']] = -0.1 ; rr(q)
q [fullBody.rankInConfiguration ['RBFoot_rz']] = 0.2 ; rr(q)

fullBody.isConfigValid(q)

fullBody.rankInConfiguration ['LFThigh_rx']
fullBody.rankInConfiguration ['LFFoot_rz']
fullBody.rankInConfiguration ['LMThigh_rx']
fullBody.rankInConfiguration ['LMFoot_rz']
fullBody.rankInConfiguration ['LBThigh_rx']
fullBody.rankInConfiguration ['LBFoot_rz']

fullBody.rankInConfiguration ['RFThigh_rx']
fullBody.rankInConfiguration ['RFFoot_rz']
fullBody.rankInConfiguration ['RMThigh_rx']
fullBody.rankInConfiguration ['RMFoot_rz']
fullBody.rankInConfiguration ['RBThigh_rx']
fullBody.rankInConfiguration ['RBFoot_rz']


## DEBUG joint orientation
fullBody.setCurrentConfig(q_0); rr(q_0)
psf.client.robot.getCurrentTransformation("LampFootSphere")
plotJointFrame (rr, psf, q_0, "LampFootSphere", 0.2)
plotFrame(rr,"framy",[0,0,0],0.2)
psf.client.robot.getCurrentTransformation("ThighJoint")
plotJointFrame (rr, psf, q_0, "ThighJoint", 0.2)

"""
