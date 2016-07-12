#/usr/bin/env python

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
from viewer_library import *

import quaternion as quat

packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "spiderman"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)


ps = ProblemSolver( fullBody )
r = Viewer (ps)
q_0 = fullBody.getCurrentConfig (); r(q_0)
plotFrame(r, "framy",[0,0,0],0.4)

# test armature pose in Blender :
q_test = [0, 0, 0, 1, 0,0, 0, 0, 0, 0.2, 0.0, 0.0, 0.0, 0.4, 0.5, 0.7, 0, -0.6, 0.0, 0.0, 0.4, 0.5, 0.7, 0, -0.6, 0.0, 0.0, -0.2, 0.3, -1.9, 1.9,-0.6, 0, -0.2, 0.3, -1.9, 1.9, -0.6, 0]; r(q_test)

# Verify that normals will be correct
cl = fullBody.client.basic
plotJointFrame (r, cl, q_0, "RHumerus_rx", 0.4)
plotJointFrame (r, cl, q_0, "RForearm_rx", 0.4)
plotJointFrame (r, cl, q_0, "RHand_rx", 0.4)
plotJointFrame (r, cl, q_0, "LHumerus_rx", 0.4)
plotJointFrame (r, cl, q_0, "LForearm_rx", 0.4)
plotJointFrame (r, cl, q_0, "LHand_rx", 0.4)
plotJointFrame (r, cl, q_0, "RThigh_rx", 0.4)
plotJointFrame (r, cl, q_0, "RShank_rx", 0.4)
plotJointFrame (r, cl, q_0, "RFoot_rx", 0.4)
plotJointFrame (r, cl, q_0, "LThigh_rx", 0.4)
plotJointFrame (r, cl, q_0, "LShank_rx", 0.4)
plotJointFrame (r, cl, q_0, "LFoot_rx", 0.4)
plotJointFrame (r, cl, q_0, "RFootToe_rx", 0.4)
plotJointFrame (r, cl, q_0, "LFootToe_rx", 0.4)
#plotJointFrame (r, cl, q_0, "SpidermanRFootSphere", 0.15); plotJointFrame (r, cl, q_0, "SpidermanLFootSphere", 0.15)
#plotJointFrame (r, cl, q_0, "SpidermanRHandSphere", 0.15); plotJointFrame (r, cl, q_0, "SpidermanLHandSphere", 0.15)
q [fullBody.rankInConfiguration ['RFootToe_rx']] = -0.2; r(q)

nbSamples = 10000
x = 0.03 # contact surface width
y = 0.08 # contact surface length

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RThigh_ry'
rfoot = 'SpidermanRFootSphere'
rLegOffset = [0,0,0]
rLegNormal = [0,0,1]
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.01)

lLegId = 'lfoot'
lLeg = 'LThigh_ry'
lfoot = 'SpidermanLFootSphere'
lLegOffset = [0,0,0]
lLegNormal = [0,0,1]
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,lLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.01)

rarmId = 'rhand'
rarm = 'RHumerus_ry'
rHand = 'SpidermanRHandSphere'
rArmOffset = [0,0,0]
rArmNormal = [1,0,0] # !! x, not z
rArmx = x; rArmy = y
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01)

larmId = 'lhand'
larm = 'RHumerus_ry'
lHand = 'SpidermanLHandSphere'
lArmOffset = [0,0,0]
lArmNormal = [1,0,0] # !! x, not z
lArmx = x; lArmy = y
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01)


#make sure this is 0
q_0 = fullBody.getCurrentConfig ()

fullBody.createOctreeBoxes(r.client.gui, r.windowId, rarmId, q_0, [1,0.3,1,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, rLegId, q_0, [1,1,0.3,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, lLegId, q_0, [1,1,0.3,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, larmId, q_0, [1,0.3,1,0.4])

fullBody.draw (q_0, r) # display robot with octrees


def printEffPosition(limbId, nbSamples):
    limit = nbSamples-1;
    f1=open('../../data/roms/spiderman/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()


printEffPosition(rarmId, nbSamples)
printEffPosition(larmId, nbSamples)

printEffPosition(rLegId, nbSamples)
printEffPosition(lLegId, nbSamples)



#to generate .obj from .erom with Matlab scripts
"""
effectorRomToObj('../../data/roms/spiderman/rfoot.erom', '../../data/roms/spiderman/rfoot.obj')
effectorRomToObj('../../data/roms/spiderman/lfoot.erom', '../../data/roms/spiderman/lfoot.obj')
effectorRomToObj('../../data/roms/spiderman/lhand.erom', '../../data/roms/spiderman/lhand.obj')
effectorRomToObj('../../data/roms/spiderman/rhand.erom', '../../data/roms/spiderman/rhand.obj')
"""

# Then Decimate (object/Modifiers) the .obj in Blender and save it as .stl
# Decimate decrease the too high number of facets.
# Rotation corrections in Blender (in degrees): -90x for feet, 90y+90x for hands


plotFrame (r, 'frame_group', [0,0,0], 0.6)

frameGroupName = "frame"
framePosition = [0,0,0]
ampl = 0.6
r.client.gui.createGroup (frameGroupName)
x = framePosition [0]; y = framePosition [1]; z = framePosition [2];
r.client.gui.addLine('frame1',[x,y,z], [x+ampl,y,z],[1,0,0,1])
r.client.gui.addToGroup ('frame1', frameGroupName)
r.client.gui.addLine('frame2',[x,y,z], [x,y+ampl,z],[0,1,0,1])
r.client.gui.addToGroup ("frame2", frameGroupName)
r.client.gui.addLine('frame3',[x,y,z], [x,y,z+ampl],[0,0,1,1])
r.client.gui.addToGroup ('frame3', frameGroupName)
r.client.gui.addSceneToWindow(frameGroupName,r.windowId)

r.client.gui.setVisibility('spiderman/thorax_lhand_rom',"OFF")
q = q_0[::]
q [fullBody.rankInConfiguration ['RShank']] = 0.5;r(q)

