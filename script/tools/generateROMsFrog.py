#/usr/bin/env python

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver
from viewer_library import *
import quaternion as quat

packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "frog"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)


ps = ProblemSolver( fullBody )
r = Viewer (ps)
q_0 = fullBody.getCurrentConfig (); r(q_0)

# Verify that normals will be correct
#cl = fullBody.client.basic
#plotJointFrame (r, cl, q_0, "FrogRFootSphere", 0.15); plotJointFrame (r, cl, q_0, "FrogLFootSphere", 0.15)
#plotJointFrame (r, cl, q_0, "FrogRHandSphere", 0.15); plotJointFrame (r, cl, q_0, "FrogLHandSphere", 0.15)


nbSamples = 10000
x = 0.03 # contact surface width
y = 0.04 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,1]

rLegId = 'rfoot'
rLeg = 'TorsoRThigh_J1'
rfoot = 'FrogRFootSphere'
rLegNormal = [0,0,1]
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,[0,0,0],rLegNormal, x, y, nbSamples, "EFORT", 0.01)

lLegId = 'lfoot'
lLeg = 'TorsoLThigh_J1'
lfoot = 'FrogLFootSphere'
lLegNormal = [0,0,1]
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,[0,0,0],lLegNormal, x, y, nbSamples, "EFORT", 0.01)

rarmId = 'rhand'
rarm = 'HeadRHumerus_J1'
rHand = 'FrogRHandSphere'
rArmNormal = [1,0,0] # !! x, not z
rArmx = x; rArmy = y
fullBody.addLimb(rarmId,rarm,rHand,[0,0,0],rArmNormal, x, y, nbSamples, "EFORT", 0.01)

larmId = 'lhand'
larm = 'HeadLHumerus_J1'
lHand = 'FrogLHandSphere'
lArmNormal = [1,0,0] # !! x, not z
lArmx = x; lArmy = y
fullBody.addLimb(larmId,larm,lHand,[0,0,0],lArmNormal, x, y, nbSamples, "EFORT", 0.01)



fullBody.createOctreeBoxes(r.client.gui, r.windowId, rarmId, q_0, [1,0.3,1,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, rLegId, q_0, [1,1,0.3,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, lLegId, q_0, [1,1,0.3,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, larmId, q_0, [1,0.3,1,0.4])

fullBody.draw (q_0, r) # display robot with octrees


def printEffPosition(limbId, nbSamples):
    limit = nbSamples-1;
    f1=open('../../data/roms/frog/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        #if(fullBody.isConfigValid(config)[0]): # TODO config
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()


printEffPosition(rarmId, nbSamples)
printEffPosition(larmId, nbSamples)
printEffPosition(rLegId, nbSamples)
printEffPosition(lLegId, nbSamples)


#to generate .obj from .erom with Matlab scripts 
"""
effectorRomToObj('../../data/roms/frog/rfoot.erom', '../../data/roms/frog/rfoot.obj')
effectorRomToObj('../../data/roms/frog/lfoot.erom', '../../data/roms/frog/lfoot.obj')
effectorRomToObj('../../data/roms/frog/lhand.erom', '../../data/roms/frog/lhand.obj')
effectorRomToObj('../../data/roms/frog/rhand.erom', '../../data/roms/frog/rhand.obj')
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

r.client.gui.setVisibility('frog/torso_lhand_rom',"OFF")

q = q_0 [::]
q [fullBody.rankInConfiguration ['HumerusLForearm_J1']] = -1.5;r(q)

