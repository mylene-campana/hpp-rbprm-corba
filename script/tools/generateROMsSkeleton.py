#/usr/bin/env python

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer


import quaternion as quat

packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
rootJointType = "freeflyer"
##
#  Information to retrieve urdf and srdf files.
urdfName = "skeleton"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 10000

ps = ProblemSolver( fullBody )

r = Viewer (ps)

x = 0.01 # contact surface width
y = 0.01 # contact surface length

#~ AFTER loading obstacles
rLegId = 'rfoot'
rLeg = 'RHip_J1'
rfoot = 'Rfoot'#'RFootSphere'
rLegOffset = [0,0,0]
rLegNormal = [1,0,0] # should be x like this, but then ROM is weird... so I kept y ...
rLegx = x; rLegy = y
fullBody.addLimb(rLegId,rLeg,rfoot,rLegOffset,rLegNormal, rLegx, rLegy, nbSamples, "EFORT", 0.01)

lLegId = 'lfoot'
lLeg = 'LHip_J1'
lfoot = 'Lfoot'#'LFootSphere'
lLegOffset = [0,0,0]
lLegNormal = [1,0,0]
lLegx = x; lLegy = y
fullBody.addLimb(lLegId,lLeg,lfoot,lLegOffset,lLegNormal, lLegx, lLegy, nbSamples, "EFORT", 0.01)

rarmId = 'rhand'
rarm = 'RShoulder_J1'
rHand = 'RHand'#'RHandSphere'
rArmOffset = [0,0,0]
rArmNormal = [1,0,0]
rArmx = x; rArmy = y
fullBody.addLimb(rarmId,rarm,rHand,rArmOffset,rArmNormal, rArmx, rArmy, nbSamples, "EFORT", 0.01)

larmId = 'lhand'
larm = 'LShoulder_J1'
lHand = 'LHand'#'LHandSphere'
lArmOffset = [0,0,0]
lArmNormal = [1,0,0]
lArmx = x; lArmy = y
fullBody.addLimb(larmId,larm,lHand,lArmOffset,lArmNormal, lArmx, lArmy, nbSamples, "EFORT", 0.01)


#make sure this is 0
q_0 = fullBody.getCurrentConfig ()

#fullBody.createOctreeBoxes(r.client.gui, r.windowId, rarmId, q_0, [1,0.3,1,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, rLegId, q_0, [1,1,0.3,0.4])
fullBody.createOctreeBoxes(r.client.gui, r.windowId, lLegId, q_0, [1,1,0.3,0.4])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, larmId, q_0, [1,0.3,1,0.4])

fullBody.draw (q_0, r) # display robot with octrees


def printEffPosition(limbId, nbSamples):
    limit = nbSamples-1;
    f1=open('./data/roms/skeleton/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()


printEffPosition(rarmId, nbSamples)
printEffPosition(larmId, nbSamples)

printEffPosition(rLegId, nbSamples)
printEffPosition(lLegId, nbSamples)



#to generate .obj from .erom with Matlab scripts
#effectorRomToObj('../../data/roms/skeleton/rfoot.erom', '../../data/roms/skeleton/rfoot.obj')
#effectorRomToObj('../../data/roms/skeleton/lfoot.erom', '../../data/roms/skeleton/lfoot.obj')
#effectorRomToObj('../../data/roms/skeleton/lhand.erom', '../../data/roms/skeleton/lhand.obj')
#effectorRomToObj('../../data/roms/skeleton/rhand.erom', '../../data/roms/skeleton/rhand.obj')

# Then Decimate (object/Modifiers) the .obj in Blender and save it as .stl
# Decimate decrease the too high number of facets.

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


