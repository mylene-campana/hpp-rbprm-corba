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
urdfName = "ant"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)

from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

nbSamples = 10000

ps = ProblemSolver( fullBody )

r = Viewer (ps)

x = 0.006 # contact surface width
y = 0.006 # contact surface length
# By default, all offset are set to [0,0,0] and all normals to [0,0,1]

lfLegId = 'lffoot'
lfLeg = 'LFThigh_rx'
lffoot = 'LFFootSphere'
fullBody.addLimb(lfLegId,lfLeg,lffoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

lmLegId = 'lmfoot'
lmLeg = 'LMThigh_rx'
lmfoot = 'LMFootSphere'
fullBody.addLimb(lmLegId,lmLeg,lmfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

lbLegId = 'lbfoot'
lbLeg = 'LBThigh_rx'
lbfoot = 'LBFootSphere'
fullBody.addLimb(lbLegId,lbLeg,lbfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

rfLegId = 'rffoot'
rfLeg = 'RFThigh_rx'
rffoot = 'RFFootSphere'
fullBody.addLimb(rfLegId,rfLeg,rffoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)


rmLegId = 'rmfoot'
rmLeg = 'RMThigh_rx'
rmfoot = 'RMFootSphere'
fullBody.addLimb(rmLegId,rmLeg,rmfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)

rbLegId = 'rbfoot'
rbLeg = 'RBThigh_rx'
rbfoot = 'RBFootSphere'
fullBody.addLimb(rbLegId,rbLeg,rbfoot,[0,0,0],[0,0,1], x, y, nbSamples, "EFORT", 0.01)



#make sure this is 0
q_0 = fullBody.getCurrentConfig ()

fullBody.createOctreeBoxes(r.client.gui, r.windowId, lfLegId, q_0, [1,1,0.3,0.4])
fullBody.createOctreeBoxes(r.client.gui, r.windowId, rbLegId, q_0, [0.3,1,1,0.4])

fullBody.draw (q_0, r) # display robot with octrees


def printEffPosition(limbId, nbSamples):
    limit = nbSamples-1;
    f1=open('./data/roms/ant/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()

printEffPosition(lfLegId, nbSamples)
printEffPosition(lmLegId, nbSamples)
printEffPosition(lbLegId, nbSamples)
printEffPosition(rfLegId, nbSamples)
printEffPosition(rmLegId, nbSamples)
printEffPosition(rbLegId, nbSamples)




#to generate .obj from .erom with Matlab scripts
#effectorRomToObj('../../data/roms/ant/lffoot.erom', '../../data/roms/ant/lffoot.obj')

# Then Decimate (object/Modifiers) the .obj in Blender and save it as .stl
# Decimate decrease the too high number of facets.

# Display global frame in viewer:
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


