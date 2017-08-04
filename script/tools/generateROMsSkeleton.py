#/usr/bin/env python

from hpp.corbaserver.rbprm.rbprmbuilder import Builder
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.gepetto import Viewer
from hpp.corbaserver.rbprm.problem_solver import ProblemSolver

import quaternion as quat

packageName = "hpp-rbprm-corba"
meshPackageName = "hpp-rbprm-corba"
rootJointType = "freeflyer"
urdfName = "skeleton_reducedDOFlimits"
urdfSuffix = ""
srdfSuffix = ""

fullBody = FullBody ()
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)


nbSamples = 10000
ps = ProblemSolver( fullBody ); r = Viewer (ps)

rLegId = 'rfoot'
lLegId = 'lfoot'
rarmId = 'rhand'
larmId = 'lhand'

fullBody.addLimb(rLegId,'RHip_J1','RFootSphere',[0,0,0],[1,0,0], 0.01, 0.01, nbSamples, "EFORT", 0.01)
#fullBody.addLimb(lLegId,'LHip_J1','LFootSphere',[0,0,0],[1,0,0], 0.01, 0.01, nbSamples, "EFORT", 0.01)
#fullBody.addLimb(rarmId,'RShoulder_J1','RHandSphere',[0,0,0],[1,0,0], 0.01, 0.01, nbSamples, "EFORT", 0.01)
#fullBody.addLimb(larmId,'LShoulder_J1','LHandSphere',[0,0,0],[1,0,0], 0.01, 0.01, nbSamples, "EFORT", 0.01)


#make sure this is 0
q_0 = fullBody.getCurrentConfig ()

#fullBody.createOctreeBoxes(r.client.gui, r.windowId, rarmId, q_0, [1,0.3,1,1])
fullBody.createOctreeBoxes(r.client.gui, r.windowId, rLegId, q_0, [1,1,0.3,1])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, lLegId, q_0, [1,1,0.3,1])
#fullBody.createOctreeBoxes(r.client.gui, r.windowId, larmId, q_0, [1,0.3,1,1])

fullBody.draw (q_0, r) # display robot with octrees

"""
def printEffPosition(limbId, nbSamples):
    limit = nbSamples-1;
    f1=open('./../../data/roms/skeleton/'+limbId+'.erom', 'w+')
    for i in range(0,limit):
        q = fullBody.getSamplePosition(limbId,i)
        f1.write(str(q[0]) + "," + str(q[1]) + "," + str(q[2]) + "\n")
    f1.close()


printEffPosition(rarmId, nbSamples)
printEffPosition(larmId, nbSamples)

printEffPosition(rLegId, nbSamples)
printEffPosition(lLegId, nbSamples)
"""


#to generate .obj from .erom with Matlab scripts
#effectorRomToObj('../../data/roms/skeleton/rfoot.erom', '../../data/roms/skeleton/rfoot.obj')
#effectorRomToObj('../../data/roms/skeleton/lfoot.erom', '../../data/roms/skeleton/lfoot.obj')
#effectorRomToObj('../../data/roms/skeleton/lhand.erom', '../../data/roms/skeleton/lhand.obj')
#effectorRomToObj('../../data/roms/skeleton/rhand.erom', '../../data/roms/skeleton/rhand.obj')

# Then Decimate (object/Modifiers) the .obj in Blender and save it as .stl
# Decimate decrease the too high number of facets.


"""
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

q_0 = fullBody.getCurrentConfig ()
q = q_0[::]
q [fullBody.rankInConfiguration ['RAnkle_J1']] = 0.5;r(q)
"""
