import bpy

taggedObjects = list()
def tagObjects ():
  global taggedObjects
  taggedObjects = list ()
  for obj in bpy.data.objects:
    taggedObjects.append (obj.name)

def getNonTaggedObjects ():
  global taggedObjects
  return [obj for obj in bpy.data.objects if obj.name not in taggedObjects]

def setParent (children, parent):
  for child in children:
    child.parent = parent

mat = bpy.data.materials.new("Blue")
mat.diffuse_color = [0.0, 0.0, 1.0]
mat.alpha = 0.5
mat = bpy.data.materials.new("White")
mat.diffuse_color = [1.0, 1.0, 1.0]
mat.alpha = 1.0
mat = bpy.data.materials.new("Green")
mat.diffuse_color = [0.0, 1.0, 0.1]
mat.alpha = 0.2
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Pelvis.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/Pelvis_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/Pelvis"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Sacrum.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/Sacrum_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/Sacrum"
currentObj.parent = empty
currentObj.location = [-0.0255, 0.0, 0.028333]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/L5.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/L5_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/L5"
currentObj.parent = empty
currentObj.location = [0.0075556, 0.0, 0.013222]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/L4.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/L4_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/L4"
currentObj.parent = empty
currentObj.location = [0.0056667, 0.0, 0.016056]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/L3.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/L3_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/L3"
currentObj.parent = empty
currentObj.location = [0.00094444, 0.0, 0.017]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/L2.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/L2_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/L2"
currentObj.parent = empty
currentObj.location = [-0.00094444, 0.0, 0.015111]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/L1.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/L1_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/L1"
currentObj.parent = empty
currentObj.location = [-0.0028333, 0.0, 0.015111]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Thorax.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/Thorax_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/Thorax"
currentObj.parent = empty
currentObj.location = [0.035889, 0.0, 0.14072]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Skull.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/Skull_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/Skull"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/RThigh.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/RThigh_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RThigh"
currentObj.parent = empty
currentObj.location = [0.0, -0.035889, -0.18606]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/RShank.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/RShank_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RShank"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.18813]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/RFoot.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/RFoot_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RFoot"
currentObj.parent = empty
currentObj.location = [0.065875, -0.018889, -0.025972]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_ico_sphere_add (size=0.01)
currentObj = bpy.context.object
bpy.context.object.name = "skeleton/RFootSphere_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RFootSphere"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/LThigh.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/LThigh_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LThigh"
currentObj.parent = empty
currentObj.location = [0.0, 0.035889, -0.18606]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/LShank.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/LShank_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LShank"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.18813]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/LFoot.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/LFoot_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LFoot"
currentObj.parent = empty
currentObj.location = [0.065875, 0.018889, -0.025972]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_ico_sphere_add (size=0.01)
currentObj = bpy.context.object
bpy.context.object.name = "skeleton/LFootSphere_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LFootSphere"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/RHumerus.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/RHumerus_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RHumerus"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.1581]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/RForearm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/RForearm_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RForearm"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.11352]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/RHand.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/RHand_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RHand"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.049867]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_ico_sphere_add (size=0.01)
currentObj = bpy.context.object
bpy.context.object.name = "skeleton/RHandSphere_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/RHandSphere"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/LHumerus.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/LHumerus_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LHumerus"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.1581]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/LForearm.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/LForearm_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LForearm"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.11352]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.import_mesh.stl (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/LHand.stl")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton/LHand_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LHand"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, -0.049867]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
bpy.ops.mesh.primitive_ico_sphere_add (size=0.01)
currentObj = bpy.context.object
bpy.context.object.name = "skeleton/LHandSphere_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["Blue"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton/LHandSphere"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
