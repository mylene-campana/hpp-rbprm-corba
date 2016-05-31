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

mat = bpy.data.materials.new("White")
mat.diffuse_color = [1.0, 1.0, 1.0]
mat.alpha = 1.0
mat = bpy.data.materials.new("Green")
mat.diffuse_color = [0.0, 1.0, 0.1]
mat.alpha = 0.2
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Pelvis_view.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton_trunk_flexible/Pelvis_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/Pelvis"
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
bpy.context.object.name = "skeleton_trunk_flexible/Sacrum_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/Sacrum"
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
bpy.context.object.name = "skeleton_trunk_flexible/L5_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/L5"
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
bpy.context.object.name = "skeleton_trunk_flexible/L4_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/L4"
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
bpy.context.object.name = "skeleton_trunk_flexible/L3_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/L3"
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
bpy.context.object.name = "skeleton_trunk_flexible/L2_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/L2"
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
bpy.context.object.name = "skeleton_trunk_flexible/L1_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/L1"
currentObj.parent = empty
currentObj.location = [-0.0028333, 0.0, 0.015111]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Thorax_view.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton_trunk_flexible/Thorax_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/Thorax"
currentObj.parent = empty
currentObj.location = [0.035889, 0.0, 0.14072]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
tagObjects()
bpy.ops.wm.collada_import (filepath="/local/mcampana/devel/hpp/install/share/hpp-rbprm-corba/meshes/skeleton/Skull_view.dae")
imported_objects = getNonTaggedObjects ()
print(imported_objects)
bpy.ops.object.empty_add ()
currentObj = bpy.context.object
setParent (imported_objects, currentObj)
bpy.context.object.name = "skeleton_trunk_flexible/Skull_visual0"
#bpy.context.object.data.materials.append(bpy.data.materials["White"])
bpy.ops.object.empty_add ()
empty = bpy.context.object
empty.name = "skeleton_trunk_flexible/Skull"
currentObj.parent = empty
currentObj.location = [0.0, 0.0, 0.0]
currentObj.rotation_euler = [0.0, 0.0, 0.0]
