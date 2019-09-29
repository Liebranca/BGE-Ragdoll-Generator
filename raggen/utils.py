import bpy
from math import radians

def createPhysBodyPart(arma_ref, bone, phys_mesh,
	doConst, _gameSettings, limits, doInvert):
	if arma_ref.name+"_phys_"+bone.name in bpy.data.objects:
		objs = bpy.data.objects
		objs.remove(objs[arma_ref.name+"_phys_"+bone.name], do_unlink=True)

	newPhys = bpy.data.objects.new(arma_ref.name+"_phys_"+bone.name,phys_mesh)
	bpy.context.scene.objects.link(newPhys)
	bpy.context.scene.objects.active = newPhys
	
	newPhys.data = newPhys.data.copy()
	if newPhys.name in bpy.data.meshes:
		bpy.data.meshes.remove(bpy.data.meshes[newPhys.name])

	newPhys.data.name = newPhys.name

	newPhys.location[1] = -bone.length
	newPhys.scale = (bone.length/2,bone.length/1.25,bone.length/2)

	newPhys.parent = arma_ref
	newPhys.parent_bone = bone.name
	newPhys.parent_type = "BONE"

	gameSet = newPhys.game
	(gameSet.physics_type,gameSet.collision_bounds_type,
	gameSet.use_collision_bounds) = ("RIGID_BODY","CONVEX_HULL",True)
	
	for key, value in _gameSettings.items():
		setattr(gameSet, key, value)

	if doConst:
		const = newPhys.constraints.new(type='RIGID_BODY_JOINT')
		const.name = bone.name
		const.show_pivot = const.use_linked_collision = True
		const.pivot_type = "BALL"

		subconst = newPhys.constraints.new(type='RIGID_BODY_JOINT')
		for i, axes in enumerate(["x","y","z"]):
			if not doInvert[i]:
				setattr(subconst, "limit_angle_min_%s"%axes, radians(limits[i][0]))
				setattr(subconst, "limit_angle_max_%s"%axes, radians(limits[i][1]))
			else:
				setattr(subconst, "limit_angle_min_%s"%axes, -radians(limits[i][1]))
				setattr(subconst, "limit_angle_max_%s"%axes, -radians(limits[i][0]))

		subconst.use_angular_limit_x = True
		subconst.use_angular_limit_y = True
		subconst.use_angular_limit_z = True
		subconst.pivot_type = "GENERIC_6_DOF"

	newPhys.hide_render = True
	newPhys.draw_type = "WIRE"

	return newPhys
	