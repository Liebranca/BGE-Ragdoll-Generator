from math import radians
from mathutils import Vector,Euler,Matrix
from string import ascii_letters as allChars

bonekeys = ({
		"upperArm":"upArm",
		"foreArm":"foArm",
		"hand":"Hand",
		"spine":["Spine1", "Spine2", "Spine3"],
		"neck":"Neck",
		"head":"Head",
		"pelvis":"Root",
		"thigh":"Thigh",
		"calf":"Calf",
		"foot":"Foot"
		})
		
spine_group = ["spine","neck","head"]
arm_group = ["upperArm", "foreArm","hand"]
leg_group = ["thigh","calf","foot"]

#list of groups ragdoll can collide with
collision_mask = [1]

#list of groups ragdoll belongs to
collision_group = [1]

gameSettings=({
	"collision_mask": [True if i in collision_mask
	 else False for i in range(16)],

	"collision_group": [True if i in collision_group
	 else False for i in range(16)],

	"friction": 0.5,
	"elasticity": 0.0,
	"collision_margin": 0.16,
	"radius": 1.0,
	"damping": 0.04
	})

#if bone on RIGHT side, [invert_x, invert_y, invert_z]
arm_invert = [0,1,1]; leg_invert = [0,1,1]

#Settings if bone on LEFT side, inverted for RIGHT.
#[ [minAngle_x, maxAngle_x], [minAngle_y, maxAngle_y], [minAngle_z, maxAngle_z] ]
limits = ({
		"upperArm": [ [-75, 75], [-25, 25], [-40, 75] ],
		"foreArm": [ [0, 0], [0, 0], [0, 75] ],
		"hand": [ [-30, 30], [0, 0], [0, 0] ],
	
		"thigh": [ [-50, 75], [-25, 0], [-10, 50] ],
		"calf": [ [-90, 0], [-25, 0], [0,0] ],
		"foot": [ [-30, 30], [0, 0], [0, 0] ],
	
		"spine": [ [-35,35], [-25,25], [-10,10] ],
		"neck": [ [-35,35], [-25,25], [-10,10] ],
		"head": [ [-35,35], [-25,25], [-10,10] ]
	})

def run():
	import bpy,bmesh
	from raggen import utils

	if "rd_phys_mesh" not in bpy.data.meshes:
		bm = bmesh.new()
		bmesh.ops.create_cube(bm)
		phys_mesh = bpy.data.meshes.new('rd_phys_mesh')
		bm.to_mesh(phys_mesh)
		obj = bpy.data.objects.new("tempPhys",phys_mesh)
		bpy.context.scene.objects.link(obj)
		mesh = obj.data
		for vert in mesh.vertices:
			vert.co[1] += 0.5
		bpy.context.scene.objects.unlink(obj)
		bm.free()
	else:
		phys_mesh = bpy.data.meshes['rd_phys_mesh']

	arma_ref = bpy.context.active_object
	arma_pose = arma_ref.pose.bones
	phys_dict = {}
	ik_phys_dict = {}
	self = arma_ref.data
	

	if not isinstance(self,bpy.types.Armature):
		print("You need to select an armature.")
		return False

	if arma_ref.name+"_physBody" in bpy.data.groups:
		bpy.data.groups.remove(bpy.data.groups[arma_ref.name+"_physBody"])
	
	if arma_ref.name+"_physBody" in bpy.data.objects:
		objs = bpy.data.objects
		objs.remove(objs[arma_ref.name+"_physBody"], do_unlink=True)

	for key, name in bonekeys.items():
		if isinstance(name, list):
			twins = name
		else:
			if name+".R" in self.bones:
				twins = [name+".R", name+".L"]
			else:
				twins = [name]

		for sName in twins:
			if ".R" in sName:
				doInvert = (arm_invert if key in arm_group
				 else leg_invert if key in leg_group
				 else [0,0,0])

			else: doInvert = [0,0,0]

			bone = self.bones[sName]
			doConst = bone.parent != None
			if key in limits: constLimits = limits[key]
			else: constLimits = None

			phys_dict[bone.name] = utils.createPhysBodyPart(arma_ref,
			bone, phys_mesh, doConst, gameSettings, constLimits, doInvert)

	bpy.context.scene.update()

	if "ragdoll_physBody_controller" not in arma_ref.game.controllers:
		bpy.ops.logic.controller_add(type="PYTHON",
			name="ragdoll_physBody_controller", object=arma_ref.name)

	arma_ragCont = arma_ref.game.controllers["ragdoll_physBody_controller"]
	arma_ragCont.mode = "MODULE"
	arma_ragCont.show_expanded = False
	arma_ragCont.module = "raggen.game.ragLogic"
	if "ragdoll_runArmature" not in arma_ref.game.actuators:
		bpy.ops.logic.actuator_add(type="ARMATURE",name="ragdoll_runArmature",
			object=arma_ref.name)

	arma_runArma = arma_ref.game.actuators["ragdoll_runArmature"]
	arma_runArma.mode = "RUN"
	arma_runArma.show_expanded = False
	if "ragdoll_always" not in arma_ref.game.sensors:
		bpy.ops.logic.sensor_add(type="DELAY",
			name="ragdoll_always",object=arma_ref.name)
	else:
		for cont in arma_ref.game.sensors["ragdoll_always"].controllers:
			arma_ref.game.sensors["ragdoll_always"].unlink(cont)
		arma_ref.game.sensors["ragdoll_always"].link(arma_ragCont)

	arma_always = arma_ref.game.sensors["ragdoll_always"]
	arma_always.use_repeat = True
	arma_always.show_expanded = False
	arma_ragCont.link(arma_always,arma_runArma)

	if "ragdoll_toggle" not in arma_ref.game.properties:
		bpy.ops.object.game_property_new(type='BOOL', name='ragdoll_toggle')
	newProp = arma_ref.game.properties["ragdoll_toggle"]
	newProp.value = False
	if "ragdoll_clear" not in arma_ref.game.properties:
		bpy.ops.object.game_property_new(type='BOOL', name='ragdoll_clear')
	newProp = arma_ref.game.properties["ragdoll_clear"]
	newProp.value = True
	if "ragdoll_physGroup" not in arma_ref.game.properties:
		bpy.ops.object.game_property_new(type='STRING', name='ragdoll_physGroup')
	newProp = arma_ref.game.properties["ragdoll_physGroup"]
	newProp.value = arma_ref.name+"_physBody"

	for key in phys_dict:
		phys = phys_dict[key]
		bone = self.bones[key]
		bone_pose = arma_pose[key]

		matrixcopy = phys.matrix_world.copy()
		phys.parent = None
		phys.matrix_world = matrixcopy
		bpy.context.scene.objects.active = phys

		if len(phys.constraints) != 0:
			const = phys.constraints[0]
			subconst = phys.constraints[-1]

			try:
				subconst.target = const.target = phys_dict[bone.parent.name]
			except:
				print("PHYS(%s): target not found"%(const.name))

			checkName = const.name.lower()
			
			in_spine_group = utils.getIsBoneGroup(checkName,spine_group)
			in_arm_group = utils.getIsBoneGroup(checkName,arm_group)
			in_leg_group = utils.getIsBoneGroup(checkName,leg_group)

			if in_leg_group:
				phys.game.mass = .65				
				phys.scale = (phys.scale[0]/2,phys.scale[1],phys.scale[2]/2)

			elif in_spine_group:
				phys.game.mass = .8
				phys.scale = (phys.scale[0]*2,phys.scale[1],phys.scale[2]*2)

			elif in_arm_group:
				phys.game.mass = .25
				phys.scale = (phys.scale[0]*1.1,phys.scale[1]/1.2,phys.scale[2]/1.1)

		if "root" in phys.name.lower():
			phys.game.mass = 1.0
			phys.scale = (phys.scale[0]*2,phys.scale[1],phys.scale[2]*2)
			if phys.name+"_limit_distance" not in bone_pose.constraints:
				const = bone_pose.constraints.new(type='LIMIT_DISTANCE')
				const.name = phys.name+"_limit_distance"

			const = bone_pose.constraints[phys.name+"_limit_distance"]
			const.target = phys
			const.active = False

			bpy.ops.object.game_property_new(type='BOOL', name='physRoot')

		
		if (arma_ref.name+"_physBody") not in bpy.data.groups:
			bpy.data.groups.new(arma_ref.name+"_physBody")

		bpy.data.groups[arma_ref.name+"_physBody"].objects.link(phys)

		bpy.ops.object.game_property_new(type='STRING', name='bone')
		bpy.ops.object.game_property_new(type='STRING', name='constraint')
		phys_boneProp = phys.game.properties["bone"]
		phys_boneProp.value = key
		phys_constProp = phys.game.properties["constraint"]
		phys_constProp.value = phys.name

		if arma_pose[key].is_in_ik_chain:
			for constraint in arma_pose[key].constraints:
				if constraint.type == "IK":

					newPhys = utils.createPhysBodyPart(arma_ref,
						self.bones[constraint.subtarget],phys_mesh,True)

					ik_rigid = newPhys.constraints[constraint.subtarget]
					ik_rigid.target = phys
					newPhys.game.mass = phys.game.mass/10
					ik_rigid.pivot_type = "HINGE"
					ik_rigid.use_angular_limit_x = True
					ik_rigid.limit_angle_min_x = radians(0)
					ik_rigid.limit_angle_max_x = radians(0)
					ik_phys_dict[constraint.subtarget] = newPhys
					ik_pose = arma_pose[constraint.subtarget]

					bpy.ops.object.game_property_new(type='BOOL', name='_IK')
					constraint.name = key+"_IK"

					if phys.name+"_IK_distance" not in ik_pose.constraints:
						const = ik_pose.constraints.new(type='LIMIT_DISTANCE')
						const.name = phys.name+"_IK_distance"
					const = ik_pose.constraints[phys.name+"_IK_distance"]
					const.target = newPhys
					const.show_expanded,const.mute,const.influence = False,True,0.0

					if phys.name+"_IK_rotation" not in ik_pose.constraints:
						const = ik_pose.constraints.new(type='COPY_ROTATION')
						const.name = phys.name+"_IK_rotation"
					const = ik_pose.constraints[phys.name+"_IK_rotation"]
					const.target = newPhys
					const.show_expanded,const.mute,const.influence = False,True,0.0

					bpy.ops.object.game_property_new(type='STRING', name='IK_bone')
					phys_boneProp = phys.game.properties["IK_bone"]
					phys_boneProp.value = constraint.subtarget

		phys.layers[19] = True
		for i in range(len(phys.layers)):
			phys.layers[i] = i == 19

		if phys.name not in bone_pose.constraints:
			const = bone_pose.constraints.new(type='COPY_ROTATION')
			const.name = phys.name
		
		const = bone_pose.constraints[phys.name]
		const.target = phys
		const.show_expanded,const.mute,const.influence = False,True,0.0

	for key in ik_phys_dict:
		phys = ik_phys_dict[key]
		matrixcopy = phys.matrix_world.copy()
		phys.parent = None
		phys.matrix_world = matrixcopy
		phys.layers[19] = True
		for i in range(len(phys.layers)):
			phys.layers[i] = i == 19

		bpy.data.groups[arma_ref.name+"_physBody"].objects.link(phys)
		bpy.context.scene.objects.active = phys
		bpy.ops.object.game_property_new(type='BOOL', name='noBone')

	bpy.context.scene.objects.active = arma_ref
	bpy.context.scene.update()
	if arma_ref.name+"_physBody" in bpy.data.groups:
		group = bpy.data.groups[arma_ref.name+"_physBody"]    
		instance = bpy.data.objects.new(group.name, None)
		instance.dupli_type = 'GROUP'
		instance.dupli_group = group
		bpy.context.scene.objects.link(instance)
		instance.layers[18] = True
		for i in range(len(instance.layers)):
			instance.layers[i] = i == 18
