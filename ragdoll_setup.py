#----------------------------------------------------------------------
"""
    Ragdoll Setup, by Liebranca
    
        To use: Select an armature, put it in rest pose, run this script.
        
        ~Cheers
        
"""
#----------------------------------------------------------------------
import bpy,bmesh,ragdoll_utils
from math import radians
from mathutils import Vector,Euler,Matrix
from string import ascii_letters as allChars
#----------------------------------------------------------------------
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
#----------------------------------------------------------------------
arma_ref = bpy.context.active_object
arma_pose = arma_ref.pose.bones
phys_dict = {}
ik_phys_dict = {}
self = arma_ref.data
if arma_ref.name+"_physBody" in bpy.data.groups:
    bpy.data.groups.remove(bpy.data.groups[arma_ref.name+"_physBody"])
#----------------------------------------------------------------------
if isinstance(self,bpy.types.Armature):
#----------------------------------------------------------------------
    for bone in self.bones:
        if bone.use_deform:
            doConst = bone.parent != None
            phys_dict[bone.name] = ragdoll_utils.createPhysBodyPart(arma_ref,bone,phys_mesh,doConst)
#----------------------------------------------------------------------
    bpy.context.scene.update()
#----------------------------------------------------------------------
    if "ragdoll_physBody_controller" not in arma_ref.game.controllers:
        bpy.ops.logic.controller_add(type="PYTHON",name="ragdoll_physBody_controller",object=arma_ref.name)
    arma_ragCont = arma_ref.game.controllers["ragdoll_physBody_controller"]
    arma_ragCont.mode = "MODULE"
    arma_ragCont.show_expanded = False
    arma_ragCont.module = "ragdoll.spawnPhysicsBody"
    if "ragdoll_runArmature" not in arma_ref.game.actuators:
        bpy.ops.logic.actuator_add(type="ARMATURE",name="ragdoll_runArmature",object=arma_ref.name)
    arma_runArma = arma_ref.game.actuators["ragdoll_runArmature"]
    arma_runArma.mode = "RUN"
    arma_runArma.show_expanded = False
    if "ragdoll_always" not in arma_ref.game.sensors:
        bpy.ops.logic.sensor_add(type="DELAY",name="ragdoll_always",object=arma_ref.name)
    else:
        for cont in arma_ref.game.sensors["ragdoll_always"].controllers:
            arma_ref.game.sensors["ragdoll_always"].unlink(cont)
        arma_ref.game.sensors["ragdoll_always"].link(arma_ragCont)
#----------------------------------------------------------------------
    arma_always = arma_ref.game.sensors["ragdoll_always"]
    arma_always.use_repeat = True
    arma_always.show_expanded = False
    arma_ragCont.link(arma_always,arma_runArma)
#----------------------------------------------------------------------
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
#----------------------------------------------------------------------
    for key in phys_dict:
        phys = phys_dict[key]
        bone = self.bones[key]
        bone_pose = arma_pose[key]
#----------------------------------------------------------------------
        matrixcopy = phys.matrix_world.copy()
        phys.parent = None
        phys.matrix_world = matrixcopy
#----------------------------------------------------------------------
        if len(phys.constraints) != 0:
            const = phys.constraints[0]
#----------------------------------------------------------------------
            try:
                const.target = phys_dict[bone.parent.name]
            except:
                print("PHYS(%s): target not found"%(const.name))
#----------------------------------------------------------------------
            checkName = const.name.lower()
            spine_group = ["spine","neck","head"]
            arm_group = ["arm","hand","clavicle"]
            foot_group = ["thigh","calf","foot"]
            in_spine_group = ragdoll_utils.getIsBoneGroup(checkName,spine_group)
            in_arm_group = ragdoll_utils.getIsBoneGroup(checkName,arm_group)
            in_foot_group = ragdoll_utils.getIsBoneGroup(checkName,foot_group)
#----------------------------------------------------------------------
            if in_foot_group:
                const.use_angular_limit_x = True
                phys.game.mass = .65
                const.pivot_type = "CONE_TWIST"
                if "calf" in checkName:
                    const.pivot_type = "HINGE"
                    const.limit_angle_min_x = radians(65)
                    const.limit_angle_max_x = radians(105)
                elif "thigh" in checkName:
                    (const.limit_angle_min_x,const.limit_angle_min_y,
                    const.limit_angle_min_z) = radians(0),radians(0),radians(0)
                    (const.limit_angle_max_x,const.limit_angle_max_y,
                    const.limit_angle_max_z) = radians(75),radians(1),radians(10)
                else:
                    const.pivot_type = "HINGE"
                phys.scale = (phys.scale[0]/2,phys.scale[1],phys.scale[2]/2)
#----------------------------------------------------------------------
            if in_spine_group:
                phys.game.mass = .8
                const.pivot_type = "CONE_TWIST"
                const.use_angular_limit_x = True
                const.use_angular_limit_y = True
                const.use_angular_limit_z = True
                (const.limit_angle_max_x,const.limit_angle_max_y,
                const.limit_angle_max_z) = radians(75),radians(25),radians(15)
                phys.scale = (phys.scale[0]*2,phys.scale[1],phys.scale[2]*2)
#----------------------------------------------------------------------
            if in_arm_group:
                phys.game.mass = .25
                phys.scale = (phys.scale[0]*1.1,phys.scale[1]/1.2,phys.scale[2]/1.1)
                const.pivot_type = "BALL"
                if "for" in checkName:
                    const.pivot_type = "HINGE"
#----------------------------------------------------------------------
        if "root" in phys.name.lower():
            phys.game.mass = 1.0
            phys.scale = (phys.scale[0]*2,phys.scale[1],phys.scale[2]*2)
            if phys.name+"_limit_distance" not in bone_pose.constraints:
                const = bone_pose.constraints.new(type='LIMIT_DISTANCE')
                const.name = phys.name+"_limit_distance"
            const = bone_pose.constraints[phys.name+"_limit_distance"]
            const.target = phys
            const.active = False
#----------------------------------------------------------------------
        bpy.context.scene.objects.active = phys
        if (arma_ref.name+"_physBody") not in bpy.data.groups:
            bpy.data.groups.new(arma_ref.name+"_physBody")
        bpy.data.groups[arma_ref.name+"_physBody"].objects.link(phys)
#----------------------------------------------------------------------
        bpy.ops.object.game_property_new(type='STRING', name='bone')
        bpy.ops.object.game_property_new(type='STRING', name='constraint')
        phys_boneProp = phys.game.properties["bone"]
        phys_boneProp.value = key
        phys_constProp = phys.game.properties["constraint"]
        phys_constProp.value = phys.name
#----------------------------------------------------------------------
        if "root" in phys.name:
            bpy.ops.object.game_property_new(type='BOOL', name='physRoot')
#----------------------------------------------------------------------
        if arma_pose[key].is_in_ik_chain:
            for constraint in arma_pose[key].constraints:
                if constraint.type == "IK":
#----------------------------------------------------------------------
                    newPhys = ragdoll_utils.createPhysBodyPart(arma_ref,self.bones[constraint.subtarget],phys_mesh,True)
                    ik_rigid = newPhys.constraints[constraint.subtarget]
                    ik_rigid.target = phys
                    newPhys.game.mass = phys.game.mass/10
                    ik_rigid.pivot_type = "HINGE"
                    ik_rigid.use_angular_limit_x = True
                    ik_rigid.limit_angle_min_x = radians(0)
                    ik_rigid.limit_angle_max_x = radians(0)
                    ik_phys_dict[constraint.subtarget] = newPhys
                    ik_pose = arma_pose[constraint.subtarget]
#----------------------------------------------------------------------
                    bpy.ops.object.game_property_new(type='BOOL', name='_IK')
                    constraint.name = key+"_IK"
#----------------------------------------------------------------------
                    if phys.name+"_IK_distance" not in ik_pose.constraints:
                        const = ik_pose.constraints.new(type='LIMIT_DISTANCE')
                        const.name = phys.name+"_IK_distance"
                    const = ik_pose.constraints[phys.name+"_IK_distance"]
                    const.target = newPhys
                    const.show_expanded,const.mute,const.influence = False,True,0.0
#----------------------------------------------------------------------
                    if phys.name+"_IK_rotation" not in ik_pose.constraints:
                        const = ik_pose.constraints.new(type='COPY_ROTATION')
                        const.name = phys.name+"_IK_rotation"
                    const = ik_pose.constraints[phys.name+"_IK_rotation"]
                    const.target = newPhys
                    const.show_expanded,const.mute,const.influence = False,True,0.0
#----------------------------------------------------------------------
                    bpy.ops.object.game_property_new(type='STRING', name='IK_bone')
                    phys_boneProp = phys.game.properties["IK_bone"]
                    phys_boneProp.value = constraint.subtarget
#----------------------------------------------------------------------
        phys.layers[19] = True
        for i in range(len(phys.layers)):
            phys.layers[i] = i == 19
#----------------------------------------------------------------------
        if phys.name not in bone_pose.constraints:
            const = bone_pose.constraints.new(type='COPY_ROTATION')
            const.name = phys.name
        const = bone_pose.constraints[phys.name]
        const.target = phys
        const.show_expanded,const.mute,const.influence = False,True,0.0
#----------------------------------------------------------------------
    for key in ik_phys_dict:
        phys = ik_phys_dict[key]
        matrixcopy = phys.matrix_world.copy()
        phys.parent = None
        phys.matrix_world = matrixcopy
        phys.layers[19] = True
        for i in range(len(phys.layers)):
            phys.layers[i] = i == 19
#----------------------------------------------------------------------
        bpy.data.groups[arma_ref.name+"_physBody"].objects.link(phys)
        bpy.context.scene.objects.active = phys
        bpy.ops.object.game_property_new(type='BOOL', name='noBone')
#----------------------------------------------------------------------
    bpy.context.scene.objects.active = arma_ref
    bpy.context.scene.update()
#----------------------------------------------------------------------
    if arma_ref.name+"_physBody" in bpy.data.groups:
        group = bpy.data.groups[arma_ref.name+"_physBody"]    
        instance = bpy.data.objects.new(group.name, None)
        instance.dupli_type = 'GROUP'
        instance.dupli_group = group
        bpy.context.scene.objects.link(instance)
        instance.layers[18] = True
        for i in range(len(instance.layers)):
            instance.layers[i] = i == 18