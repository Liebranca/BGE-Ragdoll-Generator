from bge import logic
from mathutils import Vector
from math import radians
scene = logic.getCurrentScene()
#--------------------------------------------------------------------
def trackBone(own):
    armature = own["getArmature"]
    master = armature.parent
    bone_name = own["bone"]
    bone = armature.channels[bone_name]
    const_index = ("%s:%s"%(bone.name,own["constraint"]))
    const = armature.constraints[const_index]
#--------------------------------------------------------------------
    if 'init' not in own:
        own["origin_pos"] = own.worldPosition
        own["origin_rot"] = Vector(own.worldOrientation.to_euler())
        if "physRoot" in own:
            own["z_limit"] = own.getDistanceTo(master)
#--------------------------------------------------------------------
        if "_IK" in own:
            for _c in armature.constraints:
                if hasattr(_c,"ik_weight"):
                    if bone.name in _c.name:
                        own["_IK"] = _c
                        ik_index = ("%s:%s"%(own["IK_bone"],own["constraint"]+"_IK_distance"))
                        own["IK_cont_d"] = armature.constraints[ik_index]
                        own["IK_cont_d"].target = own
                        ik_index = ("%s:%s"%(own["IK_bone"],own["constraint"]+"_IK_rotation"))
                        own["IK_cont_r"] = armature.constraints[ik_index]
                        own["IK_cont_r"].target = own
        own["init"] = True
#--------------------------------------------------------------------
    pos = (bone.pose_head)+master.worldPosition
    rot = own["origin_rot"] + Vector(bone.channel_matrix.to_euler()) 
#--------------------------------------------------------------------
    if const.active == 0:
        own.worldPosition = pos
        own.worldOrientation = rot
        const.active = True
#--------------------------------------------------------------------
    if armature["ragdoll_toggle"]:
        if own.isSuspendDynamics:
            own.restoreDynamics()
        if const.enforce != 1:
            if const.enforce < 0.75:
                const.enforce += 0.25
            else:
                const.enforce = 1
#--------------------------------------------------------------------
            if "_IK" in own:
                ik = own["_IK"]
                ik_cont_d = own["IK_cont_d"]
                ik_cont_r = own["IK_cont_r"]
                ik_cont_d.active = True
                ik_cont_r.active = True
                try:
                    ik.ik_weight -= 0.25
                    ik.enforce -= 0.25
                    ik_cont_d.enforce += 0.25
                    ik_cont_r.enforce += 0.25
#--------------------------------------------------------------------
                except:
                    ik.ik_weight = 0.0
                    ik.enforce = 0.0
                    ik_cont_d.enforce = 1.0
                    ik_cont_r.enforce = 1.0
                    ik.active = False
#--------------------------------------------------------------------
        elif "physRoot" in own:
            master.worldPosition.xy = own.worldPosition.xy
            master.worldPosition.z = own.worldPosition.z
            dis_const_index = ("%s:%s"%(bone.name,own["constraint"]+"_limit_distance"))
            dis_const = armature.constraints[dis_const_index]
            if dis_const.enforce != 1:
                if dis_const.enforce < 0.75:
                    dis_const.enforce += 0.25
                else:
                    dis_const.enforce = 1
                    dis_const.active = True
#--------------------------------------------------------------------
    else:
        if master.isSuspendDynamics:
            master.localLinearVelocity = [0,0,0]
            master.worldPosition = armature.worldPosition
            master.restoreDynamics()
        if "physRoot" in own:
            dis_const_index = ("%s:%s"%(bone.name,own["constraint"]+"_limit_distance"))
            dis_const = armature.constraints[dis_const_index]
            if dis_const.enforce != 0:
                if dis_const.enforce > 0.025:
                    dis_const.enforce -= 0.025
                else:
                    dis_const.enforce = 0
                    dis_const.active = False
        if const.enforce != 0:
            if const.enforce > 0.025:
                const.enforce -= 0.025
                own.worldPosition = pos
            else:
                const.enforce = 0
                armature["ragdoll_clear"] = True
                const.active = False
#--------------------------------------------------------------------
            if "_IK" in own:
                ik = own["_IK"]
                ik.active = True
                ik_cont_d = own["IK_cont_d"]
                ik_cont_r = own["IK_cont_r"]
                try:
                    ik.ik_weight += 0.025
                    ik.enforce += 0.025
                    ik_cont_d.enforce -= 0.025
                    ik_cont_r.enforce -= 0.025
#--------------------------------------------------------------------
                except:
                    ik.ik_weight = 1.0
                    ik.enforce = 1.0
                    ik_cont_d.enforce = 0.0
                    ik_cont_d.active = False
                    ik_cont_r.enforce = 0.0
                    ik_cont_r.active = False
                    own["IK_cont_r"].target = armature
                    own["IK_cont_d"].target = armature
#--------------------------------------------------------------------
        else:
            own.worldPosition = pos
            own.worldOrientation = rot
#--------------------------------------------------------------------
def spawnPhysicsBody(cont):
    own = cont.owner
    runArmature = cont.actuators[0]
#--------------------------------------------------------------------
    if "ragdoll_init" not in own:
        own["ragdoll_bones"] = []
        own["ragdoll_constraints"] = []
        physBody = scene.addObject(own["ragdoll_physGroup"])
        for phys in physBody.groupMembers:
            if "noBone" not in phys:
                const_index = ("%s:%s"%(phys["bone"],phys["constraint"]))
                if const_index in own.constraints:
                    own["ragdoll_bones"].append(own.channels[phys["bone"]])
                    own["ragdoll_constraints"].append(own.constraints[const_index])
                if const_index+"_limit_distance" in own.constraints:
                    own["ragdoll_constraints"].append(own.constraints[const_index+"_limit_distance"])
        own["physBody"] = own
        own["ragdoll_init"] = True
        physBody.endObject()
#--------------------------------------------------------------------
    if own["ragdoll_toggle"]:
        if own.parent != None:
            if not own.parent.isSuspendDynamics:
                own.parent.suspendDynamics(True)
            own.parent.localLinearVelocity = [0,0,0]
        own["ragdoll_clear"] = False
        if own["physBody"] == own:
            physBody = scene.addObject(own["ragdoll_physGroup"],own.parent)
#--------------------------------------------------------------------
            for newPhys in physBody.groupMembers:
                if "noBone" not in newPhys:
                    newPhys["getArmature"] = own
                    const_index = ("%s:%s"%(newPhys["bone"],newPhys["constraint"]))
                    const = own.constraints[const_index]
                    const.target = newPhys
                    if "physRoot" in newPhys:
                        const = own.constraints[const_index+"_limit_distance"]
                        const.target = newPhys
                        const.active = True
                        const.enforce = 1.0
            own["physBody"] = physBody
#--------------------------------------------------------------------
        elif not own["physBody"].invalid:
            for phys in own["physBody"].groupMembers:
                if "noBone" not in phys:
                    trackBone(phys)
#--------------------------------------------------------------------
    elif own["physBody"] != own:
        if own.parent != None:
            own.parent.localLinearVelocity = [0,0,0]
        if not own["physBody"].invalid:
            for phys in own["physBody"].groupMembers:
                if "noBone" not in phys:
                    trackBone(phys)
#--------------------------------------------------------------------
            if own["ragdoll_clear"]:
                physBody = own["physBody"]
                for const in own["ragdoll_constraints"]:
                    const.active = False
                    const.target = own.parent
                physBody.endObject()
                own["physBody"] = own
                cont.deactivate(runArmature)
#--------------------------------------------------------------------
    if not own["ragdoll_clear"]:
        cont.activate(runArmature)