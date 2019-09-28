import bpy
#----------------------------------------------------------------------
def createPhysBodyPart(arma_ref,bone,phys_mesh,doConst):
    newPhys = bpy.data.objects.new(arma_ref.name+"_phys_"+bone.name,phys_mesh)
    bpy.context.scene.objects.link(newPhys)
#----------------------------------------------------------------------
    newPhys.location[1] = -bone.length
    newPhys.scale = (bone.length/2,bone.length/1.25,bone.length/2)
#----------------------------------------------------------------------
    newPhys.parent = arma_ref
    newPhys.parent_bone = bone.name
    newPhys.parent_type = "BONE"
#----------------------------------------------------------------------
    gameSet = newPhys.game
    (gameSet.physics_type,gameSet.collision_bounds_type,
    gameSet.use_collision_bounds) = ("RIGID_BODY","CONVEX_HULL",True)
    gameSet.collision_mask = arma_ref.game.collision_mask
    gameSet.collision_group = arma_ref.game.collision_group
    gameSet.friction = arma_ref.game.friction
    gameSet.elasticity = arma_ref.game.elasticity
    gameSet.collision_margin = arma_ref.game.collision_margin
    gameSet.radius = arma_ref.game.radius
    gameSet.damping = arma_ref.game.damping
#----------------------------------------------------------------------
    if doConst:
        const = newPhys.constraints.new(type='RIGID_BODY_JOINT')
        const.name = bone.name
        const.show_pivot,const.use_linked_collision = True,True
#----------------------------------------------------------------------
    newPhys.hide_render = True
    newPhys.draw_type = "WIRE"
    return newPhys
#----------------------------------------------------------------------
def getIsBoneGroup(bone_name,bone_group):
    for s in bone_group:
        if s in bone_name:
            return True
    return False