import numpy as np
from bge.logic import getCurrentScene, getTimeScale
from mathutils import Vector
from math import radians
scene = getCurrentScene()

#//--RAGDOLL SETTINGS--
max_influence = 1.0
influence_step = 0.1
getup_rate = 0.025
logicDelay = 0

groundProp = "ground"; groundMask = 1
raggenSettings = {  "rigidBodyTime":30.0,
 					"cull":6,
 					"max":12,

 					"total":{"reserved":[]}
 				 }

#//--FUNCTION CALLED WHEN RAGDOLLS > MAX RAGDOLLS--
def onRagless(own):
	own.playAction("deathAction", 0, 6, blendin=3)
	
	wipePhysicsBody(own)
	own["ragdoll_timeout"] = True

def trackBone(own):
	armature = own["getArmature"]
	master = armature.parent

	bone_name = own["bone"]
	bone = armature.channels[bone_name]
	const_index = ("%s:%s"%(bone.name,own["constraint"]))
	const = armature.constraints[const_index]

	timerMult = 1
	rigidBodyTime, cull, maxRagdolls = list(raggenSettings.values())[0:3]
	totalRagdolls = np.asarray( list( raggenSettings["total"].values() ))
	totalRagdolls = totalRagdolls[totalRagdolls != 0]
	active = len(totalRagdolls[totalRagdolls == 1])
	total = len(totalRagdolls)	

	doCull = (total > cull) and armature["ragdoll_toggle"]
	dfac = ((total-cull)+1)+logicDelay
	
	if active <= maxRagdolls:
		if raggenSettings["total"][armature["raggenID"]] == -1:
			raggenSettings["total"][armature["raggenID"]] = 1

	if doCull:
		timerMult += dfac*0.05
		armature.sensors["ragdoll_always"].delay = dfac

		if total > maxRagdolls:
			if raggenSettings["total"][armature["raggenID"]] == -1:
				armature.sensors["ragdoll_always"].delay = dfac
				own.localLinearVelocity = [0,0,0]
				return -1

	elif armature.sensors["ragdoll_always"].delay != logicDelay:
		armature.sensors["ragdoll_always"].delay = logicDelay

	if 'init' not in own:
		own["hasRigidBody"] = True
		own["origin_pos"] = own.worldPosition
		own["origin_rot"] = Vector(own.worldOrientation.to_euler())
		const.target = own

		if master.mass <= 1000000: own.mass *= master.mass

		if "physRoot" in own:
			own["z_limit"] = own.getDistanceTo(master)

		if "_IK" in own:
			for _c in armature.constraints:
				if hasattr(_c,"ik_weight"):
					if bone.name in _c.name:
						own["_IK"] = _c
						ik_index = ("%s:%s"%(own["IK_bone"],own["constraint"]
							+"_IK_distance"))
						own["IK_cont_d"] = armature.constraints[ik_index]
						own["IK_cont_d"].target = own
						ik_index = ("%s:%s"%(own["IK_bone"],own["constraint"]
							+"_IK_rotation"))
						own["IK_cont_r"] = armature.constraints[ik_index]
						own["IK_cont_r"].target = own

		own["init"] = True
		own["timer"] = 0.0

	if armature["ragdoll_toggle"]:
		getdown = influence_step*max(1, dfac)
		own["timer"] += getdown+(0.1*timerMult)*getTimeScale()
		if const.active == 0:
			own.worldPosition = (bone.pose_head)+master.worldPosition
			own.worldOrientation = (own["origin_rot"]
			+ Vector(bone.channel_matrix.to_euler()))
			const.active = True

		if not own["hasRigidBody"]:
			own.restorePhysics()
			own.enableRigidBody()
			own["hasRigidBody"] = True			

		try: const.enforce = min(const.enforce+getdown, max_influence)
		except: const.enforce = max_influence

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

			except:
				ik.ik_weight = 0.0
				ik.enforce = 0.0
				ik_cont_d.enforce = 1.0
				ik_cont_r.enforce = 1.0
				ik.active = False

		if "physRoot" in own:

			if own.parent: own.removeParent()
			
			dis_const_index = (
				"%s:%s"%(bone.name,own["constraint"]+"_limit_distance"))
			dis_const = armature.constraints[dis_const_index]
			if not dis_const.active: dis_const.active = True
			
			z_diff = np.linalg.norm( Vector([0,0,own.worldPosition.z])
			-Vector([0,0,master.worldPosition.z]) )

			if z_diff > own["z_limit"]: z_diff = own["z_limit"]
			master.worldPosition.xy = own.worldPosition.xy
			master.worldPosition.z = own.worldPosition.z-z_diff
			
			try:
				dis_const.enforce = min(
					dis_const.enforce+getdown, max_influence)

			except: dis_const.enforce = max_influence

		if own["timer"] >= rigidBodyTime:
			if own["hasRigidBody"]:
				own.disableRigidBody()
				own["hasRigidBody"] = False
				
				own.suspendPhysics()

			if "physRoot" in own:
				unlogID(armature["raggenID"])
				groundRay = master.rayCast(master.worldPosition.copy()
					+Vector([0,0,-0.5]), prop=groundProp,
					 xray=True, mask=groundMask)

				if not groundRay[0]:
					master.restoreDynamics()
					own.setParent(master)

			return 1

	else:
		getup = getup_rate*max(1, dfac/2)
		if own["hasRigidBody"]:
			own.disableRigidBody()
			own.suspendPhysics()
			own["hasRigidBody"] = False

		if "physRoot" in own:
			if own.parent: own.removeParent()

			dis_const_index = (
				"%s:%s"%(bone.name,own["constraint"]+"_limit_distance"))
			dis_const = armature.constraints[dis_const_index]
			
			try: dis_const.enforce = max(0, dis_const.enforce-getup)
			except: dis_const.enforce = 0.0

		if const.enforce != 0:
			getup_rate*dfac
			try: const.enforce = max(0, const.enforce-getup)
			except: const.enforce = 0.0

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

				except:
					ik.ik_weight = 1.0
					ik.enforce = 1.0
					ik_cont_d.enforce = 0.0
					ik_cont_d.active = False
					ik_cont_r.enforce = 0.0
					ik_cont_r.active = False
					own["IK_cont_r"].target = armature
					own["IK_cont_d"].target = armature

		else:
			armature["ragdoll_clear"] = True
			return 1

	return 0

trackBone_ufunc = np.frompyfunc(trackBone, 1, 1)

def genID(own):
	try:
		exists = raggenSettings["total"][ own["raggenID"] ]
		return own["raggenID"]

	except:
		if raggenSettings["total"]["reserved"]:
			return raggenSettings["total"]["reserved"].pop(0)
		
		else:
			count = len(raggenSettings["total"])
			_id = "RG"+( str(0)*(9-len(str(count))) )+str(count)
			return _id

def logID(_id):
	maxRagdolls = list(raggenSettings.values())[2]
	activeRagdolls = np.asarray( list(
	 raggenSettings["total"].values()), dtype=object )
	
	total = len(activeRagdolls[activeRagdolls != 0])
	_max = raggenSettings["max"]
				
	if total < _max: value = 1
	else: value = -1

	raggenSettings["total"][_id] = value

def unlogID(_id):
	raggenSettings["total"][_id] = 0

def reserveID(_id):
	raggenSettings["total"]["reserved"].append(_id)
	
	try: del raggenSettings["total"][_id]
	except: pass

def verifyID(own):
	try:
		if raggenSettings["total"][own["raggenID"]] == 0:
			logID(own["raggenID"])
	
	except:
		own["raggenID"] = genID(own)
		logID(own["raggenID"])

def wipePhysicsBody(own):
	physBody = own["physBody"]
	for phys in physBody.groupMembers:
		if "noBone" not in phys:
			const_index = ("%s:%s"%(phys["bone"],phys["constraint"]))
			if const_index in own.constraints:
				own.constraints[const_index].target = own.parent
			if const_index+"_limit_distance" in own.constraints:
				own.constraints[const_index+"_limit_distance"].target = own.parent

		phys.endObject()
		
	own["physBody"] = own
	own["yesBones"] = []
	reserveID(own["raggenID"])
	physBody.endObject()

def spawnPhysicsBody(own):
	own["yesBones"] = []
	physBody = scene.addObject(own["ragdoll_physGroup"], own)
	for phys in physBody.groupMembers:
		if "noBone" not in phys:
			own["yesBones"].append(phys)
			phys["getArmature"] = own
			const_index = ("%s:%s"%(phys["bone"],phys["constraint"]))
			if const_index in own.constraints:
				own.constraints[const_index].target = phys
			if const_index+"_limit_distance" in own.constraints:
				own.constraints[const_index+"_limit_distance"].target = phys

	own["physBody"] = physBody
	own["raggenID"] = genID(own)
	logID(own["raggenID"])

def ragLogic(cont):
	own = cont.owner
	runArmature = cont.actuators[0]

	if "ragdoll_init" not in own:
		a = np.full(len(own.constraints), False, dtype=object)

		own["ragdoll_update"] = False
		own["ragdoll_timeout"] = False
		own["raggenID"] = genID(own)
		raggenSettings["total"][own["raggenID"]] = False

		physBody = scene.addObject(own["ragdoll_physGroup"], own)
		i = 0
		for phys in physBody.groupMembers:
			if "noBone" not in phys:
				const_index = ("%s:%s"%(phys["bone"],phys["constraint"]))
				if const_index in own.constraints:
					a[i] = own.constraints[const_index]
					i += 1
				if const_index+"_limit_distance" in own.constraints:
					a[i] = (own.constraints[const_index
						+"_limit_distance"])

					i += 1

					own.constraints[const_index+"_limit_distance"].target = own.parent

			phys.endObject()
		
		own["ragdoll_constraints"] = a[a != False]
		physBody.endObject()
		own["physBody"] = own
		own["ragdoll_init"] = True

	if own["ragdoll_toggle"] and not own["ragdoll_timeout"]:
		if own["physBody"] == own:
			spawnPhysicsBody(own)
			if not own.parent.isSuspendDynamics:
				own.parent.suspendDynamics(True)

			own.parent.localLinearVelocity = [0,0,0]

		elif not own["physBody"].invalid:
			verifyID(own)
			turnOff = trackBone_ufunc(own["yesBones"])
			total = len(turnOff)
			own["ragdoll_timeout"] = (
				len(turnOff[turnOff != 0]) == total
				)

			if len(turnOff[turnOff == -1]) == total:
				unlogID(own["raggenID"])
				onRagless(own)

			own["ragdoll_update"] = True
			own["ragdoll_clear"] = False

	elif not own["ragdoll_toggle"] and not own["ragdoll_clear"]:
		if own["physBody"] == own:
			spawnPhysicsBody(own)

		elif not own["physBody"].invalid:
			verifyID(own)
			turnOff = trackBone_ufunc(own["yesBones"])
			total = len(turnOff)
			if own["ragdoll_clear"]:
				for const in own["ragdoll_constraints"]:
					const.active = False

				cont.deactivate(runArmature)
				wipePhysicsBody(own)

				if own.parent.isSuspendDynamics:
					own.parent.localLinearVelocity = [0,0,0]
					own.parent.restoreDynamics()
			
			own["ragdoll_update"] = (
				len(turnOff[turnOff == 1]) != total
				)

			own["ragdoll_timeout"] = False

	if not own["ragdoll_clear"]:
		
		if own["ragdoll_toggle"]:
			if not own.parent.isSuspendDynamics:
				groundRay = own.parent.rayCast(own.parent.worldPosition.copy()
				+Vector([0,0,-0.5]), own.parent.worldPosition.copy()+Vector([0,0,+4]),
				 prop=groundProp, xray=True, mask=groundMask)
				
				if groundRay[0]:
					own.parent.suspendDynamics(True)
					cont.deactivate(runArmature)

				else:
					own.parent.localLinearVelocity.z += 0.1

			elif not own["ragdoll_timeout"]:
				cont.activate(runArmature)
	
	elif own["ragdoll_timeout"]:
		cont.deactivate(runArmature)
