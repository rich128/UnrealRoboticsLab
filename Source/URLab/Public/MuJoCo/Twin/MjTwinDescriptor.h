// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#pragma once

#include <mujoco/mujoco.h>

#include "CoreMinimal.h"
#include "MuJoCo/Twin/MjTwinTypes.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "UObject/SoftObjectPtr.h"

#include "MjTwinDescriptor.generated.h"

class UStaticMesh;

/**
 * @file MjTwinDescriptor.h
 * @brief Engine-agnostic description of a Chaos rigid-body replica of a MJCF model.
 *
 * A descriptor is built from a *compiled* mjModel (the ground truth for the
 * model), not from the XML: every number in it (mass, centre of mass, inertia
 * tensor, geom size, joint frame, actuator gains) is read from the same data
 * MuJoCo itself simulates, so the replica cannot drift from the source model
 * because of a parser difference.
 *
 * The descriptor is a plain value object: no world, no components, no physics
 * state. That keeps the whole MuJoCo -> Chaos translation unit-testable.
 */

/**
 * @namespace MjTwinConv
 * @brief MuJoCo -> Unreal conversions used by the twin.
 *
 * URLab maps MuJoCo's right-handed frame to Unreal's left-handed frame by
 * mirroring the Y axis: p_ue = M p_mj with M = diag(1, -1, 1) and a 100 cm/m
 * scale (see MjUtils). The same mirror applies to orientation as
 * R_ue = M R_mj M^-1, which is exactly what MjUtils::MjToUERotation returns.
 *
 * Mirroring flips the sense of a rotation, so the twin distinguishes:
 *
 *  - Position()  polar vectors:     (x, -y, z)   * 100 cm      - positions, forces
 *  - Direction() polar unit vectors:(x, -y, z)                 - directions, linear velocity
 *  - Axial()     axial vectors:     (-x, y, -z)                - angular velocity, torque,
 *                                                                and hinge axes chosen so that
 *                                                                a positive Unreal rotation about
 *                                                                Axis() equals a positive MuJoCo
 *                                                                joint coordinate
 *  - Rotation()  quaternions:       MjUtils::MjToUERotation    - orientations
 *
 * Every MuJoCo joint coordinate is therefore reproduced with the same sign on
 * the Unreal side, which is what lets ranges, gears, and actuator controls be
 * copied without sign juggling downstream.
 */
namespace MjTwinConv
{
/** Centimetres per metre: MuJoCo works in metres, Unreal in centimetres. */
constexpr double MetersToCentimeters = 100.0;

/** Square of the length conversion, for inertia tensors (kg m^2 -> kg cm^2). */
constexpr double MetersToCentimetersSq = MetersToCentimeters * MetersToCentimeters;

/** Mirrors a MuJoCo position (metres) into an Unreal position (cm). */
FORCEINLINE FVector Position(const double* V)
{
	return FVector(V[0] * MetersToCentimeters, -V[1] * MetersToCentimeters, V[2] * MetersToCentimeters);
}

/** Mirrors a MuJoCo polar vector without unit conversion (directions, unit axes). */
FORCEINLINE FVector Direction(const double* V)
{
	return FVector(V[0], -V[1], V[2]);
}

/** Mirrors a MuJoCo axial vector (angular velocity, torque, rotation axis). */
FORCEINLINE FVector Axial(const double* V)
{
	return FVector(-V[0], V[1], -V[2]);
}

/** Mirrors a MuJoCo quaternion ([w, x, y, z]) into an Unreal quaternion. */
FORCEINLINE FQuat Rotation(const double* Q)
{
	return MjUtils::MjToUERotation(Q);
}

/** Multiplies two MuJoCo quaternions ([w, x, y, z] convention), out = A * B. */
void MulQuat(const double* A, const double* B, double* Out);

/** Rotates a vector by a MuJoCo quaternion. */
void RotateVec(const double* Q, const double* V, double* Out);

/** Inverse of a MuJoCo quaternion. */
void InvQuat(const double* Q, double* Out);
} // namespace MjTwinConv

/**
 * @struct FMjTwinShape
 * @brief One collision primitive of the twin, in the owning body's physics frame.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinShape
{
	GENERATED_BODY()

	/** Index into FMjTwinDescriptor::Bodies (the rigid body this shape is welded to). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 BodyIndex = INDEX_NONE;

	/** MuJoCo geom id in the source model. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 MjGeomId = INDEX_NONE;

	/** MuJoCo geom name (may differ from the Unreal component name). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Name;

	/** Chaos primitive selected for this geom. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	EMjTwinShapeType Type = EMjTwinShapeType::Unsupported;

	/** Shape transform relative to the owning rigid body's physics frame (cm, degrees). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FTransform LocalTransform = FTransform::Identity;

	/**
	 * Shape measurements in cm:
	 *  - Box: half extents (x, y, z)
	 *  - Sphere: (radius, 0, 0)
	 *  - Capsule: (radius, half segment length, 0)
	 *  - Cylinder: (radius, half length, 0)
	 *  - Ellipsoid: (radius x, radius y, radius z)
	 *  - Mesh / PlaneSlab: bounding half extents
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector Size = FVector::ZeroVector;

	/** MuJoCo geom friction (sliding, torsional, rolling). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector Friction = FVector(1.0, 0.005, 0.0001);

	/** True when the geom can generate contacts (contype or conaffinity non-zero). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bCollides = true;

	/** True when the geom should be rendered (MuJoCo visual groups 0-2 in URLab). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bVisual = true;

	/** Raw collision masks, kept for reporting and for the contact matrix. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 ConType = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 ConAffinity = 1;

	/** MuJoCo geom group (0-5). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 Group = 0;

	/** Contact detection margin and gap (m in MuJoCo, cm here). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float Margin = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float Gap = 0.0f;

	/** Index of the MuJoCo mesh this geom uses, or INDEX_NONE. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 MjMeshId = INDEX_NONE;

	/** Unreal mesh asset resolved for a mesh geom (from the imported articulation). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TSoftObjectPtr<UStaticMesh> MeshAsset;

	/** Mesh scale factor applied by the MJCF importer (asset units per MuJoCo metre). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector MeshScale = FVector::OneVector;

	/** True when the geom has no Chaos equivalent and was skipped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bUnsupported = false;
};

/**
 * @struct FMjTwinBody
 * @brief One rigid body of the twin.
 *
 * A MuJoCo body without a joint is welded to its parent, so several MuJoCo
 * bodies can collapse into a single Chaos rigid body. Each twin body therefore
 * lists the MuJoCo bodies it represents and carries the *combined* mass
 * properties of that group.
 */
USTRUCT(BlueprintType)
struct FMjTwinBody
{
	GENERATED_BODY()

	/** Name of the MuJoCo body that owns the group (the twin body's primary name). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Name;

	/** MuJoCo body id of the primary body. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 MjBodyId = INDEX_NONE;

	/** Every MuJoCo body welded into this rigid body (including the primary). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<int32> MjBodyIds;

	/** Index of the parent twin body, or INDEX_NONE when the twin body hangs off the world. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 ParentBodyIndex = INDEX_NONE;

	/** Index of the joint that connects this body to its parent, or INDEX_NONE when welded. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 ParentJointIndex = INDEX_NONE;

	/** Combined mass in kg (sum over the welded group). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double MassKg = 0.0;

	/** Combined inertia, diagonal in the physics frame (kg cm^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector InertiaDiagonal = FVector::ZeroVector;

	/** World position of the physics frame (centre of mass, principal axes), cm. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector PhysicsFrameWorldPos = FVector::ZeroVector;

	/** World rotation of the physics frame (principal inertia axes). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FQuat PhysicsFrameWorldRot = FQuat::Identity;

	/** World pose of the primary MuJoCo body frame (used for reporting and visuals). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector BodyFrameWorldPos = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FQuat BodyFrameWorldRot = FQuat::Identity;

	/** World-space bounds of the body's shapes, relative to the physics frame (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FBox LocalBounds = FBox(ForceInit);

	/** MuJoCo gravity compensation fraction (body_gravcomp). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double GravityCompensation = 0.0;

	/** True for mocap bodies, which MuJoCo does not simulate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bMocap = false;

	/** True when the inertia diagonal is degenerate and a fallback frame was used. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bDegenerateInertia = false;

	/** True when this body is simulated by Chaos (false for mocap/kinematic bodies). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bSimulated = true;
};

/**
 * @struct FMjTwinJoint
 * @brief A joint of the source model and the Chaos constraint that replaces it.
 *
 * Unreal's angular/linear limits are symmetric about the constraint frame, while
 * MuJoCo ranges are arbitrary [min, max]. The descriptor therefore carries the
 * midpoint of the range: the constraint frame is placed/rotated at the midpoint
 * and the symmetric limit is half the range width, which reproduces the MuJoCo
 * range exactly. The true anchor (LimitOffset = 0) is kept separately because
 * joint coordinates are measured from it.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinJoint
{
	GENERATED_BODY()

	/** MuJoCo joint name. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 MjJointId = INDEX_NONE;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	EMjTwinJointType Type = EMjTwinJointType::Hinge;

	/** Index of the parent rigid body in FMjTwinDescriptor::Bodies (INDEX_NONE = world). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 ParentBodyIndex = INDEX_NONE;

	/** Index of the child rigid body in FMjTwinDescriptor::Bodies. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 ChildBodyIndex = INDEX_NONE;

	/** True anchor in world space (cm), i.e. where the joint coordinate is zero. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector AnchorWorldPos = FVector::ZeroVector;

	/** Joint axis in world space: positive rotation/translation equals a positive MuJoCo coordinate. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector AxisWorld = FVector(1, 0, 0);

	/** Constraint frame position: the anchor shifted to the middle of the joint range (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector ConstraintWorldPos = FVector::ZeroVector;

	/** Constraint frame rotation: X axis is the joint axis, rotated to the middle of the range. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FQuat ConstraintWorldRot = FQuat::Identity;

	/** True when the joint has an active limit. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bLimited = false;

	/** Joint range in Unreal units (degrees for hinge/ball, cm for slide), min <= max. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double RangeMin = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double RangeMax = 0.0;

	/** Range midpoint (degrees or cm); the constraint frame is offset by this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double LimitOffset = 0.0;

	/** Half the range width, applied to the symmetric Unreal limit (degrees or cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double HalfRange = 0.0;

	/** Cone (swing) limit for ball joints, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double SwingLimitDeg = 0.0;

	/** Twist limit for ball joints, degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double TwistLimitDeg = 0.0;

	/** MuJoCo dof damping (N m s/rad or N s/m). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Damping = 0.0;

	/** MuJoCo dof friction loss. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double FrictionLoss = 0.0;

	/** MuJoCo dof armature (added inertia), kg m^2 or kg. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Armature = 0.0;

	/** MuJoCo joint stiffness (spring), N m/rad or N/m. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Stiffness = 0.0;

	/** True when a spring is configured (stiffness or damping non-zero). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bHasSpring = false;

	/** Spring reference position from the model's qpos0 (degrees or cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double SpringRef = 0.0;

	/** True when bodies connected by this joint may collide (MuJoCo default is no). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bCollideWithParent = false;

	/** MuJoCo dof address of the joint's first dof, for runtime state lookups. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 DofAdr = INDEX_NONE;

	/** MuJoCo qpos address of the joint, for runtime state lookups. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 QposAdr = INDEX_NONE;

	/** Number of qpos entries the joint owns. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 QposNum = 0;

	/** Number of dofs the joint owns. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 DofNum = 0;

	/** True when the armature was folded into the body's inertia tensor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bArmatureAppliedToInertia = false;

	/** Actuators that drive this joint (indices into FMjTwinDescriptor::Actuators). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<int32> ActuatorIndices;
};

/**
 * @struct FMjTwinActuator
 * @brief A MuJoCo actuator and the force generator that replaces it.
 *
 * The twin evaluates MuJoCo's own actuator law - gain * act + bias, with the
 * activation dynamics of the integrator/filter types - and applies the result to
 * the Chaos body every physics sub-step (or every tick). That keeps the control
 * law identical to MuJoCo's; only the solver that integrates it differs.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinActuator
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 MjActuatorId = INDEX_NONE;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	EMjTwinActuatorModel Model = EMjTwinActuatorModel::Motor;

	/** Index into FMjTwinDescriptor::Joints for joint transmissions, else INDEX_NONE. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 JointIndex = INDEX_NONE;

	/** Index into FMjTwinDescriptor::Bodies that the force is applied to. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 BodyIndex = INDEX_NONE;

	/** True when the transmitted coordinate is an angle (torque) rather than a length (force). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bAngular = true;

	/** Unit axis the force acts about/along, in world space (cm/deg convention). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector AxisWorld = FVector(1, 0, 0);

	/** First gear element: force -> dof force scale for scalar transmissions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Gear = 1.0;

	/** Full gear vector (6 entries), used for multi-dof transmissions. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<double> GearVector;

	/** MuJoCo gaintype / biastype / dyntype, kept raw so the law is evaluated verbatim. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 GainType = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 BiasType = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 DynType = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<double> GainPrm;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<double> BiasPrm;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<double> DynPrm;

	/** Control range (MuJoCo units, absolute). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double CtrlMin = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double CtrlMax = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bCtrlLimited = false;

	/** Force range (MuJoCo units, absolute). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double ForceMin = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double ForceMax = 0.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bForceLimited = false;

	/** Actuator damping, added to the dof damping of the transmitted joint. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Damping = 0.0;

	/** True when the actuator has activation dynamics (dyntype != none). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bHasActivationDynamics = false;

	/** True when the actuator was not mirrored (no Chaos equivalent). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bUnsupported = false;
};

/**
 * @enum EMjTwinEqualityType
 * @brief Chaos mapping chosen for a MuJoCo equality constraint.
 */
UENUM(BlueprintType)
enum class EMjTwinEqualityType : uint8
{
	/** Point-to-point: 3 linear dofs locked, rotation free. */
	Connect UMETA(DisplayName = "Connect"),
	/** Rigid: all 6 dofs locked at the initial relative pose. */
	Weld UMETA(DisplayName = "Weld"),
	/** No Chaos equivalent. */
	Unsupported UMETA(DisplayName = "Unsupported")
};

/**
 * @struct FMjTwinEquality
 * @brief An equality constraint mirrored as a Chaos constraint.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinEquality
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Name;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	EMjTwinEqualityType Type = EMjTwinEqualityType::Unsupported;

	/** Twin body indices; INDEX_NONE means the world body. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 Body1Index = INDEX_NONE;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 Body2Index = INDEX_NONE;

	/** Anchor for connect equalities, world space (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector AnchorWorldPos = FVector::ZeroVector;

	/** Relative pose for weld equalities, second body's physics frame relative to the first. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FTransform RelativeTransform = FTransform::Identity;

	/** True when the equality is active in the source model (eq_active). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bActive = true;
};

/**
 * @struct FMjTwinOptions
 * @brief Model-level simulation settings carried over to Unreal.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinOptions
{
	GENERATED_BODY()

	/** Integration timestep from mjOption (seconds). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Timestep = 0.002;

	/** Gravity in Unreal space and units (cm/s^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FVector Gravity = FVector(0.0, 0.0, -981.0);

	/** True when gravity is (anti-)parallel to the Unreal Z axis and can be applied exactly. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bGravityIsVertical = true;

	/** Raw MuJoCo integrator/cone/solver ids, for the report. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 Integrator = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 Cone = 0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 Solver = 0;

	/** Solver iteration budget, mapped onto Chaos solver iteration counts. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 SolverIterations = 100;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	int32 SolverLSIterations = 50;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Tolerance = 1e-8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	double Impratio = 1.0;

	/** MuJoCo flags that change contact/parent filtering. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bFilterParent = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bContactsEnabled = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bLimitsEnabled = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bEqualityEnabled = true;

	/** Continuous collision detection requested by the model. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bCCD = false;
};

/**
 * @struct FMjTwinDescriptor
 * @brief Complete Chaos-side description of a MJCF model.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinDescriptor
{
	GENERATED_BODY()

	/** Source model name. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString ModelName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinBody> Bodies;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinShape> Shapes;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinJoint> Joints;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinActuator> Actuators;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinEquality> Equalities;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FMjTwinOptions Options;

	/** Fidelity report collected while translating the model. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FMjTwinReport Report;

	/** True when the descriptor has at least one body. */
	bool IsValid() const { return Bodies.Num() > 0; }

	/** Index of the twin body that contains the given MuJoCo body id, or INDEX_NONE. */
	int32 FindBodyByMjId(int32 MjBodyId) const;

	/** Index of the twin body with the given MuJoCo body name, or INDEX_NONE. */
	int32 FindBodyByName(const FString& BodyName) const;

	/** Index of the joint with the given MuJoCo joint name, or INDEX_NONE. */
	int32 FindJointByName(const FString& JointName) const;

	/** Index of the actuator with the given MuJoCo actuator name, or INDEX_NONE. */
	int32 FindActuatorByName(const FString& ActuatorName) const;
};

/**
 * @struct FMjTwinTensor
 * @brief A symmetric 3x3 tensor in double precision.
 *
 * Unreal's FMatrix is single precision, but inertia tensors of welded body groups are
 * combined here with differences well below float resolution, and the twin promises an
 * exact diagonal copy to Chaos, so the tensor stays double throughout.
 */
struct URLAB_API FMjTwinTensor
{
	double M[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
};

/**
 * @class FMjTwinDescriptorBuilder
 * @brief Builds a twin descriptor from a compiled MuJoCo model.
 */
class URLAB_API FMjTwinDescriptorBuilder
{
public:
	/**
	 * @brief Translates a compiled mjModel into a Chaos-side descriptor.
	 *
	 * @param Model       Compiled model to replicate. Must outlive the call only.
	 * @param Out         Descriptor to fill.
	 * @param OutError    Human-readable error when the build fails.
	 * @param MeshAssets  Optional map of MuJoCo mesh name -> imported Unreal mesh,
	 *                    so mesh geoms get real collision instead of a bounding box.
	 */
	static bool BuildFromModel(const mjModel* Model, FMjTwinDescriptor& Out, FString& OutError,
		const TMap<FString, TSoftObjectPtr<UStaticMesh>>* MeshAssets = nullptr);

	/**
	 * @brief Builds a descriptor for an actor whose MuJoCo model has been compiled.
	 *
	 * Reads the compiled model from the MuJoCo manager and picks up the mesh assets
	 * the MJCF importer created for the actor's geom components.
	 *
	 * @param Articulation  Usually an AMjArticulation, but any actor that owns UMjGeom
	 *                      components works.
	 */
	static bool BuildFromArticulation(class AActor* Articulation, FMjTwinDescriptor& Out, FString& OutError);

	/** Tessellated cylinder point cloud (convex hull input), centred on the origin. */
	static TArray<FVector> MakeCylinderPoints(float RadiusCm, float HalfLengthCm, int32 Sides = 32);

	/** Tessellated ellipsoid point cloud (convex hull input), centred on the origin. */
	static TArray<FVector> MakeEllipsoidPoints(const FVector& RadiiCm, int32 Segments = 24, int32 Rings = 12);

	/**
	 * @brief Builds a constraint frame whose X axis is the joint axis.
	 *
	 * Unreal maps twist to the constraint X axis, swing1 to Z, swing2 to Y, and
	 * linear X/Y/Z to the frame axes.
	 */
	static FQuat MakeConstraintRotation(const FVector& AxisWorld);

	/** Rotation about the joint axis by the middle of the joint range. */
	static FQuat MakeRangeOffsetRotation(const FVector& AxisWorld, double MidpointDegrees);

	/**
	 * @brief Diagonalises a symmetric 3x3 inertia tensor about a centre of mass.
	 *
	 * Returns the principal axes as a rotation and the principal moments. Used to
	 * combine the inertia of welded MuJoCo bodies - Chaos stores a diagonal
	 * inertia in the body frame, so the body frame is chosen to be the principal
	 * frame and the copy is exact.
	 */
	static bool DiagonalizeInertia(const FMjTwinTensor& Tensor, FQuat& OutAxes, FVector& OutMoments);
};
