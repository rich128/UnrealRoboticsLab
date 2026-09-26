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

#include "CoreMinimal.h"
#include "MjTwinTypes.generated.h"

/**
 * @file MjTwinTypes.h
 * @brief Shared value types for the Unreal-physics twin of a MuJoCo model.
 *
 * The twin is a second, independent representation of the same MJCF model that
 * is simulated by Chaos (Unreal's physics engine) instead of by MuJoCo. Every
 * element that is mirrored carries a fidelity verdict so a user always knows
 * which parts of the replica are exact and which are approximations.
 */

/**
 * @enum EMjTwinFidelity
 * @brief How faithfully a MuJoCo element survived the translation to Chaos.
 *
 * - Exact:       the same numbers drive the same physical model.
 * - Approximate: the element exists but Chaos is a different solver/formulation.
 * - Unsupported: the element has no Chaos equivalent and is not present.
 */
UENUM(BlueprintType)
enum class EMjTwinFidelity : uint8
{
	Exact UMETA(DisplayName = "Exact"),
	Approximate UMETA(DisplayName = "Approximate"),
	Unsupported UMETA(DisplayName = "Unsupported")
};

/**
 * @struct FMjTwinNote
 * @brief One fidelity verdict for one element (or one aspect of one element).
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinNote
{
	GENERATED_BODY()

	/** MuJoCo element the note applies to, e.g. "body/left_hand" or "actuator/knee". */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Element;

	/** Verdict for this element. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	EMjTwinFidelity Fidelity = EMjTwinFidelity::Exact;

	/** Human-readable explanation of what was copied, approximated, or dropped. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Detail;
};

/**
 * @struct FMjTwinReport
 * @brief Fidelity report produced while building a twin.
 *
 * The twin never silently drops part of a model: anything that cannot be
 * mirrored is recorded here, aggregated by fidelity, and logged.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinReport
{
	GENERATED_BODY()

	/** Name of the source model (MuJoCo model name or asset path). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString ModelName;

	/** Every verdict, in build order. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinNote> Notes;

	/** Number of MuJoCo bodies in the source model (including the world body). */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumSourceBodies = 0;

	/** Number of rigid bodies the twin creates in Chaos. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumTwinBodies = 0;

	/** Number of geoms in the source model. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumGeoms = 0;

	/** Number of collision shapes the twin creates. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumTwinShapes = 0;

	/** Number of joints in the source model. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumJoints = 0;

	/** Number of actuators in the source model. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumActuators = 0;

	/** Number of actuators mirrored as force generators. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 NumTwinActuators = 0;

	/** Adds a verdict. */
	void Add(const FString& Element, EMjTwinFidelity Fidelity, const FString& Detail);

	/** Number of notes with the given verdict (or with at least that severity). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	int32 CountOf(EMjTwinFidelity Fidelity) const;

	/** True when at least one element could not be mirrored at all. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool HasUnsupported() const;

	/** One-line summary, e.g. "12 exact, 3 approximate, 0 unsupported". */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	FString SummaryLine() const;

	/** Printable report, one line per note, ordered by severity then element. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	FString ToText() const;

	/** Machine-readable report for tooling and CI. */
	FString ToJson() const;
};

/**
 * @enum EMjTwinShapeType
 * @brief Chaos collision primitive chosen for a MuJoCo geom.
 */
UENUM(BlueprintType)
enum class EMjTwinShapeType : uint8
{
	/** FKBoxElem, exact (MuJoCo box geom). */
	Box UMETA(DisplayName = "Box"),
	/** FKSphereElem, exact (MuJoCo sphere geom). */
	Sphere UMETA(DisplayName = "Sphere"),
	/** FKSphylElem, exact (MuJoCo capsule geom, both are Z-aligned). */
	Capsule UMETA(DisplayName = "Capsule"),
	/** FKConvexElem tessellated prism (MuJoCo cylinder is analytic). */
	Cylinder UMETA(DisplayName = "Cylinder"),
	/** FKConvexElem tessellated ellipsoid (MuJoCo ellipsoid is analytic). */
	Ellipsoid UMETA(DisplayName = "Ellipsoid"),
	/** FKConvexElem copied from the imported mesh asset's simple collision. */
	Mesh UMETA(DisplayName = "Mesh"),
	/** FKBoxElem slab covering the plane's surface (MuJoCo planes are infinite). */
	PlaneSlab UMETA(DisplayName = "Plane (slab)"),
	/** No Chaos equivalent (MuJoCo hfield / SDF geoms). */
	Unsupported UMETA(DisplayName = "Unsupported")
};

/**
 * @enum EMjTwinJointType
 * @brief Chaos constraint family chosen for a MuJoCo joint.
 */
UENUM(BlueprintType)
enum class EMjTwinJointType : uint8
{
	/** No constraint: the body is free (6 dofs). */
	Free UMETA(DisplayName = "Free"),
	FreeSpin UMETA(DisplayName = "Ball (free spin)"),
	/** Twist about the constraint X axis. */
	Hinge UMETA(DisplayName = "Hinge"),
	/** Linear X travel along the constraint X axis. */
	Slide UMETA(DisplayName = "Slide")
};

/**
 * @enum EMjTwinActuatorModel
 * @brief Actuator law written into the twin's force generator.
 */
UENUM(BlueprintType)
enum class EMjTwinActuatorModel : uint8
{
	/** force = gain * ctrl (MuJoCo <motor>). */
	Motor UMETA(DisplayName = "Motor"),
	/** force = kp * (ctrl - q) - kv * qdot (MuJoCo <position>). */
	Position UMETA(DisplayName = "Position"),
	/** force = kv * (ctrl - qdot) (MuJoCo <velocity>). */
	Velocity UMETA(DisplayName = "Velocity"),
	/** General gain/bias with no activation dynamics. */
	GeneralAffine UMETA(DisplayName = "General (affine)"),
	/** Any other law (activation dynamics, muscle, user, plugin). */
	Unsupported UMETA(DisplayName = "Unsupported")
};

/**
 * @struct FMjTwinBuildSettings
 * @brief Options controlling how a twin is instantiated in a world.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinBuildSettings
{
	GENERATED_BODY()

	/** Where the twin is placed. Model coordinates are transformed by this. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FTransform BuildTransform = FTransform::Identity;

	/** Create Unreal visuals (engine primitive meshes / imported mesh assets) for the twin. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bCreateVisuals = true;

	/** Create collision and simulate the twin with Chaos. When false the twin is kinematic only. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bEnableCollision = true;

	/** Apply per-body mass, centre of mass, and inertia tensor from mjModel. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bApplyMassProperties = true;

	/** Create runtime physical materials from MuJoCo geom friction. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bApplyFriction = true;

	/** Configure Unreal's physics substepping to match mjOption.timestep. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bMatchTimestep = true;

	/** Write mjOption.gravity into the world settings (Unreal gravity is -Z only). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bApplyGravity = true;

	/** Wake the twin bodies when the twin is built. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bStartAwake = true;

	/** Evaluate actuator laws inside the physics sub-step instead of once per tick. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bSubstepActuation = false;

	/** Mirror MuJoCo state into the twin every tick (kinematic playback, no Chaos simulation). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bDriveFromMujoco = false;

	/** Log the fidelity report at build time. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bLogReport = true;
};

/**
 * @struct FMjTwinBodyDivergence
 * @brief Per-body pose difference between the twin and the MuJoCo model.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinBodyDivergence
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FString Body;

	/** World position difference in centimetres. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float LinearErrorCm = 0.0f;

	/** World orientation difference in degrees. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float AngularErrorDeg = 0.0f;
};

/**
 * @struct FMjTwinDivergence
 * @brief Result of comparing the twin against the MuJoCo simulation it replicates.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinDivergence
{
	GENERATED_BODY()

	/** True when a comparison was actually performed. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bValid = false;

	/** MuJoCo simulation time the comparison was taken at. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float SimTime = 0.0f;

	/** Worst linear error across all bodies (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float MaxLinearErrorCm = 0.0f;

	/** Worst angular error across all bodies (degrees). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	float MaxAngularErrorDeg = 0.0f;

	/** Per-body breakdown, worst first. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	TArray<FMjTwinBodyDivergence> Bodies;

	/** One-line summary suitable for logs. */
	FString ToText() const;
};
