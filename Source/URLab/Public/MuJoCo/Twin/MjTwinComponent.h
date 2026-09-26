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
#include "Components/SceneComponent.h"
#include "MuJoCo/Twin/MjTwinDescriptor.h"
#include "MuJoCo/Twin/MjTwinTypes.h"
#include "PhysicsEngine/BodyInstance.h"

#include "MjTwinComponent.generated.h"

class UPrimitiveComponent;
class UPhysicsConstraintComponent;
class UMjPhysicsEngine;

/**
 * @struct FMjTwinRuntimeBody
 * @brief Unreal objects that make up one twin rigid body.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinRuntimeBody
{
	GENERATED_BODY()

	/** The component that simulates physics (the body's primary shape). */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	TObjectPtr<UPrimitiveComponent> PhysicsRoot;

	/** Every shape component of the body, including PhysicsRoot. */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	TArray<TObjectPtr<UPrimitiveComponent>> Shapes;

	/** Visual components created for the body (may be empty). */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	TArray<TObjectPtr<USceneComponent>> Visuals;

	/** Pose of the twin physics frame inside the MuJoCo body frame. */
	UPROPERTY(Transient)
	FTransform PhysicsFrameInMjBody = FTransform::Identity;

	/** MuJoCo body id of the group root this body mirrors. */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	int32 MjBodyId = INDEX_NONE;
};

/**
 * @struct FMjTwinRuntimeJoint
 * @brief Unreal objects and cached state of one twin joint.
 */
USTRUCT(BlueprintType)
struct URLAB_API FMjTwinRuntimeJoint
{
	GENERATED_BODY()

	/** Constraint that mirrors the MuJoCo joint. */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	TObjectPtr<UPhysicsConstraintComponent> Constraint;

	/** Joint coordinate in the twin's units (degrees for hinges, cm for slides). */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	double Position = 0.0;

	/** Joint velocity in the twin's units (deg/s or cm/s). */
	UPROPERTY(Transient, BlueprintReadOnly, Category = "MuJoCo|Twin")
	double Velocity = 0.0;
};

/**
 * @class UMjTwinComponent
 * @brief Runs a MuJoCo MJCF model as native Unreal Engine rigid bodies.
 *
 * The component turns a compiled mjModel into a Chaos replica: one Unreal body per
 * MuJoCo rigid body (welded bodies are merged), native collision shapes, native
 * constraints with the same joint limits, the same mass properties, and force
 * generators that evaluate MuJoCo's actuator law every physics sub-step. It is the
 * "MuJoCo clone" companion to the regular URLab pipeline, where MuJoCo keeps the
 * physics and Unreal only renders.
 *
 * Typical use:
 * @code
 * UMjTwinComponent* Twin = NewObject<UMjTwinComponent>(Actor);
 * Twin->RegisterComponent();
 * Twin->BuildFromArticulation(ArticulationActor);
 * Twin->SetControlByName(TEXT("motor1"), 0.4);
 * @endcode
 *
 * Fidelity is reported, never silent: BuildFromArticulation fills FMjTwinReport
 * with one note per mirrored object (Exact / Approximate / Unsupported) and
 * ComputeDivergence measures the twin against the running MuJoCo simulation.
 */
UCLASS(ClassGroup = (URLab), meta = (BlueprintSpawnableComponent, DisplayName = "MuJoCo Physics Twin"),
	HideCategories = (Mobility, LOD, Cooking))
class URLAB_API UMjTwinComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UMjTwinComponent();

	// --- Building ---------------------------------------------------------

	/**
	 * Builds the twin from an articulation whose MuJoCo model has been compiled.
	 * @param ArticulationActor  An AMjArticulation (or any actor owning UMjGeom components).
	 * @return True when a twin was built; check GetReport() for fidelity notes.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool BuildFromArticulation(AActor* ArticulationActor);

	/**
	 * Builds the twin from a descriptor, e.g. one produced by the builder for a
	 * model that is not part of the running simulation.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool BuildFromDescriptor(const FMjTwinDescriptor& InDescriptor);

	/** Destroys every Unreal object the twin created. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	void ClearTwin();

	/** True when a twin is built and simulating. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	bool IsTwinBuilt() const { return bTwinBuilt; }

	// --- Control ----------------------------------------------------------

	/** Sets every actuator control (clamped to ctrlrange unless bClampControls is off). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	void SetControls(const TArray<double>& InControls);

	/** Sets one actuator control by index. */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	void SetControl(int32 ActuatorIndex, double Value);

	/** Sets one actuator control by MuJoCo actuator name (with or without the articulation prefix). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool SetControlByName(FName ActuatorName, double Value);

	/** Resets every control and activation to the model's initial state (ctrl=0, act=0). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	void ResetControls();

	/** Control value of one actuator. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	double GetControl(int32 ActuatorIndex) const;

	/** Activation state of one actuator (only meaningful with activation dynamics). */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	double GetActivation(int32 ActuatorIndex) const;

	/** Generalized force the actuator law produced on the last evaluation (Nm or N, after gear). */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	double GetActuatorForce(int32 ActuatorIndex) const;

	// --- State ------------------------------------------------------------

	/** Fidelity report of the built twin. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	FMjTwinReport GetReport() const { return Descriptor.Report; }

	/** The descriptor the twin was built from. */
	const FMjTwinDescriptor& GetDescriptor() const { return Descriptor; }

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	int32 GetBodyCount() const { return Descriptor.Bodies.Num(); }

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	int32 GetJointCount() const { return Descriptor.Joints.Num(); }

	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	int32 GetActuatorCount() const { return Descriptor.Actuators.Num(); }

	/** Simulating component of a twin body, or null. */
	UFUNCTION(BlueprintPure, Category = "MuJoCo|Twin")
	UPrimitiveComponent* GetBodyComponent(int32 BodyIndex) const;

	/** Joint coordinate and velocity (degrees or cm, per second). */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool GetJointState(int32 JointIndex, double& OutPosition, double& OutVelocity) const;

	/**
	 * Copies the running MuJoCo simulation onto the twin bodies. Only useful with
	 * bDriveFromMujoco (kinematic playback); the twin must not simulate then.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool ApplyMujocoState();

	/**
	 * Measures the twin against the running MuJoCo simulation.
	 * @return False when there is no MuJoCo model or the twin is not built.
	 */
	UFUNCTION(BlueprintCallable, Category = "MuJoCo|Twin")
	bool ComputeDivergence(FMjTwinDivergence& Out);

	// --- Settings ---------------------------------------------------------

	/** Build options: physics, visuals, mass properties, sub-stepping, playback. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	FMjTwinBuildSettings BuildSettings;

	/** Clamp controls to the actuator's ctrlrange (MuJoCo clamps unconditionally). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bClampControls = true;

	/** Log the divergence after every tick (verbose; intended for tuning runs). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bLogDivergence = false;

	/** Log a fidelity summary line when the twin is built. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MuJoCo|Twin")
	bool bLogReportOnBuild = true;

	// --- Component interface ----------------------------------------------

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	/** Physics sub-step callback: evaluates the actuator law at MuJoCo's rate. */
	void OnCalculateCustomPhysicsBody(float DeltaTime, FBodyInstance* BodyInstance);

	/** Evaluates every actuator law and applies the resulting forces. */
	void ApplyActuatorForces(double DeltaSeconds, int32 BodyIndexFilter = INDEX_NONE);

	/** MuJoCo actuator law: gain * act + bias, with activation dynamics and gear. */
	double EvaluateActuator(int32 ActuatorIndex, double DeltaSeconds);

	/** Applies mass, centre of mass, and inertia to the twin bodies. */
	void ApplyMassProperties();

	/** Applies MuJoCo geom friction to the twin shapes. */
	void ApplyFriction();

	/** Creates the constraint of one joint. */
	UPhysicsConstraintComponent* CreateJointConstraint(int32 JointIndex);

	/** Creates the collision component for one descriptor shape (null when unsupported). */
	UPrimitiveComponent* CreateShapeComponent(const FMjTwinShape& Shape);

	/** Creates a render component for one descriptor shape, attached to the body. */
	USceneComponent* CreateVisualComponent(const FMjTwinShape& Shape, USceneComponent* AttachTo);

	/** Reads one joint's coordinate and velocity in MuJoCo units (rad, m, rad/s, m/s). */
	bool GetJointStateMujocoUnits(int32 JointIndex, double& OutPosition, double& OutVelocity) const;

	/** Finds the running MuJoCo engine, or null. */
	UMjPhysicsEngine* FindPhysicsEngine() const;

	/** Unreal frame the twin is built in (component transform * BuildTransform). */
	FTransform GetBaseTransform() const;

	/** Descriptor the twin was built from. */
	FMjTwinDescriptor Descriptor;

	/** Runtime objects per descriptor body. */
	UPROPERTY(Transient)
	TArray<FMjTwinRuntimeBody> Bodies;

	/** Runtime objects per descriptor joint. */
	UPROPERTY(Transient)
	TArray<FMjTwinRuntimeJoint> Joints;

	/** Actuator controls, in MuJoCo units. */
	TArray<double> Controls;

	/** Actuator activation states. */
	TArray<double> Activations;

	/** Scalar force produced by each actuator on the last evaluation (Nm or N). */
	TArray<double> ActuatorForces;

	/** Physics sub-step delegate, bound for every simulating body. */
	FCalculateCustomPhysics OnCalculateCustomPhysics;

	/** True once the twin has been built. */
	bool bTwinBuilt = false;

	/** True when the actuator computation runs inside the physics sub-step. */
	bool bSubstepBound = false;
};
