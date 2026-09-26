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

#include "MuJoCo/Twin/MjTwinComponent.h"

#include "Components/BoxComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Core/MjRenderSnapshot.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/ConvexElem.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "PhysicsEngine/PhysicsSettings.h"
#include "ProceduralMeshComponent.h"
#include "Utils/URLabLogging.h"

namespace
{
/** 1 Nm = 1e4 kg cm^2/s^2, the torque unit Unreal's AddTorqueInRadians expects. */
constexpr double NewtonMeterToUE = 10000.0;

/** 1 N = 100 kg cm/s^2, the force unit Unreal's AddForce expects. */
constexpr double NewtonToUE = 100.0;

/** Cubic centimetres per cubic metre, for kg/m^3 -> kg/cm^3 material density. */
constexpr double CubicMetersToCubicCentimeters = 1.0e6;

/** Substitution radius used when a body has no mirrored shape. */
constexpr double FallbackHalfExtentCm = 5.0;

/** Collision profile used by the twin's shapes. */
const TCHAR* TwinCollisionProfile = TEXT("PhysicsActor");

/** One ring of a revolved shape: radius factor and Z offset along the axis. */
struct FMjTwinRing
{
	double RadiusFactor = 1.0;
	double Z = 0.0;

	FMjTwinRing(double InFactor, double InZ) : RadiusFactor(InFactor), Z(InZ) {}
};

/** Appends a quad (two triangles) with the outward winding used by the generators. */
void AddQuad(TArray<int32>& Tris, int32 A, int32 B, int32 C, int32 D)
{
	Tris.Add(A); Tris.Add(B); Tris.Add(C);
	Tris.Add(A); Tris.Add(C); Tris.Add(D);
}

/**
 * Revolves a profile around the Z axis to build render geometry.
 * Rings go from -Z to +Z, each with Segments vertices.
 */
void AddRevolvedShape(const TArray<FMjTwinRing>& Rings, int32 Segments, double RadiusX, double RadiusY,
	TArray<FVector>& OutVertices, TArray<int32>& OutTriangles)
{
	const int32 FirstVertex = OutVertices.Num();
	for (const FMjTwinRing& Ring : Rings)
	{
		for (int32 Segment = 0; Segment < Segments; ++Segment)
		{
			const double Angle = 2.0 * PI * static_cast<double>(Segment) / static_cast<double>(Segments);
			OutVertices.Add(FVector(
				FMath::Cos(Angle) * Ring.RadiusFactor * RadiusX,
				FMath::Sin(Angle) * Ring.RadiusFactor * RadiusY,
				Ring.Z));
		}
	}

	for (int32 Ring = 0; Ring + 1 < Rings.Num(); ++Ring)
	{
		for (int32 Segment = 0; Segment < Segments; ++Segment)
		{
			const int32 Next = (Segment + 1) % Segments;
			const int32 A = FirstVertex + Ring * Segments + Segment;
			const int32 B = FirstVertex + Ring * Segments + Next;
			const int32 C = FirstVertex + (Ring + 1) * Segments + Next;
			const int32 D = FirstVertex + (Ring + 1) * Segments + Segment;
			AddQuad(OutTriangles, A, B, C, D);
		}
	}
}

/** Caps a revolved shape whose end ring has RadiusX/RadiusY at Z. */
void AddCap(const TArray<FMjTwinRing>& Rings, int32 Segments, bool bTop, double RadiusX, double RadiusY,
	TArray<FVector>& OutVertices, TArray<int32>& OutTriangles)
{
	const int32 RingIndex = bTop ? Rings.Num() - 1 : 0;
	const double Z = Rings[RingIndex].Z;
	const int32 Centre = OutVertices.Add(FVector(0.0, 0.0, Z));
	const int32 RingStart = OutVertices.Num() - Segments - 1;

	for (int32 Segment = 0; Segment < Segments; ++Segment)
	{
		const int32 Next = (Segment + 1) % Segments;
		const int32 A = RingStart + Segment;
		const int32 B = RingStart + Next;
		if (bTop)
		{
			OutTriangles.Add(Centre); OutTriangles.Add(A); OutTriangles.Add(B);
		}
		else
		{
			OutTriangles.Add(Centre); OutTriangles.Add(B); OutTriangles.Add(A);
		}
	}
}

/** Render geometry for the twin's primitives, all centred on the shape origin. */
void BuildShapeRenderData(EMjTwinShapeType Type, const FVector& Size, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles)
{
	constexpr int32 Segments = 24;

	switch (Type)
	{
		case EMjTwinShapeType::Box:
		case EMjTwinShapeType::PlaneSlab:
		{
			const FVector H = Size;
			static const FVector Signs[8] = {
				FVector(-1, -1, -1), FVector(1, -1, -1), FVector(1, 1, -1), FVector(-1, 1, -1),
				FVector(-1, -1, 1), FVector(1, -1, 1), FVector(1, 1, 1), FVector(-1, 1, 1)};
			for (const FVector& Sign : Signs)
			{
				OutVertices.Add(FVector(Sign.X * H.X, Sign.Y * H.Y, Sign.Z * H.Z));
			}
			AddQuad(OutTriangles, 0, 3, 2, 1);
			AddQuad(OutTriangles, 4, 5, 6, 7);
			AddQuad(OutTriangles, 0, 1, 5, 4);
			AddQuad(OutTriangles, 1, 2, 6, 5);
			AddQuad(OutTriangles, 2, 3, 7, 6);
			AddQuad(OutTriangles, 3, 0, 4, 7);
			break;
		}

		case EMjTwinShapeType::Sphere:
		case EMjTwinShapeType::Ellipsoid:
		{
			const double RadiusX = Type == EMjTwinShapeType::Sphere ? Size.X : Size.X;
			const double RadiusY = Type == EMjTwinShapeType::Sphere ? Size.X : Size.Y;
			const double RadiusZ = Type == EMjTwinShapeType::Sphere ? Size.X : Size.Z;

			TArray<FMjTwinRing> Rings;
			constexpr int32 RingsCount = 12;
			for (int32 Ring = 0; Ring <= RingsCount; ++Ring)
			{
				const double Phi = PI * static_cast<double>(Ring) / static_cast<double>(RingsCount);
				Rings.Add(FMjTwinRing(FMath::Sin(Phi), FMath::Cos(Phi) * RadiusZ));
			}
			AddRevolvedShape(Rings, Segments, RadiusX, RadiusY, OutVertices, OutTriangles);
			break;
		}

		case EMjTwinShapeType::Cylinder:
		{
			// Degenerate end rings (radius factor 0) close the two caps with fans.
			const TArray<FMjTwinRing> Rings = {
				FMjTwinRing(0.0, -Size.Y), FMjTwinRing(1.0, -Size.Y),
				FMjTwinRing(1.0, Size.Y), FMjTwinRing(0.0, Size.Y)};
			AddRevolvedShape(Rings, Segments, Size.X, Size.X, OutVertices, OutTriangles);
			break;
		}

		case EMjTwinShapeType::Capsule:
		{
			const double Radius = Size.X;
			const double HalfSegment = Size.Y;

			TArray<FMjTwinRing> Rings;
			constexpr int32 CapRings = 6;
			for (int32 Ring = 0; Ring <= CapRings; ++Ring)
			{
				const double Phi = PI * 0.5 * static_cast<double>(Ring) / static_cast<double>(CapRings);
				Rings.Add(FMjTwinRing(FMath::Sin(Phi), -HalfSegment - Radius * FMath::Cos(Phi)));
			}
			for (int32 Ring = 0; Ring <= CapRings; ++Ring)
			{
				const double Phi = PI * 0.5 * static_cast<double>(Ring) / static_cast<double>(CapRings);
				Rings.Add(FMjTwinRing(FMath::Cos(Phi), HalfSegment + Radius * FMath::Sin(Phi)));
			}
			AddRevolvedShape(Rings, Segments, Radius, Radius, OutVertices, OutTriangles);
			break;
		}

		default:
			break;
	}
}

/** Child transform expressed in the parent's frame. */
FTransform MakeRelativeTransform(const FTransform& ChildWorld, const FTransform& ParentWorld)
{
	const FQuat InvParent = ParentWorld.GetRotation().Inverse();
	const FVector LocalPosition = InvParent.RotateVector(ChildWorld.GetTranslation() - ParentWorld.GetTranslation());
	return FTransform(InvParent * ChildWorld.GetRotation(), LocalPosition, FVector::OneVector);
}
} // namespace

UMjTwinComponent::UMjTwinComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}

FTransform UMjTwinComponent::GetBaseTransform() const
{
	return GetComponentTransform() * BuildSettings.BuildTransform;
}

bool UMjTwinComponent::BuildFromArticulation(AActor* ArticulationActor)
{
	FMjTwinDescriptor Built;
	FString Error;
	if (!FMjTwinDescriptorBuilder::BuildFromArticulation(ArticulationActor, Built, Error))
	{
		UE_LOG(LogURLabRuntime, Warning, TEXT("[MjTwin] '%s' could not build a twin: %s"), *GetName(), *Error);
		return false;
	}
	return BuildFromDescriptor(Built);
}

bool UMjTwinComponent::BuildFromDescriptor(const FMjTwinDescriptor& InDescriptor)
{
	ClearTwin();

	Descriptor = InDescriptor;
	if (!Descriptor.IsValid())
	{
		UE_LOG(LogURLabRuntime, Warning, TEXT("[MjTwin] '%s' received an empty descriptor."), *GetName());
		return false;
	}

	const FTransform Base = GetBaseTransform();
	Bodies.SetNum(Descriptor.Bodies.Num());
	Joints.SetNum(Descriptor.Joints.Num());
	Controls.Init(0.0, Descriptor.Actuators.Num());
	Activations.Init(0.0, Descriptor.Actuators.Num());
	ActuatorForces.Init(0.0, Descriptor.Actuators.Num());

	// Match MuJoCo's integration rate: sub-stepping is what makes a Chaos twin track
	// mj_step closely, so the build can turn it on and set the step from mjOption.
	if (BuildSettings.bMatchTimestep && Descriptor.Options.Timestep > 0.0)
	{
		if (UPhysicsSettings* Settings = UPhysicsSettings::Get())
		{
			Settings->bSubstepping = true;
			Settings->MaxSubstepDeltaTime = FMath::Clamp(Descriptor.Options.Timestep, 1.0 / 1000.0, 0.05);
			Settings->MaxSubsteps = FMath::Clamp(FMath::CeilToInt(0.033 / Settings->MaxSubstepDeltaTime), 1, 64);
		}
	}

	// --- Bodies ------------------------------------------------------------
	for (int32 BodyIndex = 0; BodyIndex < Descriptor.Bodies.Num(); ++BodyIndex)
	{
		const FMjTwinBody& Body = Descriptor.Bodies[BodyIndex];
		FMjTwinRuntimeBody& Runtime = Bodies[BodyIndex];
		Runtime.MjBodyId = Body.MjBodyId;
		Runtime.PhysicsFrameInMjBody = MakeRelativeTransform(
			FTransform(Body.PhysicsFrameWorldRot, Body.PhysicsFrameWorldPos, FVector::OneVector),
			FTransform(Body.BodyFrameWorldRot, Body.BodyFrameWorldPos, FVector::OneVector));

		const FTransform BodyTransform = Base * FTransform(Body.PhysicsFrameWorldRot, Body.PhysicsFrameWorldPos, FVector::OneVector);

		// First shape becomes the simulating component; the rest are welded to it.
		for (int32 ShapeIndex = 0; ShapeIndex < Descriptor.Shapes.Num(); ++ShapeIndex)
		{
			const FMjTwinShape& Shape = Descriptor.Shapes[ShapeIndex];
			if (Shape.BodyIndex != BodyIndex || Shape.bUnsupported)
			{
				continue;
			}

			UPrimitiveComponent* Component = CreateShapeComponent(Shape);
			if (!Component)
			{
				continue;
			}

			if (Runtime.PhysicsRoot == nullptr)
			{
				Runtime.PhysicsRoot = Component;
				Component->SetupAttachment(this);
				Component->SetMobility(EComponentMobility::Movable);
				Component->RegisterComponent();
				Component->SetWorldTransform(BodyTransform * Shape.LocalTransform, false, nullptr, ETeleportType::TeleportPhysics);
			}
			else
			{
				// Extra shapes are welded into the root body: they must not simulate on
				// their own, but their collision belongs to the same rigid body.
				Component->SetupAttachment(Runtime.PhysicsRoot);
				Component->SetMobility(EComponentMobility::Movable);
				Component->RegisterComponent();
				Component->SetRelativeTransform(Shape.LocalTransform, false, nullptr, ETeleportType::TeleportPhysics);
				Component->SetSimulatePhysics(false);
				if (FBodyInstance* ShapeBody = Component->GetBodyInstance())
				{
					ShapeBody->bAutoWeld = true;
				}
			}

			Runtime.Shapes.Add(Component);
			if (BuildSettings.bCreateVisuals)
			{
				if (USceneComponent* Visual = CreateVisualComponent(Shape, Runtime.PhysicsRoot))
				{
					Runtime.Visuals.Add(Visual);
				}
			}
		}

		if (Runtime.PhysicsRoot == nullptr && !Body.bSimulated && !Body.bMocap)
		{
			// A static body without mirrored shapes (the world group of a model whose
			// static geoms are all heightfields, for example) needs no Unreal object.
			continue;
		}

		if (Runtime.PhysicsRoot == nullptr)
		{
			// No mirrored shape (for example a body that only carries the world's
			// heightfield): give Chaos a small collision box so the body still exists.
			UBoxComponent* Fallback = NewObject<UBoxComponent>(this, NAME_None, RF_Transient);
			Fallback->SetupAttachment(this);
			Fallback->SetMobility(EComponentMobility::Movable);
			Fallback->InitBoxExtent(FVector(FallbackHalfExtentCm));
			Fallback->RegisterComponent();
			Fallback->SetWorldTransform(BodyTransform, false, nullptr, ETeleportType::TeleportPhysics);
			Fallback->SetCollisionEnabled(BuildSettings.bEnableCollision ? ECollisionEnabled::QueryAndPhysics : ECollisionEnabled::NoCollision);
			Runtime.PhysicsRoot = Fallback;
			Runtime.Shapes.Add(Fallback);
			UE_LOG(LogURLabRuntime, Warning,
				TEXT("[MjTwin] body '%s' has no mirrored shape; a %.0f cm box stands in for it."), *Body.Name, FallbackHalfExtentCm);
		}

		UPrimitiveComponent* Root = Runtime.PhysicsRoot;
		Root->SetCollisionEnabled(BuildSettings.bEnableCollision ? ECollisionEnabled::QueryAndPhysics : ECollisionEnabled::NoCollision);
		Root->SetCollisionProfileName(TwinCollisionProfile);
		Root->SetGenerateOverlapEvents(false);
		if (!Body.bSimulated)
		{
			// Mocap bodies are kinematic in MuJoCo as well.
			Root->SetSimulatePhysics(false);
			Root->SetMobility(EComponentMobility::Movable);
		}
		else if (BuildSettings.bEnableCollision && !BuildSettings.bDriveFromMujoco)
		{
			Root->SetSimulatePhysics(true);
		}
		if (BuildSettings.bStartAwake && Root->IsSimulatingPhysics())
		{
			Root->WakeRigidBody();
		}
	}

	// --- Constraints -------------------------------------------------------
	for (int32 JointIndex = 0; JointIndex < Descriptor.Joints.Num(); ++JointIndex)
	{
		Joints[JointIndex].Constraint = CreateJointConstraint(JointIndex);
	}

	// --- Physics properties -------------------------------------------------
	if (BuildSettings.bApplyMassProperties)
	{
		ApplyMassProperties();
	}
	if (BuildSettings.bApplyFriction)
	{
		ApplyFriction();
	}

	// --- Actuation ---------------------------------------------------------
	if (BuildSettings.bSubstepActuation && !BuildSettings.bDriveFromMujoco)
	{
		OnCalculateCustomPhysics.BindUObject(this, &UMjTwinComponent::OnCalculateCustomPhysicsBody);
		bSubstepBound = true;
	}

	bTwinBuilt = true;
	if (bLogReportOnBuild)
	{
		UE_LOG(LogURLabRuntime, Log, TEXT("[MjTwin] %s"), *Descriptor.Report.SummaryLine());
		for (const FMjTwinNote& Note : Descriptor.Report.Notes)
		{
			if (Note.Fidelity != EMjTwinFidelity::Exact)
			{
				UE_LOG(LogURLabRuntime, Log, TEXT("[MjTwin] %s: %s"), *Note.Element, *Note.Detail);
			}
		}
	}
	return true;
}

void UMjTwinComponent::ClearTwin()
{
	for (FMjTwinRuntimeBody& Body : Bodies)
	{
		for (USceneComponent* Visual : Body.Visuals)
		{
			if (Visual)
			{
				Visual->DestroyComponent();
			}
		}
		for (UPrimitiveComponent* Shape : Body.Shapes)
		{
			if (Shape && Shape != Body.PhysicsRoot)
			{
				Shape->DestroyComponent();
			}
		}
		if (Body.PhysicsRoot)
		{
			Body.PhysicsRoot->DestroyComponent();
		}
	}
	Bodies.Reset();

	for (FMjTwinRuntimeJoint& Joint : Joints)
	{
		if (Joint.Constraint)
		{
			Joint.Constraint->DestroyComponent();
		}
	}
	Joints.Reset();

	Controls.Reset();
	Activations.Reset();
	ActuatorForces.Reset();
	bTwinBuilt = false;
	bSubstepBound = false;
}

UPrimitiveComponent* UMjTwinComponent::CreateShapeComponent(const FMjTwinShape& Shape)
{
	UPrimitiveComponent* Component = nullptr;

	switch (Shape.Type)
	{
		case EMjTwinShapeType::Box:
		case EMjTwinShapeType::PlaneSlab:
		{
			UBoxComponent* Box = NewObject<UBoxComponent>(this, NAME_None, RF_Transient);
			Box->InitBoxExtent(FVector(
				FMath::Max(Shape.Size.X, 0.1),
				FMath::Max(Shape.Size.Y, 0.1),
				FMath::Max(Shape.Size.Z, 0.1)));
			Component = Box;
			break;
		}

		case EMjTwinShapeType::Sphere:
		{
			USphereComponent* Sphere = NewObject<USphereComponent>(this, NAME_None, RF_Transient);
			Sphere->InitSphereRadius(FMath::Max(Shape.Size.X, 0.1));
			Component = Sphere;
			break;
		}

		case EMjTwinShapeType::Capsule:
		{
			UCapsuleComponent* Capsule = NewObject<UCapsuleComponent>(this, NAME_None, RF_Transient);
			// Unreal's capsule half height covers the caps, MuJoCo's is the segment only.
			Capsule->InitCapsuleSize(FMath::Max(Shape.Size.X, 0.1), FMath::Max(Shape.Size.Y + Shape.Size.X, 0.1));
			Component = Capsule;
			break;
		}

		case EMjTwinShapeType::Cylinder:
		{
			UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this, NAME_None, RF_Transient);
			Mesh->AddCollisionConvexMesh(FMjTwinDescriptorBuilder::MakeCylinderPoints(Shape.Size.X, Shape.Size.Y));
			Component = Mesh;
			break;
		}

		case EMjTwinShapeType::Ellipsoid:
		{
			UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this, NAME_None, RF_Transient);
			Mesh->AddCollisionConvexMesh(FMjTwinDescriptorBuilder::MakeEllipsoidPoints(Shape.Size));
			Component = Mesh;
			break;
		}

		case EMjTwinShapeType::Mesh:
		{
			TArray<FVector> HullPoints;
			if (UStaticMesh* Asset = Shape.MeshAsset.LoadSynchronous())
			{
				if (const UBodySetup* BodySetup = Asset->GetBodySetup())
				{
					for (const FKConvexElem& Elem : BodySetup->AggGeom.ConvexElems)
					{
						for (const FVector& Vertex : Elem.VertexData)
						{
							HullPoints.Add(Vertex * Shape.MeshScale);
						}
					}
				}
			}

			if (HullPoints.Num() >= 4)
			{
				UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this, NAME_None, RF_Transient);
				Mesh->AddCollisionConvexMesh(HullPoints);
				Component = Mesh;
			}
			else
			{
				// No imported hull to copy: the bounding box is the closest known shape.
				UBoxComponent* Box = NewObject<UBoxComponent>(this, NAME_None, RF_Transient);
				Box->InitBoxExtent(FVector(
					FMath::Max(Shape.Size.X, 0.1),
					FMath::Max(Shape.Size.Y, 0.1),
					FMath::Max(Shape.Size.Z, 0.1)));
				Component = Box;
			}
			break;
		}

		default:
			return nullptr;
	}

	Component->SetCollisionProfileName(TwinCollisionProfile);
	Component->SetGenerateOverlapEvents(false);
	if (!Shape.bCollides)
	{
		Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	}
	return Component;
}

USceneComponent* UMjTwinComponent::CreateVisualComponent(const FMjTwinShape& Shape, USceneComponent* AttachTo)
{
	if (!AttachTo)
	{
		return nullptr;
	}

	if (UStaticMesh* Asset = Shape.MeshAsset.LoadSynchronous())
	{
		UStaticMeshComponent* Visual = NewObject<UStaticMeshComponent>(this, NAME_None, RF_Transient);
		Visual->SetupAttachment(AttachTo);
		Visual->SetMobility(EComponentMobility::Movable);
		Visual->SetStaticMesh(Asset);
		Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		Visual->RegisterComponent();
		Visual->SetRelativeTransform(Shape.LocalTransform, false, nullptr, ETeleportType::TeleportPhysics);
		Visual->SetRelativeScale3D(Shape.MeshScale);
		return Visual;
	}

	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	BuildShapeRenderData(Shape.Type, Shape.Size, Vertices, Triangles);
	if (Vertices.Num() == 0 || Triangles.Num() == 0)
	{
		return nullptr;
	}

	// Flat normals are not needed: the procedural mesh renders with the accumulated
	// face normals the component derives from the triangles.
	UProceduralMeshComponent* Visual = NewObject<UProceduralMeshComponent>(this, NAME_None, RF_Transient);
	Visual->SetupAttachment(AttachTo);
	Visual->SetMobility(EComponentMobility::Movable);
	Visual->RegisterComponent();
	Visual->SetRelativeTransform(Shape.LocalTransform, false, nullptr, ETeleportType::TeleportPhysics);
	Visual->bUseAsyncCooking = false;
	Visual->CreateMeshSection(0, Vertices, Triangles, TArray<FVector>(), TArray<FVector2D>(), TArray<FColor>(),
		TArray<FProcMeshTangent>(), false);
	return Visual;
}

void UMjTwinComponent::ApplyMassProperties()
{
	// MuJoCo treats a massless body as immovable; Chaos needs a positive mass, so a
	// massless body gets a minimal one (reported by the descriptor as approximate).
	constexpr double MinimumMassKg = 1.0e-3;

	for (int32 BodyIndex = 0; BodyIndex < Descriptor.Bodies.Num(); ++BodyIndex)
	{
		const FMjTwinBody& Body = Descriptor.Bodies[BodyIndex];
		UPrimitiveComponent* Root = Bodies.IsValidIndex(BodyIndex) ? Bodies[BodyIndex].PhysicsRoot : nullptr;
		if (!Root || !Root->IsSimulatingPhysics())
		{
			continue;
		}

		Root->SetMassOverrideInKg(NAME_None, FMath::Max(Body.MassKg, MinimumMassKg), true);

		FBodyInstance* Instance = Root->GetBodyInstance();
		if (!Instance)
		{
			continue;
		}

		// Chaos computes the tensor from the mirrored shapes; scaling it by the ratio to
		// the MuJoCo principal moments makes the diagonal inertia match the model exactly.
		const FVector Current = Instance->GetBodyInertiaTensor();
		const FVector Desired = Body.InertiaDiagonal;
		if (Current.GetMin() > SMALL_NUMBER && Desired.GetMin() > 0.0)
		{
			Root->SetInertiaTensorScale(Desired / Current, NAME_None);
		}
		else
		{
			UE_LOG(LogURLabRuntime, Warning,
				TEXT("[MjTwin] body '%s': Chaos reported a degenerate inertia tensor, the MuJoCo tensor could not be enforced."),
				*Body.Name);
		}
	}
}

void UMjTwinComponent::ApplyFriction()
{
	for (const FMjTwinShape& Shape : Descriptor.Shapes)
	{
		if (Shape.bUnsupported || Shape.BodyIndex == INDEX_NONE || !Bodies.IsValidIndex(Shape.BodyIndex))
		{
			continue;
		}

		for (UPrimitiveComponent* ShapeComponent : Bodies[Shape.BodyIndex].Shapes)
		{
			if (!ShapeComponent)
			{
				continue;
			}

			UPhysicalMaterial* Material = NewObject<UPhysicalMaterial>(ShapeComponent, NAME_None, RF_Transient);
			Material->Friction = FMath::Max(static_cast<float>(Shape.Friction.X), 0.0f);
			Material->Restitution = 0.0f;
			Material->bOverrideFrictionCombineMode = true;
			// MuJoCo combines geom friction with max (at equal priority).
			Material->FrictionCombineMode = EFrictionCombineMode::Max;
			ShapeComponent->SetPhysMaterialOverride(Material);
		}
	}
}

UPhysicsConstraintComponent* UMjTwinComponent::CreateJointConstraint(int32 JointIndex)
{
	if (!Descriptor.Joints.IsValidIndex(JointIndex))
	{
		return nullptr;
	}

	const FMjTwinJoint& Joint = Descriptor.Joints[JointIndex];
	if (Joint.Type == EMjTwinJointType::Free || Joint.ChildBodyIndex == INDEX_NONE || !Bodies.IsValidIndex(Joint.ChildBodyIndex))
	{
		// A free joint is simply an unconstrained body.
		return nullptr;
	}

	UPrimitiveComponent* Child = Bodies[Joint.ChildBodyIndex].PhysicsRoot;
	UPrimitiveComponent* Parent = Joint.ParentBodyIndex != INDEX_NONE && Bodies.IsValidIndex(Joint.ParentBodyIndex)
		? Bodies[Joint.ParentBodyIndex].PhysicsRoot
		: nullptr;
	if (!Child)
	{
		return nullptr;
	}

	UPhysicsConstraintComponent* Constraint = NewObject<UPhysicsConstraintComponent>(this, NAME_None, RF_Transient);
	Constraint->SetupAttachment(this);
	Constraint->SetMobility(EComponentMobility::Movable);
	Constraint->RegisterComponent();

	const FTransform Frame = GetBaseTransform() * FTransform(Joint.ConstraintWorldRot, Joint.ConstraintWorldPos, FVector::OneVector);
	Constraint->SetWorldTransform(Frame, false, nullptr, ETeleportType::TeleportPhysics);
	Constraint->SetConstrainedComponents(Parent, NAME_None, Child, NAME_None);
	Constraint->ConstraintInstance.SetDisableCollision(!Joint.bCollideWithParent);

	switch (Joint.Type)
	{
		case EMjTwinJointType::Hinge:
			Constraint->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Locked);
			Constraint->SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Locked);
			if (Joint.bLimited)
			{
				Constraint->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, static_cast<float>(Joint.HalfRange));
			}
			else
			{
				Constraint->SetAngularTwistMotion(EAngularConstraintMotion::ACM_Free);
			}
			break;

		case EMjTwinJointType::Slide:
			Constraint->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Locked);
			Constraint->SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Locked);
			Constraint->SetAngularTwistMotion(EAngularConstraintMotion::ACM_Locked);
			if (Joint.bLimited)
			{
				Constraint->SetLinearXLimit(ELinearConstraintMotion::LCM_Limited, static_cast<float>(Joint.HalfRange));
			}
			else
			{
				Constraint->SetLinearXLimit(ELinearConstraintMotion::LCM_Free, 0.0f);
			}
			break;

		case EMjTwinJointType::FreeSpin:
			Constraint->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			Constraint->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0.0f);
			if (Joint.bLimited)
			{
				Constraint->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Limited, static_cast<float>(Joint.SwingLimitDeg));
				Constraint->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Limited, static_cast<float>(Joint.SwingLimitDeg));
				Constraint->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Limited, static_cast<float>(Joint.TwistLimitDeg));
			}
			else
			{
				Constraint->SetAngularSwing1Motion(EAngularConstraintMotion::ACM_Free);
				Constraint->SetAngularSwing2Motion(EAngularConstraintMotion::ACM_Free);
				Constraint->SetAngularTwistMotion(EAngularConstraintMotion::ACM_Free);
			}
			break;

		default:
			break;
	}

	return Constraint;
}

UMjPhysicsEngine* UMjTwinComponent::FindPhysicsEngine() const
{
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			return Manager->PhysicsEngine;
		}
	}

	if (UWorld* World = GetWorld())
	{
		for (TActorIterator<AAMjManager> It(World); It; ++It)
		{
			if (It->PhysicsEngine)
			{
				return It->PhysicsEngine;
			}
		}
	}
	return nullptr;
}

void UMjTwinComponent::BeginPlay()
{
	Super::BeginPlay();
}

void UMjTwinComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	ClearTwin();
	Super::EndPlay(EndPlayReason);
}

void UMjTwinComponent::OnCalculateCustomPhysicsBody(float DeltaTime, FBodyInstance* BodyInstance)
{
	// The callback fires once per body per sub-step, so the forces are filtered by the
	// body that triggered it: every force (gravity correction, actuator, joint reaction)
	// then lands exactly once per sub-step.
	for (int32 BodyIndex = 0; BodyIndex < Bodies.Num(); ++BodyIndex)
	{
		UPrimitiveComponent* Root = Bodies[BodyIndex].PhysicsRoot;
		if (Root && Root->GetBodyInstance() == BodyInstance)
		{
			ApplyActuatorForces(DeltaTime, BodyIndex);
			return;
		}
	}
}

void UMjTwinComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bTwinBuilt)
	{
		return;
	}

	if (BuildSettings.bDriveFromMujoco)
	{
		ApplyMujocoState();
	}
	else if (BuildSettings.bSubstepActuation)
	{
		// Re-register every tick: a custom physics delegate is consumed by one step.
		for (const FMjTwinRuntimeBody& Body : Bodies)
		{
			if (Body.PhysicsRoot && Body.PhysicsRoot->IsSimulatingPhysics())
			{
				if (FBodyInstance* Instance = Body.PhysicsRoot->GetBodyInstance())
				{
					Instance->AddCustomPhysics(OnCalculateCustomPhysics);
				}
			}
		}
	}
	else
	{
		ApplyActuatorForces(DeltaTime);
	}

	if (bLogDivergence)
	{
		FMjTwinDivergence Divergence;
		if (ComputeDivergence(Divergence))
		{
			UE_LOG(LogURLabRuntime, Verbose, TEXT("[MjTwin] %s"), *Divergence.ToText());
		}
	}
}

double UMjTwinComponent::EvaluateActuator(int32 ActuatorIndex, double DeltaSeconds)
{
	if (!Descriptor.Actuators.IsValidIndex(ActuatorIndex))
	{
		return 0.0;
	}

	const FMjTwinActuator& Actuator = Descriptor.Actuators[ActuatorIndex];
	if (Actuator.bUnsupported)
	{
		ActuatorForces[ActuatorIndex] = 0.0;
		return 0.0;
	}

	// MuJoCo's actuator law is evaluated in MuJoCo units: radians and metres for the
	// joint coordinate, radians/s and m/s for its velocity.
	double Length = 0.0;
	double Velocity = 0.0;
	if (Actuator.JointIndex != INDEX_NONE)
	{
		GetJointStateMujocoUnits(Actuator.JointIndex, Length, Velocity);
	}

	double Control = Controls.IsValidIndex(ActuatorIndex) ? Controls[ActuatorIndex] : 0.0;
	if (bClampControls && Actuator.bCtrlLimited)
	{
		const double Low = FMath::Min(Actuator.CtrlMin, Actuator.CtrlMax);
		const double High = FMath::Max(Actuator.CtrlMin, Actuator.CtrlMax);
		Control = FMath::Clamp(Control, Low, High);
	}

	// Activation dynamics: the integrator type is integrated exactly, filter and
	// muscle types are approximated by the same first-order lag (reported as such).
	double Activation = Control;
	if (Actuator.bHasActivationDynamics)
	{
		const double TimeConstant = Actuator.DynPrm.Num() > 0 ? Actuator.DynPrm[0] : 0.0;
		if (TimeConstant > 0.0 && DeltaSeconds > 0.0)
		{
			Activations[ActuatorIndex] += (Control - Activations[ActuatorIndex]) * DeltaSeconds / TimeConstant;
		}
		else
		{
			Activations[ActuatorIndex] = Control;
		}
		Activation = Activations[ActuatorIndex];
	}

	double Gain = 0.0;
	switch (Actuator.GainType)
	{
		case static_cast<int32>(mjGAIN_FIXED):
			Gain = Actuator.GainPrm.Num() > 0 ? Actuator.GainPrm[0] : 0.0;
			break;

		case static_cast<int32>(mjGAIN_AFFINE):
			Gain = (Actuator.GainPrm.Num() > 0 ? Actuator.GainPrm[0] : 0.0)
				+ (Actuator.GainPrm.Num() > 1 ? Actuator.GainPrm[1] * Length : 0.0)
				+ (Actuator.GainPrm.Num() > 2 ? Actuator.GainPrm[2] * Velocity : 0.0);
			break;

		default:
			Gain = Actuator.GainPrm.Num() > 0 ? Actuator.GainPrm[0] : 0.0;
			break;
	}

	double Bias = 0.0;
	if (Actuator.BiasType == static_cast<int32>(mjBIAS_AFFINE))
	{
		Bias = (Actuator.BiasPrm.Num() > 0 ? Actuator.BiasPrm[0] : 0.0)
			+ (Actuator.BiasPrm.Num() > 1 ? Actuator.BiasPrm[1] * Length : 0.0)
			+ (Actuator.BiasPrm.Num() > 2 ? Actuator.BiasPrm[2] * Velocity : 0.0);
	}

	double Force = Gain * Activation + Bias;

	// Damper actuators (zero gain, no bias) apply -damping * velocity.
	if (!Actuator.bHasActivationDynamics && FMath::IsNearlyZero(Gain) && Actuator.BiasType == static_cast<int32>(mjBIAS_NONE))
	{
		Force += -Actuator.Damping * Velocity;
	}

	if (Actuator.bForceLimited)
	{
		const double Low = FMath::Min(Actuator.ForceMin, Actuator.ForceMax);
		const double High = FMath::Max(Actuator.ForceMin, Actuator.ForceMax);
		Force = FMath::Clamp(Force, Low, High);
	}

	ActuatorForces[ActuatorIndex] = Force;
	return Force;
}

void UMjTwinComponent::ApplyActuatorForces(double DeltaSeconds, int32 BodyIndexFilter)
{
	if (!bTwinBuilt)
	{
		return;
	}

	const FTransform Base = GetBaseTransform();
	const double WorldGravityZ = GetWorld() ? GetWorld()->GetGravityZ() : -980.0;
	const double DesiredGravityZ = BuildSettings.bApplyGravity ? Descriptor.Options.Gravity.Z : WorldGravityZ;

	auto GetRoot = [this](int32 BodyIndex) -> UPrimitiveComponent*
	{
		return Bodies.IsValidIndex(BodyIndex) ? Bodies[BodyIndex].PhysicsRoot : nullptr;
	};

	auto ApplyJointForce = [&](int32 ChildBodyIndex, int32 ParentBodyIndex, const FVector& Axis,
		double ScalarForce, bool bAngular)
	{
		const double ToUE = bAngular ? NewtonMeterToUE : NewtonToUE;
		if (UPrimitiveComponent* Child = GetRoot(ChildBodyIndex))
		{
			if (Child->IsSimulatingPhysics())
			{
				const FVector Wrench = Axis * (ScalarForce * ToUE);
				if (bAngular)
				{
					Child->AddTorqueInRadians(Wrench, NAME_None, false);
				}
				else
				{
					Child->AddForce(Wrench, NAME_None, false);
				}
			}
		}
		if (UPrimitiveComponent* Parent = GetRoot(ParentBodyIndex))
		{
			if (Parent->IsSimulatingPhysics())
			{
				// Newton's third law: the reaction goes to the body on the other side.
				const FVector Wrench = Axis * (-ScalarForce * ToUE);
				if (bAngular)
				{
					Parent->AddTorqueInRadians(Wrench, NAME_None, false);
				}
				else
				{
					Parent->AddForce(Wrench, NAME_None, false);
				}
			}
		}
	};

	// --- Gravity correction and compensation -------------------------------
	for (int32 BodyIndex = 0; BodyIndex < Descriptor.Bodies.Num(); ++BodyIndex)
	{
		if (BodyIndexFilter != INDEX_NONE && BodyIndex != BodyIndexFilter)
		{
			continue;
		}

		const FMjTwinBody& Body = Descriptor.Bodies[BodyIndex];
		UPrimitiveComponent* Root = GetRoot(BodyIndex);
		if (!Root || !Root->IsSimulatingPhysics())
		{
			continue;
		}

		// Unreal applies one world gravity to every body; the twin adds the difference
		// between the model's gravity and the world's, plus MuJoCo's gravity compensation.
		const double ExtraAcceleration = (WorldGravityZ - DesiredGravityZ) - Body.GravityCompensation * DesiredGravityZ;
		if (!FMath::IsNearlyZero(ExtraAcceleration))
		{
			Root->AddForce(FVector(0.0, 0.0, Body.MassKg * ExtraAcceleration), NAME_None, false);
		}
	}

	// --- Actuators ---------------------------------------------------------
	for (int32 ActuatorIndex = 0; ActuatorIndex < Descriptor.Actuators.Num(); ++ActuatorIndex)
	{
		const FMjTwinActuator& Actuator = Descriptor.Actuators[ActuatorIndex];
		if (Actuator.bUnsupported || Actuator.BodyIndex == INDEX_NONE)
		{
			continue;
		}
		if (BodyIndexFilter != INDEX_NONE && Actuator.BodyIndex != BodyIndexFilter)
		{
			continue;
		}

		const double LawForce = EvaluateActuator(ActuatorIndex, DeltaSeconds);
		const double Gear = FMath::IsNearlyZero(Actuator.Gear) ? 1.0 : Actuator.Gear;
		const FVector Axis = Base.GetRotation().RotateVector(Actuator.AxisWorld).GetSafeNormal();
		const int32 ParentBodyIndex = Descriptor.Joints.IsValidIndex(Actuator.JointIndex)
			? Descriptor.Joints[Actuator.JointIndex].ParentBodyIndex
			: INDEX_NONE;
		ApplyJointForce(Actuator.BodyIndex, ParentBodyIndex, Axis, LawForce * Gear, Actuator.bAngular);
	}

	// --- Joint springs, damping, and friction loss -------------------------
	for (int32 JointIndex = 0; JointIndex < Descriptor.Joints.Num(); ++JointIndex)
	{
		const FMjTwinJoint& Joint = Descriptor.Joints[JointIndex];
		if (Joint.ChildBodyIndex == INDEX_NONE
			|| (Joint.Type != EMjTwinJointType::Hinge && Joint.Type != EMjTwinJointType::Slide))
		{
			continue;
		}
		if (BodyIndexFilter != INDEX_NONE && Joint.ChildBodyIndex != BodyIndexFilter)
		{
			continue;
		}
		if (FMath::IsNearlyZero(Joint.Stiffness) && FMath::IsNearlyZero(Joint.Damping) && FMath::IsNearlyZero(Joint.FrictionLoss))
		{
			continue;
		}

		double Position = 0.0;
		double Velocity = 0.0;
		if (!GetJointStateMujocoUnits(JointIndex, Position, Velocity))
		{
			continue;
		}

		double SpringReference = Joint.SpringRef;
		if (Joint.Type == EMjTwinJointType::Hinge)
		{
			SpringReference = FMath::DegreesToRadians(SpringReference);
		}
		else
		{
			SpringReference /= MjTwinConv::MetersToCentimeters;
		}

		double Force = 0.0;
		if (Joint.bHasSpring)
		{
			Force += -Joint.Stiffness * (Position - SpringReference);
		}
		Force += -Joint.Damping * Velocity;
		if (!FMath::IsNearlyZero(Joint.FrictionLoss))
		{
			// Coulomb friction along the joint axis, with a velocity dead zone so a
			// resting joint does not chatter.
			constexpr double DeadZone = 1.0e-3;
			if (FMath::Abs(Velocity) > DeadZone)
			{
				Force += -Joint.FrictionLoss * FMath::Sign(Velocity);
			}
			else
			{
				Force += -Joint.FrictionLoss * FMath::Clamp(Velocity / DeadZone, -1.0, 1.0);
			}
		}

		if (!FMath::IsNearlyZero(Force))
		{
			const FVector Axis = Base.GetRotation().RotateVector(Joint.AxisWorld).GetSafeNormal();
			ApplyJointForce(Joint.ChildBodyIndex, Joint.ParentBodyIndex, Axis, Force, Joint.Type == EMjTwinJointType::Hinge);
		}
	}
}

bool UMjTwinComponent::GetJointStateMujocoUnits(int32 JointIndex, double& OutPosition, double& OutVelocity) const
{
	OutPosition = 0.0;
	OutVelocity = 0.0;

	if (!Descriptor.Joints.IsValidIndex(JointIndex))
	{
		return false;
	}

	const FMjTwinJoint& Joint = Descriptor.Joints[JointIndex];
	if (Joint.ChildBodyIndex == INDEX_NONE || !Bodies.IsValidIndex(Joint.ChildBodyIndex))
	{
		return false;
	}

	UPrimitiveComponent* Child = Bodies[Joint.ChildBodyIndex].PhysicsRoot;
	UPrimitiveComponent* Parent = Joint.ParentBodyIndex != INDEX_NONE && Bodies.IsValidIndex(Joint.ParentBodyIndex)
		? Bodies[Joint.ParentBodyIndex].PhysicsRoot
		: nullptr;
	if (!Child)
	{
		return false;
	}

	const FQuat ChildRotation = Child->GetComponentQuat();
	const FQuat ParentRotation = Parent ? Parent->GetComponentQuat() : FQuat::Identity;
	const FVector ChildPosition = Child->GetComponentLocation();
	const FVector ParentPosition = Parent ? Parent->GetComponentLocation() : FVector::ZeroVector;
	const FVector Axis = GetBaseTransform().GetRotation().RotateVector(Joint.AxisWorld).GetSafeNormal();

	if (Joint.Type == EMjTwinJointType::Hinge || Joint.Type == EMjTwinJointType::FreeSpin)
	{
		// Signed rotation about the joint axis. Quaternions double-cover rotations, so
		// the representative with a non-negative W gives the short-way angle.
		FQuat Relative = ParentRotation.Inverse() * ChildRotation;
		if (Relative.W < 0.0)
		{
			Relative = FQuat(-Relative.X, -Relative.Y, -Relative.Z, -Relative.W);
		}
		const double HalfAngle = FMath::Acos(FMath::Clamp(Relative.W, -1.0, 1.0));
		const double Sign = (Relative.X * Axis.X + Relative.Y * Axis.Y + Relative.Z * Axis.Z) >= 0.0 ? 1.0 : -1.0;
		OutPosition = Sign * 2.0 * HalfAngle;

		const FVector ChildAngular = Child->GetPhysicsAngularVelocityInRadians();
		const FVector ParentAngular = Parent ? Parent->GetPhysicsAngularVelocityInRadians() : FVector::ZeroVector;
		OutVelocity = (ChildAngular - ParentAngular).Dot(Axis);
		return true;
	}

	const double LinearCm = (ChildPosition - ParentPosition).Dot(Axis);
	OutPosition = LinearCm / MjTwinConv::MetersToCentimeters;

	const FVector ChildLinear = Child->GetPhysicsLinearVelocity();
	const FVector ParentLinear = Parent ? Parent->GetPhysicsLinearVelocity() : FVector::ZeroVector;
	OutVelocity = (ChildLinear - ParentLinear).Dot(Axis) / MjTwinConv::MetersToCentimeters;
	return true;
}

bool UMjTwinComponent::ApplyMujocoState()
{
	if (!bTwinBuilt)
	{
		return false;
	}

	UMjPhysicsEngine* Engine = FindPhysicsEngine();
	if (!Engine)
	{
		return false;
	}

	const FTransform Base = GetBaseTransform();
	bool bApplied = false;
	Engine->WithRenderState([&](const FMjRenderSnapshot& Snapshot)
	{
		for (int32 BodyIndex = 0; BodyIndex < Descriptor.Bodies.Num(); ++BodyIndex)
		{
			const int32 MjBodyId = Descriptor.Bodies[BodyIndex].MjBodyId;
			UPrimitiveComponent* Root = Bodies.IsValidIndex(BodyIndex) ? Bodies[BodyIndex].PhysicsRoot : nullptr;
			if (!Root || MjBodyId < 0
				|| !Snapshot.XPos.IsValidIndex(3 * MjBodyId + 2)
				|| !Snapshot.XQuat.IsValidIndex(4 * MjBodyId + 3))
			{
				continue;
			}

			const FTransform MjWorld(MjTwinConv::Rotation(&Snapshot.XQuat[4 * MjBodyId]),
				MjTwinConv::Position(&Snapshot.XPos[3 * MjBodyId]), FVector::OneVector);
			const FTransform Target = Base * (MjWorld * Bodies[BodyIndex].PhysicsFrameInMjBody);
			Root->SetWorldTransform(Target, false, nullptr, ETeleportType::TeleportPhysics);
			bApplied = true;
		}
	});
	return bApplied;
}

bool UMjTwinComponent::ComputeDivergence(FMjTwinDivergence& Out)
{
	Out = FMjTwinDivergence();
	if (!bTwinBuilt)
	{
		return false;
	}

	UMjPhysicsEngine* Engine = FindPhysicsEngine();
	if (!Engine)
	{
		return false;
	}

	const FTransform Base = GetBaseTransform();
	Engine->WithRenderState([&](const FMjRenderSnapshot& Snapshot)
	{
		Out.SimTime = static_cast<float>(Snapshot.SimTime);
		Out.bValid = true;

		for (int32 BodyIndex = 0; BodyIndex < Descriptor.Bodies.Num(); ++BodyIndex)
		{
			const int32 MjBodyId = Descriptor.Bodies[BodyIndex].MjBodyId;
			UPrimitiveComponent* Root = Bodies.IsValidIndex(BodyIndex) ? Bodies[BodyIndex].PhysicsRoot : nullptr;
			if (!Root || MjBodyId < 0
				|| !Snapshot.XPos.IsValidIndex(3 * MjBodyId + 2)
				|| !Snapshot.XQuat.IsValidIndex(4 * MjBodyId + 3))
			{
				continue;
			}

			const FTransform MjWorld(MjTwinConv::Rotation(&Snapshot.XQuat[4 * MjBodyId]),
				MjTwinConv::Position(&Snapshot.XPos[3 * MjBodyId]), FVector::OneVector);
			const FTransform Expected = Base * (MjWorld * Bodies[BodyIndex].PhysicsFrameInMjBody);
			const FTransform Actual = Root->GetComponentTransform();

			FMjTwinBodyDivergence Entry;
			Entry.Body = Descriptor.Bodies[BodyIndex].Name;
			Entry.LinearErrorCm = static_cast<float>((Actual.GetLocation() - Expected.GetLocation()).Size());
			Entry.AngularErrorDeg = static_cast<float>(FMath::RadiansToDegrees(
				Actual.GetRotation().AngularDistance(Expected.GetRotation())));
			Out.MaxLinearErrorCm = FMath::Max(Out.MaxLinearErrorCm, Entry.LinearErrorCm);
			Out.MaxAngularErrorDeg = FMath::Max(Out.MaxAngularErrorDeg, Entry.AngularErrorDeg);
			Out.Bodies.Add(MoveTemp(Entry));
		}

		Out.Bodies.Sort([](const FMjTwinBodyDivergence& A, const FMjTwinBodyDivergence& B)
		{
			return A.LinearErrorCm > B.LinearErrorCm;
		});
	});

	return Out.bValid;
}

UPrimitiveComponent* UMjTwinComponent::GetBodyComponent(int32 BodyIndex) const
{
	return Bodies.IsValidIndex(BodyIndex) ? Bodies[BodyIndex].PhysicsRoot : nullptr;
}

bool UMjTwinComponent::GetJointState(int32 JointIndex, double& OutPosition, double& OutVelocity) const
{
	if (!GetJointStateMujocoUnits(JointIndex, OutPosition, OutVelocity))
	{
		return false;
	}

	// Public accessors report the twin's own units: degrees for rotation, cm for slide.
	if (Descriptor.Joints.IsValidIndex(JointIndex) && Descriptor.Joints[JointIndex].Type == EMjTwinJointType::Hinge)
	{
		OutPosition = FMath::RadiansToDegrees(OutPosition);
		OutVelocity = FMath::RadiansToDegrees(OutVelocity);
	}
	else
	{
		OutPosition *= MjTwinConv::MetersToCentimeters;
		OutVelocity *= MjTwinConv::MetersToCentimeters;
	}
	return true;
}

void UMjTwinComponent::SetControls(const TArray<double>& InControls)
{
	for (int32 Index = 0; Index < Controls.Num(); ++Index)
	{
		Controls[Index] = InControls.IsValidIndex(Index) ? InControls[Index] : 0.0;
	}
}

void UMjTwinComponent::SetControl(int32 ActuatorIndex, double Value)
{
	if (Controls.IsValidIndex(ActuatorIndex))
	{
		Controls[ActuatorIndex] = Value;
	}
}

bool UMjTwinComponent::SetControlByName(FName ActuatorName, double Value)
{
	const int32 Index = Descriptor.FindActuatorByName(ActuatorName.ToString());
	if (Index == INDEX_NONE)
	{
		return false;
	}
	SetControl(Index, Value);
	return true;
}

void UMjTwinComponent::ResetControls()
{
	for (int32 Index = 0; Index < Controls.Num(); ++Index)
	{
		Controls[Index] = 0.0;
		Activations[Index] = 0.0;
		ActuatorForces[Index] = 0.0;
	}
}

double UMjTwinComponent::GetControl(int32 ActuatorIndex) const
{
	return Controls.IsValidIndex(ActuatorIndex) ? Controls[ActuatorIndex] : 0.0;
}

double UMjTwinComponent::GetActivation(int32 ActuatorIndex) const
{
	return Activations.IsValidIndex(ActuatorIndex) ? Activations[ActuatorIndex] : 0.0;
}

double UMjTwinComponent::GetActuatorForce(int32 ActuatorIndex) const
{
	return ActuatorForces.IsValidIndex(ActuatorIndex) ? ActuatorForces[ActuatorIndex] : 0.0;
}
