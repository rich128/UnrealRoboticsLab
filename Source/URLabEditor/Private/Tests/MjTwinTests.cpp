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

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"

#include "MjTestHelpers.h"
#include "MuJoCo/Components/Actuators/MjGeneralActuator.h"
#include "MuJoCo/Components/Actuators/MjMotorActuator.h"
#include "MuJoCo/Components/Actuators/MjPositionActuator.h"
#include "MuJoCo/Components/Actuators/MjVelocityActuator.h"
#include "MuJoCo/Twin/MjTwinDescriptor.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "MuJoCo/Twin/MjTwinTypes.h"

namespace
{
/** Builds a twin descriptor from a compiled raw model, failing the test on error. */
bool BuildTwinFromRaw(FAutomationTestBase& Test, FMjTestSession& Session, FMjTwinDescriptor& Out)
{
	FString Error;
	const bool bBuilt = FMjTwinDescriptorBuilder::BuildFromModel(Session.m, Out, Error);
	Test.TestTrue(FString::Printf(TEXT("descriptor built (%s)"), *Error), bBuilt);
	return bBuilt;
}

/** True when two doubles agree within a tolerance. */
bool NearlyEqual(double A, double B, double Tolerance = 1e-4)
{
	return FMath::Abs(A - B) <= Tolerance;
}
} // namespace

/**
 * The twin built from the plugin's own component pipeline must mirror the compiled
 * model: one body, its box geom in centimetres, the hinge limits in place, and the
 * motor actuator classified with its gear.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTwinFromArticulationTest, "URLab.Twin.BuildsFromArticulation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjTwinFromArticulationTest::RunTest(const FString& Parameters)
{
	FMjUESession Session;
	const bool bInitialised = Session.Init([](FMjUESession& S)
	{
		S.Joint->bOverride_range = true;
		S.Joint->range = {-30.0f, 90.0f};
		S.Joint->bOverride_limited = true;
		S.Joint->limited = true;

		UMjMotorActuator* Motor = NewObject<UMjMotorActuator>(S.Robot, TEXT("Motor"));
		Motor->TargetName = TEXT("TestJoint");
		Motor->bOverride_gear = true;
		Motor->gear = {2.0f};
		Motor->RegisterComponent();
		Motor->AttachToComponent(S.Joint, FAttachmentTransformRules::KeepRelativeTransform);
	});

	if (!bInitialised)
	{
		AddError(Session.LastError);
		return false;
	}

	mjModel* Model = Session.Manager->PhysicsEngine->GetModel();
	TestNotNull(TEXT("compiled model"), Model);
	if (!Model)
	{
		Session.Cleanup();
		return false;
	}

	FMjTwinDescriptor Descriptor;
	if (!BuildTwinFromRaw(*this, Session, Descriptor))
	{
		Session.Cleanup();
		return false;
	}

	TestEqual(TEXT("twin body count"), Descriptor.Bodies.Num(), 1);
	TestEqual(TEXT("twin shape count"), Descriptor.Shapes.Num(), 1);
	TestEqual(TEXT("twin joint count"), Descriptor.Joints.Num(), 1);
	TestEqual(TEXT("twin actuator count"), Descriptor.Actuators.Num(), 1);

	if (Descriptor.Bodies.Num() == 1)
	{
		TestTrue(TEXT("body is simulated"), Descriptor.Bodies[0].bSimulated);
		TestTrue(TEXT("body has mass"), Descriptor.Bodies[0].MassKg > 0.0);
		TestTrue(TEXT("inertia is positive"), Descriptor.Bodies[0].InertiaDiagonal.GetMin() > 0.0);
	}

	if (Descriptor.Shapes.Num() == 1)
	{
		TestTrue(TEXT("geom mapped to a box"), Descriptor.Shapes[0].Type == EMjTwinShapeType::Box);
		TestTrue(TEXT("box half extents in cm"), NearlyEqual(Descriptor.Shapes[0].Size.X, 10.0)
			&& NearlyEqual(Descriptor.Shapes[0].Size.Y, 10.0) && NearlyEqual(Descriptor.Shapes[0].Size.Z, 10.0));
	}

	if (Descriptor.Joints.Num() == 1)
	{
		const FMjTwinJoint& Joint = Descriptor.Joints[0];
		TestTrue(TEXT("hinge joint"), Joint.Type == EMjTwinJointType::Hinge);
		TestTrue(TEXT("joint limited"), Joint.bLimited);
		TestTrue(TEXT("range min"), NearlyEqual(Joint.RangeMin, -30.0));
		TestTrue(TEXT("range max"), NearlyEqual(Joint.RangeMax, 90.0));
		TestTrue(TEXT("limit offset is the range midpoint"), NearlyEqual(Joint.LimitOffset, 30.0));
		TestTrue(TEXT("half range"), NearlyEqual(Joint.HalfRange, 60.0));
		TestTrue(TEXT("constraint frame carries the midpoint rotation"),
			NearlyEqual(FMath::RadiansToDegrees(Joint.ConstraintWorldRot.GetAngle()), 30.0, 1e-2));
	}

	if (Descriptor.Actuators.Num() == 1)
	{
		TestTrue(TEXT("motor classified as motor"), Descriptor.Actuators[0].Model == EMjTwinActuatorModel::Motor);
		TestTrue(TEXT("gear copied"), NearlyEqual(Descriptor.Actuators[0].Gear, 2.0));
		TestTrue(TEXT("actuator bound to the joint"), Descriptor.Actuators[0].JointIndex == 0);
	}

	TestFalse(TEXT("no unsupported elements"), Descriptor.Report.HasUnsupported());
	TestTrue(TEXT("report mentions the model"), Descriptor.Report.ToJson().Contains(TEXT("\"twin_bodies\"")));

	Session.Cleanup();
	return true;
}

/**
 * Bodies without a joint are welded to the world and must be mirrored as static
 * collision geometry, not dropped; a capsule keeps its radius and half segment.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTwinStaticWeldGroupTest, "URLab.Twin.StaticWeldGroupAndCapsule",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjTwinStaticWeldGroupTest::RunTest(const FString& Parameters)
{
	FMjTestSession Session;
	const bool bCompiled = Session.CompileXml(TEXT(R"(
<mujoco>
  <worldbody>
    <body name="table" pos="0 0 0.5">
      <geom name="top" type="box" size="0.4 0.4 0.05"/>
    </body>
    <body name="arm" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1" range="-45 45"/>
      <geom name="link" type="capsule" fromto="0 0 -0.2 0 0 0.2" size="0.03"/>
    </body>
  </worldbody>
</mujoco>
)"));
	TestTrue(TEXT("model compiled"), bCompiled);
	if (!bCompiled)
	{
		AddError(Session.LastError);
		return false;
	}

	FMjTwinDescriptor Descriptor;
	if (!BuildTwinFromRaw(*this, Session, Descriptor))
	{
		return false;
	}

	TestEqual(TEXT("two twin bodies (static table + arm)"), Descriptor.Bodies.Num(), 2);
	TestEqual(TEXT("two shapes"), Descriptor.Shapes.Num(), 2);

	const int32 TableIndex = Descriptor.FindBodyByName(TEXT("table"));
	const int32 ArmIndex = Descriptor.FindBodyByName(TEXT("arm"));
	TestTrue(TEXT("static body found"), TableIndex != INDEX_NONE);
	TestTrue(TEXT("articulated body found"), ArmIndex != INDEX_NONE);

	if (TableIndex != INDEX_NONE)
	{
		TestFalse(TEXT("welded-to-world body is static"), Descriptor.Bodies[TableIndex].bSimulated);
	}
	if (ArmIndex != INDEX_NONE)
	{
		TestTrue(TEXT("jointed body is simulated"), Descriptor.Bodies[ArmIndex].bSimulated);
	}

	for (const FMjTwinShape& Shape : Descriptor.Shapes)
	{
		TestTrue(TEXT("no unsupported shapes"), !Shape.bUnsupported);
		if (Shape.Name == TEXT("link"))
		{
			TestTrue(TEXT("capsule mapped"), Shape.Type == EMjTwinShapeType::Capsule);
			TestTrue(TEXT("capsule radius in cm"), NearlyEqual(Shape.Size.X, 3.0));
			TestTrue(TEXT("capsule half segment in cm"), NearlyEqual(Shape.Size.Y, 20.0));
		}
	}

	if (Descriptor.Joints.Num() == 1)
	{
		// A symmetric range keeps the constraint frame on the joint axis.
		TestTrue(TEXT("symmetric range has no offset"), NearlyEqual(Descriptor.Joints[0].LimitOffset, 0.0));
		TestTrue(TEXT("half range in degrees"), NearlyEqual(Descriptor.Joints[0].HalfRange, 45.0));
	}
	else
	{
		AddError(TEXT("expected exactly one joint"));
	}

	TestTrue(TEXT("exact notes for the welded group"), Descriptor.Report.CountOf(EMjTwinFidelity::Exact) > 0);
	return true;
}

/**
 * Every primitive geom gets the closest Chaos shape, and an infinite MuJoCo plane
 * becomes a finite slab that cannot be missed by fast bodies.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTwinShapeMappingTest, "URLab.Twin.ShapeMapping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjTwinShapeMappingTest::RunTest(const FString& Parameters)
{
	FMjTestSession Session;
	const bool bCompiled = Session.CompileXml(TEXT(R"(
<mujoco>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1"/>
    <body name="b" pos="0 0 1">
      <freejoint/>
      <geom name="s" type="sphere" size="0.05"/>
      <geom name="c" type="cylinder" size="0.04 0.1"/>
      <geom name="e" type="ellipsoid" size="0.1 0.05 0.02"/>
      <geom name="bx" type="box" size="0.03 0.04 0.05"/>
    </body>
  </worldbody>
</mujoco>
)"));
	TestTrue(TEXT("model compiled"), bCompiled);
	if (!bCompiled)
	{
		AddError(Session.LastError);
		return false;
	}

	FMjTwinDescriptor Descriptor;
	if (!BuildTwinFromRaw(*this, Session, Descriptor))
	{
		return false;
	}

	for (const FMjTwinShape& Shape : Descriptor.Shapes)
	{
		if (Shape.Name == TEXT("floor"))
		{
			TestTrue(TEXT("plane mapped to a slab"), Shape.Type == EMjTwinShapeType::PlaneSlab);
			TestTrue(TEXT("slab half extent"), NearlyEqual(Shape.Size.X, 5000.0));
		}
		else if (Shape.Name == TEXT("s"))
		{
			TestTrue(TEXT("sphere mapped"), Shape.Type == EMjTwinShapeType::Sphere);
			TestTrue(TEXT("sphere radius in cm"), NearlyEqual(Shape.Size.X, 5.0));
		}
		else if (Shape.Name == TEXT("c"))
		{
			TestTrue(TEXT("cylinder tessellated"), Shape.Type == EMjTwinShapeType::Cylinder);
			TestTrue(TEXT("cylinder radius and half length"), NearlyEqual(Shape.Size.X, 4.0) && NearlyEqual(Shape.Size.Y, 10.0));
		}
		else if (Shape.Name == TEXT("e"))
		{
			TestTrue(TEXT("ellipsoid tessellated"), Shape.Type == EMjTwinShapeType::Ellipsoid);
			TestTrue(TEXT("ellipsoid radii in cm"), NearlyEqual(Shape.Size.X, 10.0) && NearlyEqual(Shape.Size.Y, 5.0)
				&& NearlyEqual(Shape.Size.Z, 2.0));
		}
		else if (Shape.Name == TEXT("bx"))
		{
			TestTrue(TEXT("box mapped"), Shape.Type == EMjTwinShapeType::Box);
		}
	}

	TestTrue(TEXT("plane is reported as approximate"), Descriptor.Report.CountOf(EMjTwinFidelity::Approximate) > 0);

	// A free joint leaves the body unconstrained.
	const int32 BodyIndex = Descriptor.FindBodyByName(TEXT("b"));
	TestTrue(TEXT("free body found"), BodyIndex != INDEX_NONE);
	if (BodyIndex != INDEX_NONE)
	{
		TestTrue(TEXT("free body is simulated"), Descriptor.Bodies[BodyIndex].bSimulated);
	}
	if (Descriptor.Joints.Num() == 1)
	{
		TestTrue(TEXT("free joint detected"), Descriptor.Joints[0].Type == EMjTwinJointType::Free);
	}
	return true;
}

/**
 * Actuator classification follows the MJCF shortcut the model used, and anything the
 * twin cannot reproduce is reported instead of being dropped quietly.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTwinActuatorClassificationTest, "URLab.Twin.ActuatorClassification",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjTwinActuatorClassificationTest::RunTest(const FString& Parameters)
{
	FMjTestSession Session;
	const bool bCompiled = Session.CompileXml(TEXT(R"(
<mujoco>
  <worldbody>
    <body name="b1" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 0 1" range="-30 30"/>
      <geom name="g1" type="box" size="0.05 0.05 0.05"/>
    </body>
    <body name="b2" pos="0 0 2">
      <joint name="ball" type="ball"/>
      <geom name="g2" type="sphere" size="0.05"/>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t1"><joint joint="hinge" coef="1"/></fixed>
  </tendon>
  <actuator>
    <motor name="motor1" joint="hinge" gear="2"/>
    <position name="pos1" joint="hinge" kp="100"/>
    <velocity name="vel1" joint="hinge" kv="10"/>
    <general name="affine1" joint="hinge" gaintype="affine" gainprm="3 1 0"/>
    <motor name="ball_act" joint="ball"/>
    <general name="tendon_act" tendon="t1"/>
  </actuator>
</mujoco>
)"));
	TestTrue(TEXT("model compiled"), bCompiled);
	if (!bCompiled)
	{
		AddError(Session.LastError);
		return false;
	}

	FMjTwinDescriptor Descriptor;
	if (!BuildTwinFromRaw(*this, Session, Descriptor))
	{
		return false;
	}

	TestEqual(TEXT("six actuators in the model"), Descriptor.Actuators.Num(), 6);
	if (Descriptor.Actuators.Num() != 6)
	{
		return false;
	}

	TestTrue(TEXT("motor -> Motor"), Descriptor.Actuators[0].Model == EMjTwinActuatorModel::Motor);
	TestTrue(TEXT("position -> Position"), Descriptor.Actuators[1].Model == EMjTwinActuatorModel::Position);
	TestTrue(TEXT("velocity -> Velocity"), Descriptor.Actuators[2].Model == EMjTwinActuatorModel::Velocity);
	TestTrue(TEXT("affine gain -> GeneralAffine"), Descriptor.Actuators[3].Model == EMjTwinActuatorModel::GeneralAffine);
	TestTrue(TEXT("ball joint actuator unsupported"), Descriptor.Actuators[4].bUnsupported);
	TestTrue(TEXT("tendon actuator unsupported"), Descriptor.Actuators[5].bUnsupported);

	TestTrue(TEXT("report flags unsupported elements"), Descriptor.Report.HasUnsupported());
	TestTrue(TEXT("report counts mirrored actuators"), Descriptor.Report.NumTwinActuators == 4);
	TestTrue(TEXT("ball joint mapped to free spin"),
		Descriptor.FindJointByName(TEXT("ball")) != INDEX_NONE
			&& Descriptor.Joints[Descriptor.FindJointByName(TEXT("ball"))].Type == EMjTwinJointType::FreeSpin);

	// The report must explain every dropped capability in plain language.
	const FString ReportText = Descriptor.Report.ToText();
	TestTrue(TEXT("report names the tendon actuator"), ReportText.Contains(TEXT("tendon_act")));
	TestTrue(TEXT("report names the ball actuator"), ReportText.Contains(TEXT("ball_act")));
	return true;
}

/**
 * The twin's tensor helpers: the diagonalisation has to return the principal moments
 * in descending order with a right-handed orthonormal frame.
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjTwinTensorTest, "URLab.Twin.InertiaDiagonalisation",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

bool FMjTwinTensorTest::RunTest(const FString& Parameters)
{
	// A tensor with a known answer: rotating the diagonal (1, 2, 3) about Z by 30 degrees.
	{
		FMjTwinTensor Tensor;
		Tensor.M[0][0] = 0.5 * 1.0 + 0.5 * 2.0;
		Tensor.M[1][1] = 0.5 * 1.0 + 0.5 * 2.0;
		Tensor.M[2][2] = 3.0;
		Tensor.M[0][1] = Tensor.M[1][0] = 0.5 * (2.0 - 1.0) * 0.5;
		Tensor.M[0][2] = Tensor.M[2][0] = 0.0;
		Tensor.M[1][2] = Tensor.M[2][1] = 0.0;

		FQuat Axes = FQuat::Identity;
		FVector Moments = FVector::ZeroVector;
		TestTrue(TEXT("diagonalisation succeeds"), FMjTwinDescriptorBuilder::DiagonalizeInertia(Tensor, Axes, Moments));
		TestTrue(TEXT("largest moment first"), Moments.X >= Moments.Y && Moments.Y >= Moments.Z);
		TestTrue(TEXT("moments are the eigenvalues"), NearlyEqual(Moments.X, 3.0, 1e-6)
			&& NearlyEqual(Moments.Y, 2.0, 1e-6) && NearlyEqual(Moments.Z, 1.0, 1e-6));
		TestTrue(TEXT("axes are a rotation"), NearlyEqual(Axes.Size(), 1.0, 1e-6));

		// R * diag * R^T has to reproduce the input tensor.
		const FMatrix Rotation = Axes.ToMatrix();
		const FMatrix Reconstructed = Rotation * FMatrix(FVector(Moments.X, 0, 0), FVector(0, Moments.Y, 0), FVector(0, 0, Moments.Z), FVector::ZeroVector) * Rotation.GetTransposed();
		TestTrue(TEXT("tensor is reproduced"), NearlyEqual(Reconstructed.M[0][0], Tensor.M[0][0], 1e-5)
			&& NearlyEqual(Reconstructed.M[0][1], Tensor.M[0][1], 1e-5)
			&& NearlyEqual(Reconstructed.M[2][2], Tensor.M[2][2], 1e-5));
	}

	// The MuJoCo -> Unreal conventions the twin relies on.
	{
		const double Position[3] = {1.0, 2.0, 3.0};
		const FVector Converted = MjTwinConv::Position(Position);
		TestTrue(TEXT("position mirrors Y and scales to cm"),
			NearlyEqual(Converted.X, 100.0) && NearlyEqual(Converted.Y, -200.0) && NearlyEqual(Converted.Z, 300.0));

		const FVector DirectionVector = MjTwinConv::Direction(Position);
		TestTrue(TEXT("direction mirrors Y only"),
			NearlyEqual(DirectionVector.X, 1.0) && NearlyEqual(DirectionVector.Y, -2.0) && NearlyEqual(DirectionVector.Z, 3.0));

		const FVector AxialVector = MjTwinConv::Axial(Position);
		TestTrue(TEXT("axial vectors flip X and Z"),
			NearlyEqual(AxialVector.X, -1.0) && NearlyEqual(AxialVector.Y, 2.0) && NearlyEqual(AxialVector.Z, -3.0));

		const double Quaternion[4] = {0.70710678, 0.70710678, 0.0, 0.0};
		const FQuat Rotated = MjTwinConv::Rotation(Quaternion);
		const double Vector[3] = {0.0, 1.0, 0.0};
		double RotatedVector[3];
		MjTwinConv::RotateVec(Quaternion, Vector, RotatedVector);
		TestTrue(TEXT("rotation keeps MuJoCo's handedness"),
			NearlyEqual(RotatedVector[0], 0.0, 1e-6) && NearlyEqual(RotatedVector[1], 0.0, 1e-6)
			&& NearlyEqual(RotatedVector[2], 1.0, 1e-6));

		// A positive MuJoCo rotation is a positive Unreal rotation about the axial axis.
		TestTrue(TEXT("quaternion mirrored like MjUtils"), Rotated.Equals(MjUtils::MjToUERotation(Quaternion), 1e-6));
	}

	return true;
}
