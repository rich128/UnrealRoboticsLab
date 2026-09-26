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

#include "MuJoCo/Twin/MjTwinDescriptor.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "EngineUtils.h"
#include "MuJoCo/Components/Geometry/MjGeom.h"
#include "MuJoCo/Core/AMjManager.h"
#include "MuJoCo/Core/MjArticulation.h"
#include "MuJoCo/Core/MjPhysicsEngine.h"
#include "MuJoCo/Utils/MjUtils.h"
#include "Utils/URLabLogging.h"

namespace
{
/** Plane geoms are infinite half-spaces in MuJoCo; the twin uses a finite slab. */
constexpr double PlaneSlabHalfExtentCm = 5000.0;
constexpr double PlaneSlabHalfThicknessCm = 100.0;

/** Inertia below this (kg cm^2) is treated as degenerate: the body cannot spin. */
constexpr double DegenerateInertiaEpsilon = 1e-9;

/** Two axes closer than this are considered parallel when folding in armature. */
constexpr double AxisAlignmentTolerance = 1e-4;

/** A group of MuJoCo bodies that form one Chaos rigid body. */
struct FWeldGroup
{
	/** Index into FMjTwinDescriptor::Bodies, filled in when the body is created. */
	int32 TwinBodyIndex = INDEX_NONE;

	/** MuJoCo body ids welded together, group root first. */
	TArray<int32> MjBodyIds;
};

/** Mass properties of a single MuJoCo body, in mirrored world axes and Unreal units. */
struct FMassProps
{
	double Mass = 0.0;
	double Com[3] = {0.0, 0.0, 0.0};
	double Inertia[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
};

double Det3(const double M[3][3])
{
	return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
		- M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
		+ M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

/** Rotation matrix (columns are the rotated basis vectors) from a MuJoCo quaternion. */
void QuatToMatrix(const double* Q, double Out[3][3])
{
	const double W = Q[0], X = Q[1], Y = Q[2], Z = Q[3];
	Out[0][0] = 1.0 - 2.0 * (Y * Y + Z * Z);
	Out[0][1] = 2.0 * (X * Y - W * Z);
	Out[0][2] = 2.0 * (X * Z + W * Y);

	Out[1][0] = 2.0 * (X * Y + W * Z);
	Out[1][1] = 1.0 - 2.0 * (X * X + Z * Z);
	Out[1][2] = 2.0 * (Y * Z - W * X);

	Out[2][0] = 2.0 * (X * Z - W * Y);
	Out[2][1] = 2.0 * (Y * Z + W * X);
	Out[2][2] = 1.0 - 2.0 * (X * X + Y * Y);
}

/**
 * Jacobi eigenvalue iteration for a symmetric 3x3 matrix.
 * Returns the eigenvalues (descending) and the eigenvectors as the columns of OutVectors.
 */
void JacobiEigen(const double In[3][3], double OutValues[3], double OutVectors[3][3])
{
	double A[3][3];
	double V[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
	for (int I = 0; I < 3; ++I)
	{
		for (int J = 0; J < 3; ++J)
		{
			A[I][J] = In[I][J];
		}
	}

	// A symmetric 3x3 matrix converges in a handful of sweeps; 32 is generous.
	for (int Sweep = 0; Sweep < 32; ++Sweep)
	{
		const double OffDiagonal = FMath::Abs(A[0][1]) + FMath::Abs(A[0][2]) + FMath::Abs(A[1][2]);
		if (OffDiagonal < 1e-15)
		{
			break;
		}

		for (int P = 0; P < 2; ++P)
		{
			for (int Q = P + 1; Q < 3; ++Q)
			{
				if (FMath::Abs(A[P][Q]) < 1e-18)
				{
					continue;
				}

				const double Theta = (A[Q][Q] - A[P][P]) / (2.0 * A[P][Q]);
				const double Sign = Theta >= 0.0 ? 1.0 : -1.0;
				const double T = Sign / (FMath::Abs(Theta) + FMath::Sqrt(Theta * Theta + 1.0));
				const double C = 1.0 / FMath::Sqrt(T * T + 1.0);
				const double S = T * C;

				for (int K = 0; K < 3; ++K)
				{
					const double AKP = A[K][P];
					const double AKQ = A[K][Q];
					A[K][P] = C * AKP - S * AKQ;
					A[K][Q] = S * AKP + C * AKQ;
				}
				for (int K = 0; K < 3; ++K)
				{
					const double APK = A[P][K];
					const double AQK = A[Q][K];
					A[P][K] = C * APK - S * AQK;
					A[Q][K] = S * APK + C * AQK;
				}
				for (int K = 0; K < 3; ++K)
				{
					const double VKP = V[K][P];
					const double VKQ = V[K][Q];
					V[K][P] = C * VKP - S * VKQ;
					V[K][Q] = S * VKP + C * VKQ;
				}
			}
		}
	}

	for (int I = 0; I < 3; ++I)
	{
		OutValues[I] = A[I][I];
		for (int J = 0; J < 3; ++J)
		{
			OutVectors[I][J] = V[I][J];
		}
	}

	// Sort descending so the resulting frame is deterministic, keeping columns in sync.
	for (int I = 0; I < 2; ++I)
	{
		int Max = I;
		for (int J = I + 1; J < 3; ++J)
		{
			if (OutValues[J] > OutValues[Max])
			{
				Max = J;
			}
		}
		if (Max != I)
		{
			const double SwapValue = OutValues[I];
			OutValues[I] = OutValues[Max];
			OutValues[Max] = SwapValue;
			for (int K = 0; K < 3; ++K)
			{
				const double SwapVector = OutVectors[K][I];
				OutVectors[K][I] = OutVectors[K][Max];
				OutVectors[K][Max] = SwapVector;
			}
		}
	}
}

/** Converts an orthonormal rotation matrix into an Unreal quaternion. */
FQuat RotationMatrixToQuat(const double R[3][3])
{
	const double Trace = R[0][0] + R[1][1] + R[2][2];
	double Q[4] = {0.0, 0.0, 0.0, 1.0};

	if (Trace > 0.0)
	{
		const double S = FMath::Sqrt(Trace + 1.0) * 2.0;
		Q[3] = 0.25 * S;
		Q[0] = (R[2][1] - R[1][2]) / S;
		Q[1] = (R[0][2] - R[2][0]) / S;
		Q[2] = (R[1][0] - R[0][1]) / S;
	}
	else if (R[0][0] > R[1][1] && R[0][0] > R[2][2])
	{
		const double S = FMath::Sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
		Q[3] = (R[2][1] - R[1][2]) / S;
		Q[0] = 0.25 * S;
		Q[1] = (R[0][1] + R[1][0]) / S;
		Q[2] = (R[0][2] + R[2][0]) / S;
	}
	else if (R[1][1] > R[2][2])
	{
		const double S = FMath::Sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
		Q[3] = (R[0][2] - R[2][0]) / S;
		Q[0] = (R[0][1] + R[1][0]) / S;
		Q[1] = 0.25 * S;
		Q[2] = (R[1][2] + R[2][1]) / S;
	}
	else
	{
		const double S = FMath::Sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
		Q[3] = (R[1][0] - R[0][1]) / S;
		Q[0] = (R[0][2] + R[2][0]) / S;
		Q[1] = (R[1][2] + R[2][1]) / S;
		Q[2] = 0.25 * S;
	}

	FQuat Result(Q[0], Q[1], Q[2], Q[3]);
	Result.Normalize();
	return Result;
}

/** Unreal relative transform of ChildWorld expressed in ParentWorld. */
FTransform MakeRelativeTransform(const FTransform& ChildWorld, const FTransform& ParentWorld)
{
	const FQuat InvParent = ParentWorld.GetRotation().Inverse();
	const FVector LocalPos = InvParent.RotateVector(ChildWorld.GetTranslation() - ParentWorld.GetTranslation());
	return FTransform(InvParent * ChildWorld.GetRotation(), LocalPos, FVector::OneVector);
}

FString ElementName(const mjModel* Model, mjtObj Type, int32 Id, const TCHAR* FallbackPrefix)
{
	const char* Raw = mj_id2name(Model, Type, Id);
	if (Raw && Raw[0] != '\0')
	{
		return MjUtils::MjToString(Raw);
	}
	return FString::Printf(TEXT("%s%d"), FallbackPrefix, Id);
}

/** MuJoCo quaternion (w, x, y, z) multiplication. */
void MulQuatMj(const double* A, const double* B, double* Out)
{
	const double W = A[0] * B[0] - A[1] * B[1] - A[2] * B[2] - A[3] * B[3];
	const double X = A[0] * B[1] + A[1] * B[0] + A[2] * B[3] - A[3] * B[2];
	const double Y = A[0] * B[2] - A[1] * B[3] + A[2] * B[0] + A[3] * B[1];
	const double Z = A[0] * B[3] + A[1] * B[2] - A[2] * B[1] + A[3] * B[0];
	Out[0] = W;
	Out[1] = X;
	Out[2] = Y;
	Out[3] = Z;
}

void RotateVecMj(const double* Q, const double* V, double* Out)
{
	double R[3][3];
	QuatToMatrix(Q, R);
	for (int I = 0; I < 3; ++I)
	{
		Out[I] = R[I][0] * V[0] + R[I][1] * V[1] + R[I][2] * V[2];
	}
}

double Dot3(const double* A, const double* B)
{
	return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
}

/**
 * Adds one body's inertia into an accumulated Unreal tensor about the group's centre
 * of mass (kg cm^2, mirrored axes).
 *
 * @param In       Inertia about the body's own centre of mass, MuJoCo axes, kg m^2.
 * @param Mass     Body mass in kg.
 * @param DeltaCm  Offset from the group centre of mass to the body centre of mass,
 *                 already mirrored into Unreal axes and converted to centimetres.
 * @param Out      Accumulator, kg cm^2 in mirrored axes.
 *
 * Mirroring negates the off-diagonal terms in the Y row/column of the tensor; the
 * parallel-axis term is covariant, so it is evaluated directly in the mirrored frame.
 */
void AccumulateMirroredInertia(const double In[3][3], double Mass, const double* DeltaCm, double Out[3][3])
{
	const double DeltaSq = Dot3(DeltaCm, DeltaCm);
	for (int I = 0; I < 3; ++I)
	{
		for (int J = 0; J < 3; ++J)
		{
			const double Sign = ((I == 1) != (J == 1)) ? -1.0 : 1.0;
			const double Parallel = (I == J ? DeltaSq : 0.0) - DeltaCm[I] * DeltaCm[J];
			Out[I][J] += Sign * In[I][J] * MjTwinConv::MetersToCentimetersSq + Mass * Parallel;
		}
	}
}

/**
 * Reads the per-equality activation flag. Newer MuJoCo releases moved the runtime flag
 * from mjModel to mjData, so both are probed: the first overload is selected when the
 * struct exposes eq_active, and the fallback treats the equality as active.
 */
template <typename TStruct>
auto ReadEqActiveFlag(const TStruct* S, int32 Id, int) -> decltype(S->eq_active[Id] != 0)
{
	return S->eq_active[Id] != 0;
}

template <typename TStruct>
bool ReadEqActiveFlag(const TStruct*, int32, long)
{
	return true;
}

bool NearlyParallel(const FVector& A, const FVector& B, double Tolerance)
{
	const double Denom = A.Size() * B.Size();
	if (Denom <= 1e-12)
	{
		return false;
	}
	return FMath::Abs(FMath::Abs(A.Dot(B)) / Denom - 1.0) < Tolerance;
}
} // namespace

namespace MjTwinConv
{
void MulQuat(const double* A, const double* B, double* Out)
{
	MulQuatMj(A, B, Out);
}

void RotateVec(const double* Q, const double* V, double* Out)
{
	RotateVecMj(Q, V, Out);
}

void InvQuat(const double* Q, double* Out)
{
	Out[0] = Q[0];
	Out[1] = -Q[1];
	Out[2] = -Q[2];
	Out[3] = -Q[3];
}
} // namespace MjTwinConv

int32 FMjTwinDescriptor::FindBodyByMjId(int32 MjBodyId) const
{
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		if (Bodies[Index].MjBodyIds.Contains(MjBodyId))
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

int32 FMjTwinDescriptor::FindBodyByName(const FString& BodyName) const
{
	for (int32 Index = 0; Index < Bodies.Num(); ++Index)
	{
		if (Bodies[Index].Name == BodyName)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

int32 FMjTwinDescriptor::FindJointByName(const FString& JointName) const
{
	for (int32 Index = 0; Index < Joints.Num(); ++Index)
	{
		if (Joints[Index].Name == JointName)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

int32 FMjTwinDescriptor::FindActuatorByName(const FString& ActuatorName) const
{
	for (int32 Index = 0; Index < Actuators.Num(); ++Index)
	{
		if (Actuators[Index].Name == ActuatorName)
		{
			return Index;
		}
	}
	return INDEX_NONE;
}

TArray<FVector> FMjTwinDescriptorBuilder::MakeCylinderPoints(float RadiusCm, float HalfLengthCm, int32 Sides)
{
	Sides = FMath::Max(Sides, 6);
	TArray<FVector> Points;
	Points.Reserve(Sides * 2);
	for (int32 Side = 0; Side < Sides; ++Side)
	{
		const double Angle = 2.0 * PI * static_cast<double>(Side) / static_cast<double>(Sides);
		const double X = FMath::Cos(Angle) * RadiusCm;
		const double Y = FMath::Sin(Angle) * RadiusCm;
		Points.Add(FVector(X, Y, HalfLengthCm));
		Points.Add(FVector(X, Y, -HalfLengthCm));
	}
	return Points;
}

TArray<FVector> FMjTwinDescriptorBuilder::MakeEllipsoidPoints(const FVector& RadiiCm, int32 Segments, int32 Rings)
{
	Segments = FMath::Max(Segments, 6);
	Rings = FMath::Max(Rings, 3);

	TArray<FVector> Points;
	Points.Reserve(Segments * Rings + 2);

	for (int32 Ring = 1; Ring < Rings; ++Ring)
	{
		const double Phi = PI * static_cast<double>(Ring) / static_cast<double>(Rings);
		const double SinPhi = FMath::Sin(Phi);
		const double CosPhi = FMath::Cos(Phi);
		for (int32 Segment = 0; Segment < Segments; ++Segment)
		{
			const double Theta = 2.0 * PI * static_cast<double>(Segment) / static_cast<double>(Segments);
			Points.Add(FVector(
				SinPhi * FMath::Cos(Theta) * RadiiCm.X,
				SinPhi * FMath::Sin(Theta) * RadiiCm.Y,
				CosPhi * RadiiCm.Z));
		}
	}

	// Poles keep the hull closed: without them the tessellated shape is flat in -Z.
	Points.Add(FVector(0.0, 0.0, RadiiCm.Z));
	Points.Add(FVector(0.0, 0.0, -RadiiCm.Z));
	return Points;
}

FQuat FMjTwinDescriptorBuilder::MakeConstraintRotation(const FVector& AxisWorld)
{
	const FVector Axis = AxisWorld.GetSafeNormal();
	if (Axis.IsNearlyZero())
	{
		return FQuat::Identity;
	}

	// Unreal maps twist to the constraint X axis, swing1 to Z, and swing2 to Y.
	// Any orthonormal completion works for a twist or linear X constraint; pick the
	// world axis that is least parallel to the joint axis so the frame stays well conditioned.
	FVector Up = FVector::ZAxisVector;
	if (FMath::Abs(Axis.Dot(Up)) > 0.9)
	{
		Up = FVector::XAxisVector;
	}

	const FVector Y = FVector::CrossProduct(Up, Axis).GetSafeNormal();
	const FVector Z = FVector::CrossProduct(Axis, Y).GetSafeNormal();

	// Rotation matrix whose columns are (Axis, Y, Z).
	double R[3][3];
	R[0][0] = Axis.X; R[1][0] = Axis.Y; R[2][0] = Axis.Z;
	R[0][1] = Y.X;    R[1][1] = Y.Y;    R[2][1] = Y.Z;
	R[0][2] = Z.X;    R[1][2] = Z.Y;    R[2][2] = Z.Z;
	return RotationMatrixToQuat(R);
}

FQuat FMjTwinDescriptorBuilder::MakeRangeOffsetRotation(const FVector& AxisWorld, double MidpointDegrees)
{
	const FVector Axis = AxisWorld.GetSafeNormal();
	if (Axis.IsNearlyZero() || FMath::IsNearlyZero(MidpointDegrees))
	{
		return FQuat::Identity;
	}
	return FQuat(Axis, FMath::DegreesToRadians(MidpointDegrees));
}

bool FMjTwinDescriptorBuilder::DiagonalizeInertia(const FMjTwinTensor& Tensor, FQuat& OutAxes, FVector& OutMoments)
{
	double In[3][3];
	for (int I = 0; I < 3; ++I)
	{
		for (int J = 0; J < 3; ++J)
		{
			In[I][J] = Tensor.M[I][J];
		}
	}

	double Values[3];
	double Vectors[3][3];
	JacobiEigen(In, Values, Vectors);

	// Re-orthonormalise and force a right-handed frame: the eigen solve is exact for a
	// symmetric matrix, but rounding can leave a small residual skew.
	FVector Axis0(Vectors[0][0], Vectors[1][0], Vectors[2][0]);
	FVector Axis1(Vectors[0][1], Vectors[1][1], Vectors[2][1]);
	const double Length0 = Axis0.Size();
	const double Length1 = Axis1.Size();
	if (Length0 <= 1e-12 || Length1 <= 1e-12)
	{
		return false;
	}

	Axis0 /= Length0;
	Axis1 = (Axis1 - Axis0 * Axis1.Dot(Axis0));
	const double Length1Ortho = Axis1.Size();
	if (Length1Ortho <= 1e-12)
	{
		return false;
	}
	Axis1 /= Length1Ortho;
	const FVector Axis2 = FVector::CrossProduct(Axis0, Axis1);

	double R[3][3];
	R[0][0] = Axis0.X; R[1][0] = Axis0.Y; R[2][0] = Axis0.Z;
	R[0][1] = Axis1.X; R[1][1] = Axis1.Y; R[2][1] = Axis1.Z;
	R[0][2] = Axis2.X; R[1][2] = Axis2.Y; R[2][2] = Axis2.Z;

	if (Det3(R) < 0.0)
	{
		return false;
	}

	OutAxes = RotationMatrixToQuat(R);
	OutMoments = FVector(Values[0], Values[1], Values[2]);
	return true;
}

bool FMjTwinDescriptorBuilder::BuildFromModel(const mjModel* Model, FMjTwinDescriptor& Out, FString& OutError,
	const TMap<FString, TSoftObjectPtr<UStaticMesh>>* MeshAssets)
{
	Out = FMjTwinDescriptor();
	if (!Model)
	{
		OutError = TEXT("no model");
		return false;
	}
	if (Model->nbody <= 1)
	{
		OutError = TEXT("model has no bodies beyond the world body");
		return false;
	}

	FMjTwinReport& Report = Out.Report;
	FMjTwinOptions& Options = Out.Options;

	// mjModel::names starts with the model name, followed by every object name.
	Out.ModelName = Model->names ? MjUtils::MjToString(Model->names) : TEXT("<unnamed>");
	Report.ModelName = Out.ModelName;
	Report.NumSourceBodies = Model->nbody;
	Report.NumGeoms = Model->ngeom;
	Report.NumJoints = Model->njnt;
	Report.NumActuators = Model->nu;

	// --- Model options -----------------------------------------------------
	Options.Timestep = Model->opt.timestep;
	Options.Integrator = static_cast<int32>(Model->opt.integrator);
	Options.Cone = static_cast<int32>(Model->opt.cone);
	Options.Solver = static_cast<int32>(Model->opt.solver);
	Options.SolverIterations = Model->opt.iterations;
	Options.SolverLSIterations = Model->opt.ls_iterations;
	Options.Tolerance = Model->opt.tolerance;
	Options.Impratio = Model->opt.impratio;
	Options.bFilterParent = (Model->opt.disableflags & mjDSBL_FILTERPARENT) == 0;
	Options.bContactsEnabled = (Model->opt.disableflags & mjDSBL_CONTACT) == 0;
	Options.bLimitsEnabled = (Model->opt.disableflags & mjDSBL_LIMIT) == 0;
	Options.bEqualityEnabled = (Model->opt.disableflags & mjDSBL_EQUALITY) == 0;

	{
		const FVector Gravity = MjTwinConv::Position(Model->opt.gravity);
		Options.Gravity = Gravity;
		// Unreal gravity is a single world-space -Z value; anything else cannot be copied.
		Options.bGravityIsVertical = FMath::Abs(Gravity.X) < 1e-6 && FMath::Abs(Gravity.Y) < 1e-6;
		Report.Add(TEXT("option/gravity"), Options.bGravityIsVertical ? EMjTwinFidelity::Exact : EMjTwinFidelity::Approximate,
			FString::Printf(TEXT("g = (%.3f, %.3f, %.3f) cm/s^2; Unreal gravity is world -Z only"), Gravity.X, Gravity.Y, Gravity.Z));
	}

	Report.Add(TEXT("option/timestep"), EMjTwinFidelity::Exact,
		FString::Printf(TEXT("%.6f s (%.1f Hz): enable substepping with this step to match MuJoCo's integration rate"),
			Options.Timestep, Options.Timestep > 0.0 ? 1.0 / Options.Timestep : 0.0));

	if (Options.Integrator == static_cast<int32>(mjINT_EULER))
	{
		Report.Add(TEXT("option/integrator"), EMjTwinFidelity::Exact,
			TEXT("MuJoCo Euler maps onto Chaos's semi-implicit Euler substeps"));
	}
	else
	{
		Report.Add(TEXT("option/integrator"), EMjTwinFidelity::Approximate,
			FString::Printf(TEXT("integrator id %d replaced by Chaos's semi-implicit Euler"), Options.Integrator));
	}

	Report.Add(TEXT("option/solver"), EMjTwinFidelity::Approximate,
		FString::Printf(TEXT("MuJoCo solver id %d with %d iterations -> Chaos with %d velocity / %d position iterations"),
			Options.Solver, Options.SolverIterations, Options.SolverIterations, Options.SolverLSIterations));

	Report.Add(TEXT("option/contact_model"), EMjTwinFidelity::Approximate,
		FString::Printf(TEXT("cone id %d and solref/solimp softness replaced by Chaos friction and restitution materials"), Options.Cone));

	// --- Bodies ------------------------------------------------------------
	mjData* Data = mj_makeData(Model);
	if (!Data)
	{
		OutError = TEXT("mj_makeData failed");
		return false;
	}
	mj_resetData(Model, Data);
	mj_forward(Model, Data);

	const int32 NumBodies = Model->nbody;

	// A body with no joint and no mocap is welded into its parent in MuJoCo; the twin
	// therefore folds every weld chain into a single Chaos rigid body.
	TArray<int32> WeldRoot;
	WeldRoot.SetNum(NumBodies);
	TArray<FMassProps> BodyProps;
	BodyProps.SetNum(NumBodies);

	for (int32 BodyId = 0; BodyId < NumBodies; ++BodyId)
	{
		const bool bHasJoint = Model->body_jntnum[BodyId] > 0;
		const bool bMocap = Model->body_mocapid[BodyId] >= 0;
		const bool bWorld = BodyId == 0;
		const bool bWelded = !bWorld && !bHasJoint && !bMocap;

		if (bWorld)
		{
			WeldRoot[BodyId] = 0;
		}
		else if (bWelded)
		{
			WeldRoot[BodyId] = WeldRoot[Model->body_parentid[BodyId]];
		}
		else
		{
			WeldRoot[BodyId] = BodyId;
		}

		FMassProps Props;
		Props.Mass = Model->body_mass[BodyId];

		double WorldCom[3];
		RotateVecMj(Data->xquat + 4 * BodyId, Model->body_ipos + 3 * BodyId, WorldCom);
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			WorldCom[Axis] += Data->xpos[3 * BodyId + Axis];
		}
		const FVector ComUe = MjTwinConv::Position(WorldCom);
		Props.Com[0] = ComUe.X;
		Props.Com[1] = ComUe.Y;
		Props.Com[2] = ComUe.Z;

		// Inertia about the body's own centre of mass, in MuJoCo axes and kg m^2.
		double BodyRot[4];
		MulQuatMj(Data->xquat + 4 * BodyId, Model->body_iquat + 4 * BodyId, BodyRot);
		double BodyRotM[3][3];
		QuatToMatrix(BodyRot, BodyRotM);
		const double* Principal = Model->body_inertia + 3 * BodyId;
		for (int I = 0; I < 3; ++I)
		{
			for (int J = 0; J < 3; ++J)
			{
				Props.Inertia[I][J] = BodyRotM[I][0] * Principal[0] * BodyRotM[J][0]
					+ BodyRotM[I][1] * Principal[1] * BodyRotM[J][1]
					+ BodyRotM[I][2] * Principal[2] * BodyRotM[J][2];
			}
		}
		BodyProps[BodyId] = Props;
	}

	// Group welded bodies, keeping MuJoCo's depth-first order so parents come first.
	// Body 0 (the world) is a root as well: everything welded to it is static geometry
	// in MuJoCo, and the twin mirrors it as a non-simulated collision body.
	TArray<FWeldGroup> Groups;
	TArray<int32> BodyToGroup;
	BodyToGroup.Init(INDEX_NONE, NumBodies);
	TArray<int32> Roots;
	Roots.Add(0);
	for (int32 BodyId = 1; BodyId < NumBodies; ++BodyId)
	{
		if (WeldRoot[BodyId] == BodyId)
		{
			Roots.Add(BodyId);
		}
	}

	for (int32 Root : Roots)
	{
		FWeldGroup Group;
		const int32 FirstMember = Root == 0 ? 1 : Root;
		for (int32 Other = FirstMember; Other < NumBodies; ++Other)
		{
			if (WeldRoot[Other] == Root)
			{
				Group.MjBodyIds.Add(Other);
			}
		}
		if (Group.MjBodyIds.Num() == 0)
		{
			continue;
		}
		for (int32 Member : Group.MjBodyIds)
		{
			BodyToGroup[Member] = Groups.Num();
		}
		Groups.Add(MoveTemp(Group));
	}

	TArray<int32> MjBodyToTwinBody;
	MjBodyToTwinBody.Init(INDEX_NONE, NumBodies);

	// Build one descriptor body per weld group with the combined mass properties.
	Out.Bodies.Reserve(Groups.Num());
	for (FWeldGroup& Group : Groups)
	{
		const int32 RootBody = Group.MjBodyIds[0];
		const bool bWorldGroup = WeldRoot[RootBody] == 0 && RootBody != 0;

		FMassProps Combined;
		for (int32 Member : Group.MjBodyIds)
		{
			Combined.Mass += BodyProps[Member].Mass;
		}

		if (Combined.Mass > SMALL_NUMBER)
		{
			for (int32 Member : Group.MjBodyIds)
			{
				const double Weight = BodyProps[Member].Mass / Combined.Mass;
				for (int Axis = 0; Axis < 3; ++Axis)
				{
					Combined.Com[Axis] += Weight * BodyProps[Member].Com[Axis];
				}
			}
		}
		else
		{
			// A massless body: the centre of mass is irrelevant in MuJoCo, but Chaos needs
			// a finite mass. Keep the group's average COM and let the caller clamp the mass.
			for (int32 Member : Group.MjBodyIds)
			{
				for (int Axis = 0; Axis < 3; ++Axis)
				{
					Combined.Com[Axis] += BodyProps[Member].Com[Axis] / Group.MjBodyIds.Num();
				}
			}
		}

		for (int32 Member : Group.MjBodyIds)
		{
			const FMassProps& Props = BodyProps[Member];
			double Delta[3];
			for (int Axis = 0; Axis < 3; ++Axis)
			{
				Delta[Axis] = Props.Com[Axis] - Combined.Com[Axis];
			}
			AccumulateMirroredInertia(Props.Inertia, Props.Mass, Delta, Combined.Inertia);
		}

		// Chaos stores a diagonal inertia in the body frame, so the twin body's frame is
		// chosen to be the principal frame of the combined tensor and the copy is exact.
		// FMatrix is single precision; the combined tensor is kept in double so the
		// diagonal copy Chaos receives is numerically exact.
		FMjTwinTensor Tensor;
		for (int I = 0; I < 3; ++I)
		{
			for (int J = 0; J < 3; ++J)
			{
				Tensor.M[I][J] = Combined.Inertia[I][J];
			}
		}

		FQuat PrincipalAxes = FQuat::Identity;
		FVector Moments = FVector::ZeroVector;
		bool bDegenerate = false;
		if (!DiagonalizeInertia(Tensor, PrincipalAxes, Moments) || Moments.GetMax() < DegenerateInertiaEpsilon)
		{
			// A body with (near) zero inertia cannot spin: fall back to the MuJoCo
			// principal axes of the group root and clamp the moments.
			bDegenerate = true;
			double WorldPrincipal[4];
			MulQuatMj(Data->xquat + 4 * RootBody, Model->body_iquat + 4 * RootBody, WorldPrincipal);
			PrincipalAxes = MjTwinConv::Rotation(WorldPrincipal);
			Moments = FVector(
				FMath::Max(FMath::Abs(Combined.Inertia[0][0]), DegenerateInertiaEpsilon),
				FMath::Max(FMath::Abs(Combined.Inertia[1][1]), DegenerateInertiaEpsilon),
				FMath::Max(FMath::Abs(Combined.Inertia[2][2]), DegenerateInertiaEpsilon));
		}

		FMjTwinBody Body;
		Body.Name = bWorldGroup && Group.MjBodyIds.Num() == 1
			? ElementName(Model, mjOBJ_BODY, RootBody, TEXT("body"))
			: (bWorldGroup ? FString::Printf(TEXT("%s_static"), *ElementName(Model, mjOBJ_BODY, RootBody, TEXT("body")))
				: ElementName(Model, mjOBJ_BODY, RootBody, TEXT("body")));
		Body.MjBodyId = RootBody;
		Body.MjBodyIds = Group.MjBodyIds;
		Body.MassKg = Combined.Mass;
		Body.InertiaDiagonal = Moments;
		// The inertia was combined in mirrored world axes, so the principal axes are world
		// axes as well: the physics frame has the group COM at its origin, which is what
		// Chaos stores (a diagonal inertia with no offset).
		Body.PhysicsFrameWorldPos = FVector(Combined.Com[0], Combined.Com[1], Combined.Com[2]);
		Body.PhysicsFrameWorldRot = PrincipalAxes;
		Body.BodyFrameWorldPos = MjTwinConv::Position(Data->xpos + 3 * RootBody);
		Body.BodyFrameWorldRot = MjTwinConv::Rotation(Data->xquat + 4 * RootBody);
		Body.GravityCompensation = Model->body_gravcomp[RootBody];
		Body.bMocap = Model->body_mocapid[RootBody] >= 0;
		Body.bDegenerateInertia = bDegenerate;
		// Bodies welded to the parent have joints further down their chain; bodies welded
		// to the world never move in MuJoCo, so the twin keeps them static as well.
		Body.bSimulated = !Body.bMocap && !bWorldGroup;

		Group.TwinBodyIndex = Out.Bodies.Add(Body);
		for (int32 Member : Group.MjBodyIds)
		{
			MjBodyToTwinBody[Member] = Group.TwinBodyIndex;
		}
	}

	// Parent links between twin bodies, and the joint that connects them.
	for (int32 Index = 0; Index < Out.Bodies.Num(); ++Index)
	{
		const int32 RootBody = Out.Bodies[Index].MjBodyId;
		const int32 ParentMjBody = Model->body_parentid[RootBody];
		if (ParentMjBody > 0)
		{
			Out.Bodies[Index].ParentBodyIndex = MjBodyToTwinBody[ParentMjBody];
		}
	}

	// Report bodies and their mass properties.
	for (const FMjTwinBody& Body : Out.Bodies)
	{
		if (!Body.bSimulated && !Body.bMocap)
		{
			Report.Add(FString::Printf(TEXT("body/%s"), *Body.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("%d MuJoCo bodies welded to the world mirrored as static collision geometry"),
					Body.MjBodyIds.Num()));
		}
		else if (Body.bMocap)
		{
			Report.Add(FString::Printf(TEXT("body/%s"), *Body.Name), EMjTwinFidelity::Approximate,
				TEXT("mocap body: kept kinematic, which is how MuJoCo treats it as well"));
		}
		else if (Body.MjBodyIds.Num() > 1)
		{
			Report.Add(FString::Printf(TEXT("body/%s"), *Body.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("weld group of %d MuJoCo bodies: combined mass %.6g kg and combined inertia copied"),
					Body.MjBodyIds.Num(), Body.MassKg));
		}
		else
		{
			Report.Add(FString::Printf(TEXT("body/%s"), *Body.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("mass %.6g kg, principal inertia (%.6g, %.6g, %.6g) kg cm^2"),
					Body.MassKg, Body.InertiaDiagonal.X, Body.InertiaDiagonal.Y, Body.InertiaDiagonal.Z));
		}

		if (Body.MassKg <= 0.0 && Body.bSimulated)
		{
			Report.Add(FString::Printf(TEXT("body/%s/mass"), *Body.Name), EMjTwinFidelity::Approximate,
				TEXT("massless in MuJoCo (infinite mass); Chaos needs a positive mass, so the twin instead uses a minimal 1 g"));
		}
		if (Body.bDegenerateInertia)
		{
			Report.Add(FString::Printf(TEXT("body/%s/inertia"), *Body.Name), EMjTwinFidelity::Approximate,
				TEXT("degenerate inertia tensor: principal axes fall back to the MuJoCo body frame and the moments are clamped"));
		}
		if (!FMath::IsNearlyZero(Body.GravityCompensation))
		{
			Report.Add(FString::Printf(TEXT("body/%s/gravcomp"), *Body.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("gravity compensation %.4g applied as an external force every step"), Body.GravityCompensation));
		}
	}

	// --- Geoms -------------------------------------------------------------
	int32 NumTwinShapes = 0;
	int32 NumUnsupportedGeoms = 0;
	for (int32 GeomId = 0; GeomId < Model->ngeom; ++GeomId)
	{
		const int32 MjBodyId = Model->geom_bodyid[GeomId];
		const int32 TwinBodyIndex = MjBodyToTwinBody[MjBodyId];
		if (TwinBodyIndex == INDEX_NONE)
		{
			continue;
		}

		FMjTwinShape Shape;
		Shape.BodyIndex = TwinBodyIndex;
		Shape.MjGeomId = GeomId;
		Shape.Name = ElementName(Model, mjOBJ_GEOM, GeomId, TEXT("geom"));
		Shape.ConType = Model->geom_contype[GeomId];
		Shape.ConAffinity = Model->geom_conaffinity[GeomId];
		Shape.Group = Model->geom_group[GeomId];
		Shape.Margin = static_cast<float>(Model->geom_margin[GeomId] * MjTwinConv::MetersToCentimeters);
		Shape.Gap = static_cast<float>(Model->geom_gap[GeomId] * MjTwinConv::MetersToCentimeters);
		Shape.bCollides = (Shape.ConType | Shape.ConAffinity) != 0 && Options.bContactsEnabled;
		Shape.bVisual = Shape.Group >= 0 && Shape.Group <= 2;
		Shape.Friction = FVector(Model->geom_friction[3 * GeomId], Model->geom_friction[3 * GeomId + 1],
			Model->geom_friction[3 * GeomId + 2]);

		// World pose of the geom: the body's world pose composed with the geom's local pose.
		double GeomWorldQuat[4];
		MulQuatMj(Data->xquat + 4 * MjBodyId, Model->geom_quat + 4 * GeomId, GeomWorldQuat);
		double GeomWorldPos[3];
		RotateVecMj(Data->xquat + 4 * MjBodyId, Model->geom_pos + 3 * GeomId, GeomWorldPos);
		for (int Axis = 0; Axis < 3; ++Axis)
		{
			GeomWorldPos[Axis] += Data->xpos[3 * MjBodyId + Axis];
		}

		const FTransform GeomWorld(MjTwinConv::Rotation(GeomWorldQuat), MjTwinConv::Position(GeomWorldPos), FVector::OneVector);
		const FTransform BodyFrame(Out.Bodies[TwinBodyIndex].PhysicsFrameWorldRot,
			Out.Bodies[TwinBodyIndex].PhysicsFrameWorldPos, FVector::OneVector);
		Shape.LocalTransform = MakeRelativeTransform(GeomWorld, BodyFrame);

		const double* Size = Model->geom_size + 3 * GeomId;
		const int32 GeomType = Model->geom_type[GeomId];
		const double Cm = MjTwinConv::MetersToCentimeters;

		switch (GeomType)
		{
			case mjGEOM_PLANE:
			{
				// MuJoCo planes are infinite half-spaces pointing along +Z of the geom
				// frame; a slab whose -Z face sits on the plane is the closest Chaos match.
				Shape.Type = EMjTwinShapeType::PlaneSlab;
				Shape.Size = FVector(PlaneSlabHalfExtentCm, PlaneSlabHalfExtentCm, PlaneSlabHalfThicknessCm);
				const FVector Offset = Shape.LocalTransform.GetRotation().RotateVector(FVector(0.0, 0.0, -PlaneSlabHalfThicknessCm));
				Shape.LocalTransform.SetTranslation(Shape.LocalTransform.GetTranslation() + Offset);
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Approximate,
					FString::Printf(TEXT("infinite half-space approximated by a %.0f cm slab whose surface lies on the plane"),
						PlaneSlabHalfExtentCm * 2.0));
				break;
			}

			case mjGEOM_SPHERE:
				Shape.Type = EMjTwinShapeType::Sphere;
				Shape.Size = FVector(Size[0] * Cm, 0.0, 0.0);
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Exact,
					FString::Printf(TEXT("sphere radius %.4f cm (FKSphereElem)"), Shape.Size.X));
				break;

			case mjGEOM_CAPSULE:
				Shape.Type = EMjTwinShapeType::Capsule;
				Shape.Size = FVector(Size[0] * Cm, Size[1] * Cm, 0.0);
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Exact,
					FString::Printf(TEXT("capsule radius %.4f cm, cylinder length %.4f cm (FKSphylElem; Z is the axis in both engines)"),
						Shape.Size.X, Shape.Size.Y * 2.0));
				break;

			case mjGEOM_BOX:
				Shape.Type = EMjTwinShapeType::Box;
				Shape.Size = FVector(Size[0], Size[1], Size[2]) * Cm;
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Exact,
					FString::Printf(TEXT("box half extents (%.4f, %.4f, %.4f) cm (FKBoxElem)"),
						Shape.Size.X, Shape.Size.Y, Shape.Size.Z));
				break;

			case mjGEOM_CYLINDER:
				Shape.Type = EMjTwinShapeType::Cylinder;
				Shape.Size = FVector(Size[0] * Cm, Size[1] * Cm, 0.0);
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Approximate,
					FString::Printf(TEXT("cylinder radius %.4f cm, length %.4f cm tessellated into a convex hull (Chaos has no cylinder primitive)"),
						Shape.Size.X, Shape.Size.Y * 2.0));
				break;

			case mjGEOM_ELLIPSOID:
				Shape.Type = EMjTwinShapeType::Ellipsoid;
				Shape.Size = FVector(Size[0], Size[1], Size[2]) * Cm;
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Approximate,
					FString::Printf(TEXT("ellipsoid radii (%.4f, %.4f, %.4f) cm tessellated into a convex hull"),
						Shape.Size.X, Shape.Size.Y, Shape.Size.Z));
				break;

			case mjGEOM_MESH:
			{
				Shape.Type = EMjTwinShapeType::Mesh;
				Shape.MjMeshId = Model->geom_dataid[GeomId];
				const double BoundingRadius = Model->geom_rbound[GeomId] * Cm;
				Shape.Size = FVector(BoundingRadius, BoundingRadius, BoundingRadius);

				const FString MeshName = Shape.MjMeshId >= 0 && Shape.MjMeshId < Model->nmesh
					? ElementName(Model, mjOBJ_MESH, Shape.MjMeshId, TEXT("mesh"))
					: FString();

				if (MeshAssets != nullptr && !MeshName.IsEmpty())
				{
					const TSoftObjectPtr<UStaticMesh>* Found = MeshAssets->Find(MeshName);
					if (Found == nullptr)
					{
						// Decomposed hulls are named "<mesh>_<index>": fall back to the base mesh.
						int32 Underscore = INDEX_NONE;
						if (MeshName.FindLastChar(TEXT('_'), Underscore))
						{
							Found = MeshAssets->Find(MeshName.Left(Underscore));
						}
					}
					if (Found != nullptr)
					{
						Shape.MeshAsset = *Found;
					}
				}

				if (Shape.MeshAsset.IsNull())
				{
					Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Approximate,
						FString::Printf(TEXT("mesh '%s' has no Unreal asset: collision falls back to a bounding box"),
							MeshName.IsEmpty() ? TEXT("<unnamed>") : *MeshName));
				}
				else
				{
					Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Exact,
						FString::Printf(TEXT("mesh '%s' uses the imported asset's convex hull collision"), *MeshName));
				}
				break;
			}

			case mjGEOM_HFIELD:
				Shape.Type = EMjTwinShapeType::Unsupported;
				Shape.bUnsupported = true;
				++NumUnsupportedGeoms;
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Unsupported,
					TEXT("heightfield geoms have no Chaos equivalent; add a collision actor for the terrain instead"));
				break;

			case mjGEOM_SDF:
				Shape.Type = EMjTwinShapeType::Unsupported;
				Shape.bUnsupported = true;
				++NumUnsupportedGeoms;
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Unsupported,
					TEXT("SDF geoms have no Chaos equivalent"));
				break;

			default:
				Shape.Type = EMjTwinShapeType::Unsupported;
				Shape.bUnsupported = true;
				++NumUnsupportedGeoms;
				Report.Add(FString::Printf(TEXT("geom/%s"), *Shape.Name), EMjTwinFidelity::Unsupported,
					FString::Printf(TEXT("geom type %d is not mirrored"), GeomType));
				break;
		}

		if (Shape.bUnsupported)
		{
			Out.Shapes.Add(MoveTemp(Shape));
			continue;
		}

		++NumTwinShapes;
		if (!Shape.bCollides)
		{
			Report.Add(FString::Printf(TEXT("geom/%s/filter"), *Shape.Name), EMjTwinFidelity::Approximate,
				FString::Printf(TEXT("contype=%d conaffinity=%d: this geom generates no contacts"),
					Shape.ConType, Shape.ConAffinity));
		}
		Out.Shapes.Add(MoveTemp(Shape));
	}

	// --- Joints ------------------------------------------------------------
	TMap<int32, int32> MjJointToTwinJoint;
	Out.Joints.Reserve(Model->njnt);
	for (int32 JointId = 0; JointId < Model->njnt; ++JointId)
	{
		const int32 MjBodyId = Model->jnt_bodyid[JointId];
		const int32 TwinChild = MjBodyToTwinBody[MjBodyId];
		if (TwinChild == INDEX_NONE)
		{
			continue;
		}

		const int32 ParentMjBody = Model->body_parentid[MjBodyId];
		const int32 TwinParent = ParentMjBody > 0 ? MjBodyToTwinBody[ParentMjBody] : INDEX_NONE;

		FMjTwinJoint Joint;
		Joint.MjJointId = JointId;
		Joint.Name = ElementName(Model, mjOBJ_JOINT, JointId, TEXT("joint"));
		Joint.ParentBodyIndex = TwinParent;
		Joint.ChildBodyIndex = TwinChild;
		Joint.DofAdr = Model->jnt_dofadr[JointId];
		Joint.QposAdr = Model->jnt_qposadr[JointId];
		Joint.QposNum = Model->jnt_type[JointId] == mjJNT_FREE ? 7 : (Model->jnt_type[JointId] == mjJNT_BALL ? 4 : 1);
		Joint.DofNum = Model->jnt_type[JointId] == mjJNT_FREE ? 6 : (Model->jnt_type[JointId] == mjJNT_BALL ? 3 : 1);
		Joint.bCollideWithParent = !Options.bFilterParent;

		if (Joint.DofAdr >= 0)
		{
			Joint.Damping = Model->dof_damping[Joint.DofAdr];
			Joint.FrictionLoss = Model->dof_frictionloss[Joint.DofAdr];
			Joint.Armature = Model->dof_armature[Joint.DofAdr];
		}
		Joint.Stiffness = Model->jnt_stiffness[JointId];
		Joint.bHasSpring = !FMath::IsNearlyZero(Joint.Stiffness);

		// The constraint frame's X axis is the joint axis. Hinge axes are mirrored as
		// axial vectors so a positive Unreal rotation equals a positive MuJoCo qpos;
		// slide axes are mirrored as polar vectors so a positive displacement matches.
		const double* Anchor = Data->xanchor + 3 * JointId;
		const double* Axis = Data->xaxis + 3 * JointId;
		const bool bLimited = Model->jnt_limited[JointId] != 0 && Options.bLimitsEnabled;
		Joint.bLimited = bLimited;

		switch (Model->jnt_type[JointId])
		{
			case mjJNT_HINGE:
			{
				Joint.Type = EMjTwinJointType::Hinge;
				Joint.AxisWorld = MjTwinConv::Axial(Axis).GetSafeNormal();
				Joint.AnchorWorldPos = MjTwinConv::Position(Anchor);
				Joint.ConstraintWorldPos = Joint.AnchorWorldPos;

				if (bLimited)
				{
					const double Low = FMath::RadiansToDegrees(Model->jnt_range[2 * JointId]);
					const double High = FMath::RadiansToDegrees(Model->jnt_range[2 * JointId + 1]);
					Joint.RangeMin = FMath::Min(Low, High);
					Joint.RangeMax = FMath::Max(Low, High);
					Joint.LimitOffset = 0.5 * (Joint.RangeMin + Joint.RangeMax);
					Joint.HalfRange = 0.5 * (Joint.RangeMax - Joint.RangeMin);
					// Chaos limits are symmetric about the constraint frame, so the frame is
					// rotated to the middle of the range and the limit is the half range.
					Joint.ConstraintWorldRot = MakeRangeOffsetRotation(Joint.AxisWorld, Joint.LimitOffset)
						* MakeConstraintRotation(Joint.AxisWorld);
					Report.Add(FString::Printf(TEXT("joint/%s/limits"), *Joint.Name), EMjTwinFidelity::Exact,
						FString::Printf(TEXT("range [%.4f, %.4f] deg reproduced by offsetting the constraint frame to the range midpoint"),
							Joint.RangeMin, Joint.RangeMax));
				}
				else
				{
					Joint.ConstraintWorldRot = MakeConstraintRotation(Joint.AxisWorld);
				}

				Report.Add(FString::Printf(TEXT("joint/%s"), *Joint.Name), EMjTwinFidelity::Exact,
					TEXT("hinge -> Chaos twist limit about the constraint X axis"));
				break;
			}

			case mjJNT_SLIDE:
			{
				Joint.Type = EMjTwinJointType::Slide;
				Joint.AxisWorld = MjTwinConv::Direction(Axis).GetSafeNormal();
				Joint.AnchorWorldPos = MjTwinConv::Position(Anchor);
				Joint.ConstraintWorldRot = MakeConstraintRotation(Joint.AxisWorld);

				if (bLimited)
				{
					Joint.RangeMin = Model->jnt_range[2 * JointId] * Cm;
					Joint.RangeMax = Model->jnt_range[2 * JointId + 1] * Cm;
					if (Joint.RangeMin > Joint.RangeMax)
					{
						Swap(Joint.RangeMin, Joint.RangeMax);
					}
					Joint.LimitOffset = 0.5 * (Joint.RangeMin + Joint.RangeMax);
					Joint.HalfRange = 0.5 * (Joint.RangeMax - Joint.RangeMin);
					Report.Add(FString::Printf(TEXT("joint/%s/limits"), *Joint.Name), EMjTwinFidelity::Exact,
						FString::Printf(TEXT("range [%.4f, %.4f] cm reproduced by offsetting the constraint frame to the range midpoint"),
							Joint.RangeMin, Joint.RangeMax));
				}

				// The Chaos linear limit is symmetric about the constraint frame; moving the
				// frame to the range midpoint makes it span the MuJoCo range exactly.
				Joint.ConstraintWorldPos = Joint.AnchorWorldPos + Joint.AxisWorld * Joint.LimitOffset;
				Report.Add(FString::Printf(TEXT("joint/%s"), *Joint.Name), EMjTwinFidelity::Exact,
					TEXT("slide -> Chaos linear X limit along the constraint X axis"));
				break;
			}

			case mjJNT_BALL:
			{
				Joint.Type = EMjTwinJointType::FreeSpin;
				// A ball joint has no single axis; the child's rest orientation defines the
				// constraint frame so swing and twist are measured from the rest pose.
				Joint.AxisWorld = MjTwinConv::Axial(Axis).GetSafeNormal();
				Joint.AnchorWorldPos = MjTwinConv::Position(Anchor);
				Joint.ConstraintWorldPos = Joint.AnchorWorldPos;
				Joint.ConstraintWorldRot = Out.Bodies[TwinChild].PhysicsFrameWorldRot;

				if (bLimited)
				{
					Joint.SwingLimitDeg = FMath::RadiansToDegrees(Model->jnt_range[2 * JointId]);
					Joint.TwistLimitDeg = FMath::RadiansToDegrees(Model->jnt_range[2 * JointId + 1]);
					if (Joint.TwistLimitDeg <= 0.0)
					{
						Joint.TwistLimitDeg = Joint.SwingLimitDeg;
					}
					Report.Add(FString::Printf(TEXT("joint/%s/limits"), *Joint.Name), EMjTwinFidelity::Approximate,
						FString::Printf(TEXT("ball cone %.4f deg / twist %.4f deg -> Chaos swing1/swing2/twist limits (different parameterisation)"),
							Joint.SwingLimitDeg, Joint.TwistLimitDeg));
				}

				Report.Add(FString::Printf(TEXT("joint/%s"), *Joint.Name), EMjTwinFidelity::Approximate,
					TEXT("ball joint -> Chaos swing/twist constraint; the quaternion and angular velocity dofs differ internally"));
				break;
			}

			case mjJNT_FREE:
			{
				Joint.Type = EMjTwinJointType::Free;
				Joint.AxisWorld = MjTwinConv::Axial(Axis).GetSafeNormal();
				Joint.AnchorWorldPos = MjTwinConv::Position(Anchor);
				Joint.ConstraintWorldPos = Joint.AnchorWorldPos;
				Joint.ConstraintWorldRot = Out.Bodies[TwinChild].PhysicsFrameWorldRot;
				Report.Add(FString::Printf(TEXT("joint/%s"), *Joint.Name), EMjTwinFidelity::Exact,
					TEXT("free joint -> fully unconstrained Chaos body (6 dofs)"));
				break;
			}

			default:
			{
				Joint.Type = EMjTwinJointType::Free;
				Report.Add(FString::Printf(TEXT("joint/%s"), *Joint.Name), EMjTwinFidelity::Unsupported,
					FString::Printf(TEXT("joint type %d is not mirrored; the body is left free"), Model->jnt_type[JointId]));
				break;
			}
		}

		if (Joint.bHasSpring)
		{
			Joint.SpringRef = Model->qpos0[Joint.QposAdr];
			if (Joint.Type == EMjTwinJointType::Hinge)
			{
				Joint.SpringRef = FMath::RadiansToDegrees(Joint.SpringRef);
			}
			else if (Joint.Type == EMjTwinJointType::Slide)
			{
				Joint.SpringRef *= Cm;
			}
			Report.Add(FString::Printf(TEXT("joint/%s/spring"), *Joint.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("spring stiffness %.6g applied about the joint axis about the qpos0 reference"), Joint.Stiffness));
		}
		if (!FMath::IsNearlyZero(Joint.Damping))
		{
			Report.Add(FString::Printf(TEXT("joint/%s/damping"), *Joint.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("dof damping %.6g applied as a force along the joint axis"), Joint.Damping));
		}
		if (!FMath::IsNearlyZero(Joint.FrictionLoss))
		{
			Report.Add(FString::Printf(TEXT("joint/%s/frictionloss"), *Joint.Name), EMjTwinFidelity::Approximate,
				FString::Printf(TEXT("dof friction loss %.6g applied as Coulomb friction along the joint axis"), Joint.FrictionLoss));
		}
		if (!FMath::IsNearlyZero(Joint.Armature))
		{
			Report.Add(FString::Printf(TEXT("joint/%s/armature"), *Joint.Name), EMjTwinFidelity::Exact,
				FString::Printf(TEXT("dof armature %.6g folded into the child body's inertia tensor"), Joint.Armature));
		}

		MjJointToTwinJoint.Add(JointId, Out.Joints.Num());
		Out.Joints.Add(MoveTemp(Joint));
	}

	// Fold armature into the child body's inertia along the joint axis. Chaos stores a
	// diagonal tensor, so this is only exact when the axis is a principal axis.
	for (FMjTwinJoint& Joint : Out.Joints)
	{
		if (Joint.DofNum != 1 || FMath::IsNearlyZero(Joint.Armature) || Joint.ChildBodyIndex == INDEX_NONE)
		{
			continue;
		}

		FMjTwinBody& Body = Out.Bodies[Joint.ChildBodyIndex];
		const FVector LocalAxis = Body.PhysicsFrameWorldRot.Inverse().RotateVector(Joint.AxisWorld);
		const double ArmatureCm2 = Joint.Armature * MjTwinConv::MetersToCentimetersSq;

		if (NearlyParallel(LocalAxis, FVector(1.0, 0.0, 0.0), AxisAlignmentTolerance))
		{
			Body.InertiaDiagonal.X += ArmatureCm2;
			Joint.bArmatureAppliedToInertia = true;
		}
		else if (NearlyParallel(LocalAxis, FVector(0.0, 1.0, 0.0), AxisAlignmentTolerance))
		{
			Body.InertiaDiagonal.Y += ArmatureCm2;
			Joint.bArmatureAppliedToInertia = true;
		}
		else if (NearlyParallel(LocalAxis, FVector(0.0, 0.0, 1.0), AxisAlignmentTolerance))
		{
			Body.InertiaDiagonal.Z += ArmatureCm2;
			Joint.bArmatureAppliedToInertia = true;
		}
	}

	// The joint that connects each twin body to its parent, for callers that walk the
	// hierarchy (the physics side works through the constraints instead).
	for (int32 BodyIndex = 0; BodyIndex < Out.Bodies.Num(); ++BodyIndex)
	{
		for (int32 JointIndex = 0; JointIndex < Out.Joints.Num(); ++JointIndex)
		{
			if (Out.Joints[JointIndex].ChildBodyIndex == BodyIndex
				&& Out.Joints[JointIndex].ParentBodyIndex == Out.Bodies[BodyIndex].ParentBodyIndex)
			{
				Out.Bodies[BodyIndex].ParentJointIndex = JointIndex;
				break;
			}
		}
	}

	for (const FMjTwinJoint& Joint : Out.Joints)
	{
		if (Joint.DofNum == 1 && !FMath::IsNearlyZero(Joint.Armature) && !Joint.bArmatureAppliedToInertia)
		{
			Report.Add(FString::Printf(TEXT("joint/%s/armature"), *Joint.Name), EMjTwinFidelity::Approximate,
				FString::Printf(TEXT("armature %.6g does not lie on a principal inertia axis, and Chaos's diagonal inertia cannot hold it"),
					Joint.Armature));
		}
	}

	// --- Actuators ---------------------------------------------------------
	Out.Actuators.Reserve(Model->nu);
	for (int32 ActuatorId = 0; ActuatorId < Model->nu; ++ActuatorId)
	{
		FMjTwinActuator Actuator;
		Actuator.MjActuatorId = ActuatorId;
		Actuator.Name = ElementName(Model, mjOBJ_ACTUATOR, ActuatorId, TEXT("actuator"));
		Actuator.GainType = Model->actuator_gaintype[ActuatorId];
		Actuator.BiasType = Model->actuator_biastype[ActuatorId];
		Actuator.DynType = Model->actuator_dyntype[ActuatorId];
		Actuator.GainPrm.Reserve(mjNGAIN);
		Actuator.BiasPrm.Reserve(mjNBIAS);
		Actuator.DynPrm.Reserve(mjNDYN);
		for (int Index = 0; Index < mjNGAIN; ++Index)
		{
			Actuator.GainPrm.Add(Model->actuator_gainprm[ActuatorId * mjNGAIN + Index]);
		}
		for (int Index = 0; Index < mjNBIAS; ++Index)
		{
			Actuator.BiasPrm.Add(Model->actuator_biasprm[ActuatorId * mjNBIAS + Index]);
		}
		for (int Index = 0; Index < mjNDYN; ++Index)
		{
			Actuator.DynPrm.Add(Model->actuator_dynprm[ActuatorId * mjNDYN + Index]);
		}
		for (int Index = 0; Index < 6; ++Index)
		{
			Actuator.GearVector.Add(Model->actuator_gear[ActuatorId * 6 + Index]);
		}
		Actuator.Gear = Actuator.GearVector[0];
		Actuator.Damping = Model->actuator_damping[ActuatorId];
		Actuator.bHasActivationDynamics = Actuator.DynType != static_cast<int32>(mjDYN_NONE);
		Actuator.bCtrlLimited = Model->actuator_ctrllimited[ActuatorId] != 0;
		Actuator.CtrlMin = Model->actuator_ctrlrange[2 * ActuatorId];
		Actuator.CtrlMax = Model->actuator_ctrlrange[2 * ActuatorId + 1];
		Actuator.bForceLimited = Model->actuator_forcelimited[ActuatorId] != 0;
		Actuator.ForceMin = Model->actuator_forcerange[2 * ActuatorId];
		Actuator.ForceMax = Model->actuator_forcerange[2 * ActuatorId + 1];

		const int32 TrnType = Model->actuator_trntype[ActuatorId];
		const int32 TrnId = Model->actuator_trnid[2 * ActuatorId];
		const bool bJointTransmission = TrnType == mjTRN_JOINT || TrnType == mjTRN_JOINTINPARENT;
		const bool bBodyTransmission = TrnType == mjTRN_BODY;

		if (bJointTransmission)
		{
			const int32* TwinJoint = MjJointToTwinJoint.Find(TrnId);
			if (TwinJoint != nullptr)
			{
				Actuator.JointIndex = *TwinJoint;
				const FMjTwinJoint& Joint = Out.Joints[*TwinJoint];
				Actuator.BodyIndex = Joint.ChildBodyIndex;
				Actuator.AxisWorld = Joint.AxisWorld;
				Actuator.bAngular = Joint.Type == EMjTwinJointType::Hinge || Joint.Type == EMjTwinJointType::FreeSpin;
			}
		}
		else if (bBodyTransmission)
		{
			const int32 TwinBody = MjBodyToTwinBody[TrnId];
			if (TwinBody != INDEX_NONE)
			{
				Actuator.BodyIndex = TwinBody;
				Actuator.bAngular = false;
				Actuator.AxisWorld = FVector(Actuator.GearVector[3], Actuator.GearVector[4], Actuator.GearVector[5]).GetSafeNormal();
				if (Actuator.AxisWorld.IsNearlyZero())
				{
					Actuator.AxisWorld = FVector::XAxisVector;
				}
			}
		}

		const bool bMuscle = Actuator.GainType == static_cast<int32>(mjGAIN_MUSCLE)
			|| Actuator.BiasType == static_cast<int32>(mjBIAS_MUSCLE);
		const bool bUserCallback = Actuator.GainType == static_cast<int32>(mjGAIN_USER)
			|| Actuator.BiasType == static_cast<int32>(mjBIAS_USER)
			|| Actuator.DynType == static_cast<int32>(mjDYN_USER);
		const bool bPlugin = Model->actuator_plugin[ActuatorId] >= 0;
		const bool bScalarJoint = bJointTransmission && Actuator.JointIndex != INDEX_NONE
			&& (Out.Joints[Actuator.JointIndex].Type == EMjTwinJointType::Hinge
				|| Out.Joints[Actuator.JointIndex].Type == EMjTwinJointType::Slide);

		if (bMuscle)
		{
			Actuator.Model = EMjTwinActuatorModel::Unsupported;
			Actuator.bUnsupported = true;
			Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Unsupported,
				TEXT("muscle gain/bias is a nonlinear model with no Chaos equivalent"));
		}
		else if (bUserCallback)
		{
			Actuator.Model = EMjTwinActuatorModel::Unsupported;
			Actuator.bUnsupported = true;
			Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Unsupported,
				TEXT("user gain/bias/dynamics callbacks cannot be called from the twin"));
		}
		else if (bPlugin)
		{
			Actuator.Model = EMjTwinActuatorModel::Unsupported;
			Actuator.bUnsupported = true;
			Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Unsupported,
				TEXT("actuator plugins are engine-side code and are not mirrored"));
		}
		else if (!bJointTransmission && !bBodyTransmission)
		{
			Actuator.Model = EMjTwinActuatorModel::Unsupported;
			Actuator.bUnsupported = true;
			Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Unsupported,
				FString::Printf(TEXT("transmission type %d (tendon, site, or slider-crank) needs MuJoCo's transmission kinematics"),
					TrnType));
		}
		else if (bJointTransmission && !bScalarJoint)
		{
			Actuator.Model = EMjTwinActuatorModel::Unsupported;
			Actuator.bUnsupported = true;
			Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Unsupported,
				TEXT("actuators on multi-dof (free or ball) joints need a full moment matrix; only scalar joints are mirrored"));
		}
		else
		{
			if (Actuator.bHasActivationDynamics)
			{
				Actuator.Model = EMjTwinActuatorModel::GeneralAffine;
				Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Approximate,
					FString::Printf(TEXT("activation dynamics (dyntype %d) integrated by the twin; filter and muscle-activation limits are approximate"),
						Actuator.DynType));
			}
			else
			{
				const bool bFixedGain = Actuator.GainType == static_cast<int32>(mjGAIN_FIXED);
				const bool bNoBias = Actuator.BiasType == static_cast<int32>(mjBIAS_NONE);
				const bool bAffineBias = Actuator.BiasType == static_cast<int32>(mjBIAS_AFFINE);

				if (bFixedGain && bNoBias)
				{
					Actuator.Model = EMjTwinActuatorModel::Motor;
				}
				else if (bFixedGain && bAffineBias && Actuator.BiasPrm.Num() >= 3)
				{
					const bool bPosition = !FMath::IsNearlyZero(Actuator.BiasPrm[1]) && !FMath::IsNearlyZero(Actuator.BiasPrm[2]);
					const bool bVelocity = FMath::IsNearlyZero(Actuator.BiasPrm[1]) && !FMath::IsNearlyZero(Actuator.BiasPrm[2]);
					Actuator.Model = bPosition ? EMjTwinActuatorModel::Position
						: (bVelocity ? EMjTwinActuatorModel::Velocity : EMjTwinActuatorModel::GeneralAffine);
				}
				else
				{
					Actuator.Model = EMjTwinActuatorModel::GeneralAffine;
				}

				Report.Add(FString::Printf(TEXT("actuator/%s"), *Actuator.Name), EMjTwinFidelity::Exact,
					FString::Printf(TEXT("force = gain*act + bias evaluated every physics step (gain %.6g, bias %.6g / %.6g), gear %.6g, forcerange [%.6g, %.6g]"),
						Actuator.GainPrm[0],
						Actuator.BiasPrm[1],
						Actuator.BiasPrm[2],
						Actuator.Gear, Actuator.ForceMin, Actuator.ForceMax));
			}
		}

		if (!Actuator.bUnsupported)
		{
			++Report.NumTwinActuators;
		}
		Out.Actuators.Add(MoveTemp(Actuator));
	}

	// Actuators that drive a joint, listed on the joint for the runtime force law.
	for (int32 JointIndex = 0; JointIndex < Out.Joints.Num(); ++JointIndex)
	{
		Out.Joints[JointIndex].ActuatorIndices.Reset();
		for (int32 ActuatorIndex = 0; ActuatorIndex < Out.Actuators.Num(); ++ActuatorIndex)
		{
			if (Out.Actuators[ActuatorIndex].JointIndex == JointIndex)
			{
				Out.Joints[JointIndex].ActuatorIndices.Add(ActuatorIndex);
			}
		}
	}

	// --- Equalities --------------------------------------------------------
	// MuJoCo stores eq_data with mjNEQDATA slots per equality. The layouts used here are
	//    connect: data[0:3] anchor in body 1's frame
	//    weld:    data[0:3] anchor in body 1's frame, data[3:6] + data[6:10] the target pose
	//             of body 2's frame expressed in body 1's frame
	constexpr int32 EqDataStride = mjNEQDATA;

	// The twin body frame is the principal/COM frame, which is rotated and offset from the
	// MuJoCo body frame. D converts between the two: mirror(B) = P * D.
	auto FrameOf = [&Out](int32 BodyIndex)
	{
		if (BodyIndex == INDEX_NONE)
		{
			return FTransform::Identity;
		}
		return FTransform(Out.Bodies[BodyIndex].PhysicsFrameWorldRot, Out.Bodies[BodyIndex].PhysicsFrameWorldPos, FVector::OneVector);
	};
	auto MirrorFrameOf = [&Out](int32 BodyIndex)
	{
		if (BodyIndex == INDEX_NONE)
		{
			return FTransform::Identity;
		}
		return FTransform(Out.Bodies[BodyIndex].BodyFrameWorldRot, Out.Bodies[BodyIndex].BodyFrameWorldPos, FVector::OneVector);
	};

	for (int32 EqId = 0; EqId < Model->neq; ++EqId)
	{
		FMjTwinEquality Equality;
		Equality.Name = ElementName(Model, mjOBJ_EQUALITY, EqId, TEXT("equality"));
		// The runtime activation flag lives on mjModel in older releases and on mjData in
		// newer ones; the descriptor only needs "active at rest", so both are consulted.
		Equality.bActive = ReadEqActiveFlag(Model, EqId, 0) && ReadEqActiveFlag(Data, EqId, 0);

		const int32 Body1 = Model->eq_obj1id[EqId];
		const int32 Body2 = Model->eq_obj2id[EqId];
		Equality.Body1Index = Body1 > 0 ? MjBodyToTwinBody[Body1] : INDEX_NONE;
		Equality.Body2Index = Body2 > 0 ? MjBodyToTwinBody[Body2] : INDEX_NONE;

		const FTransform Physics1 = FrameOf(Equality.Body1Index);
		const FTransform Physics2 = FrameOf(Equality.Body2Index);
		const FTransform Mirror1 = MirrorFrameOf(Equality.Body1Index);
		const FTransform Mirror2 = MirrorFrameOf(Equality.Body2Index);
		const FVector LocalAnchor = MjTwinConv::Position(Model->eq_data + EqId * EqDataStride);

		switch (Model->eq_type[EqId])
		{
			case mjEQ_CONNECT:
			{
				Equality.Type = EMjTwinEqualityType::Connect;
				// The anchor is a point on body 1; the same point on body 2 is the one that
				// coincides with it at model definition time, which is the rest pose here.
				Equality.AnchorWorldPos = Mirror1.TransformPosition(LocalAnchor);
				Equality.RelativeTransform = MakeRelativeTransform(Physics2, Physics1);
				Report.Add(FString::Printf(TEXT("equality/%s"), *Equality.Name), EMjTwinFidelity::Approximate,
					TEXT("connect -> Chaos constraint locking the three linear dofs at the anchor (rigid instead of MuJoCo's soft equality)"));
				break;
			}

			case mjEQ_WELD:
			{
				Equality.Type = EMjTwinEqualityType::Weld;
				const FVector TargetPos = MjTwinConv::Position(Model->eq_data + EqId * EqDataStride + 3);
				const FQuat TargetRot = MjTwinConv::Rotation(Model->eq_data + EqId * EqDataStride + 6);

				// Target world pose of body 2's MuJoCo frame is mirror(B1) * (TargetPos, TargetRot).
				// Expressing it in the twin frames gives the relative transform the Chaos
				// constraint has to drive to: X = D1 * R * D2^-1.
				const FTransform D1 = MakeRelativeTransform(Mirror1, Physics1);
				const FTransform D2Inv = MakeRelativeTransform(Physics2, Mirror2);
				Equality.AnchorWorldPos = Mirror1.TransformPosition(LocalAnchor);
				Equality.RelativeTransform = D1 * FTransform(TargetRot, TargetPos, FVector::OneVector) * D2Inv;
				Report.Add(FString::Printf(TEXT("equality/%s"), *Equality.Name), EMjTwinFidelity::Approximate,
					TEXT("weld -> Chaos constraint locking all six dofs at the model's weld pose (rigid instead of MuJoCo's soft weld)"));
				break;
			}

			default:
				Equality.Type = EMjTwinEqualityType::Unsupported;
				Report.Add(FString::Printf(TEXT("equality/%s"), *Equality.Name), EMjTwinFidelity::Unsupported,
					FString::Printf(TEXT("equality type %d (joint, tendon, flex, or distance) has no Chaos equivalent"), Model->eq_type[EqId]));
				break;
		}

		Out.Equalities.Add(MoveTemp(Equality));
	}

	// --- Model features the twin does not reproduce ------------------------
	if (Model->ntendon > 0)
	{
		Report.Add(TEXT("model/tendons"), EMjTwinFidelity::Unsupported,
			FString::Printf(TEXT("%d tendons are not mirrored: tendon routing is MuJoCo-specific"), Model->ntendon));
	}
	if (Model->nflex > 0)
	{
		Report.Add(TEXT("model/deformables"), EMjTwinFidelity::Unsupported,
			FString::Printf(TEXT("%d flex/deformable objects are not mirrored"), Model->nflex));
	}
	if (Model->nsensor > 0)
	{
		Report.Add(TEXT("model/sensors"), EMjTwinFidelity::Unsupported,
			FString::Printf(TEXT("%d sensors are MuJoCo-only; the URLab sensor components keep publishing them"), Model->nsensor));
	}
	if (Model->ncam > 0)
	{
		Report.Add(TEXT("model/cameras"), EMjTwinFidelity::Unsupported,
			FString::Printf(TEXT("%d cameras are MuJoCo-only; use the URLab camera components"), Model->ncam));
	}
	if (Model->nhfield > 0)
	{
		Report.Add(TEXT("model/heightfields"), EMjTwinFidelity::Unsupported,
			FString::Printf(TEXT("%d heightfields are not mirrored (AMjHeightfieldActor covers MuJoCo terrain)"), Model->nhfield));
	}
	if (Model->nplugin > 0)
	{
		Report.Add(TEXT("model/plugins"), EMjTwinFidelity::Unsupported,
			FString::Printf(TEXT("%d engine plugins cannot run inside Unreal"), Model->nplugin));
	}
	if (Model->nkey > 0)
	{
		Report.Add(TEXT("model/keyframes"), EMjTwinFidelity::Approximate,
			FString::Printf(TEXT("%d keyframes are not applied: the twin starts at qpos0, keyframes can be applied at runtime"), Model->nkey));
	}
	if (NumUnsupportedGeoms > 0)
	{
		Report.Add(TEXT("model/collision"), EMjTwinFidelity::Approximate,
			FString::Printf(TEXT("%d of %d geoms have no Chaos collision shape: the twin's collision is incomplete there"),
				NumUnsupportedGeoms, Model->ngeom));
	}

	// --- Local bounds ------------------------------------------------------
	// Per-shape AABB in the owning body's physics frame, used for component bounds.
	for (const FMjTwinShape& Shape : Out.Shapes)
	{
		if (Shape.BodyIndex == INDEX_NONE || Shape.bUnsupported)
		{
			continue;
		}

		FVector HalfExtent = FVector::ZeroVector;
		switch (Shape.Type)
		{
			case EMjTwinShapeType::Box:
			case EMjTwinShapeType::Ellipsoid:
			case EMjTwinShapeType::PlaneSlab:
			case EMjTwinShapeType::Mesh:
				HalfExtent = Shape.Size;
				break;

			case EMjTwinShapeType::Sphere:
				HalfExtent = FVector(Shape.Size.X, Shape.Size.X, Shape.Size.X);
				break;

			case EMjTwinShapeType::Capsule:
				// Capsule axis is Z: the segment stretches Z, the radius covers X/Y.
				HalfExtent = FVector(Shape.Size.X, Shape.Size.X, Shape.Size.Y + Shape.Size.X);
				break;

			case EMjTwinShapeType::Cylinder:
				HalfExtent = FVector(Shape.Size.X, Shape.Size.X, Shape.Size.Y);
				break;

			default:
				break;
		}

		const FQuat Rotation = Shape.LocalTransform.GetRotation();
		const FVector Centre = Shape.LocalTransform.GetTranslation();
		const double Margin = Shape.Margin;
		FBox Box(ForceInit);
		for (int Corner = 0; Corner < 8; ++Corner)
		{
			const FVector Local(
				(Corner & 1) ? HalfExtent.X : -HalfExtent.X,
				(Corner & 2) ? HalfExtent.Y : -HalfExtent.Y,
				(Corner & 4) ? HalfExtent.Z : -HalfExtent.Z);
			Box += Centre + Rotation.RotateVector(Local);
		}
		Box = Box.ExpandBy(Margin);
		Out.Bodies[Shape.BodyIndex].LocalBounds += Box;
	}

	Report.NumTwinBodies = Out.Bodies.Num();
	Report.NumTwinShapes = NumTwinShapes;

	mj_deleteData(Data);
	return true;
}

bool FMjTwinDescriptorBuilder::BuildFromArticulation(AActor* Articulation, FMjTwinDescriptor& Out, FString& OutError)
{
	Out = FMjTwinDescriptor();
	if (!Articulation)
	{
		OutError = TEXT("no articulation");
		return false;
	}

	mjModel* Model = nullptr;
	if (AAMjManager* Manager = AAMjManager::GetManager())
	{
		if (Manager->PhysicsEngine)
		{
			Model = Manager->PhysicsEngine->GetModel();
		}
	}
	if (!Model)
	{
		for (TActorIterator<AAMjManager> It(Articulation->GetWorld()); It; ++It)
		{
			if (It->PhysicsEngine && It->PhysicsEngine->GetModel())
			{
				Model = It->PhysicsEngine->GetModel();
				break;
			}
		}
	}
	if (!Model)
	{
		OutError = TEXT("the MuJoCo model is not compiled yet: compile the manager (or run the simulation once) before building a twin");
		return false;
	}

	// Collect the meshes the MJCF importer created, so mesh geoms get real collision
	// instead of a bounding box. Decomposed hulls are named "<mesh>_<index>".
	TMap<FString, TSoftObjectPtr<UStaticMesh>> MeshAssets;
	auto RegisterMesh = [&MeshAssets](const FString& MeshName, UStaticMesh* Asset)
	{
		if (MeshName.IsEmpty() || !Asset)
		{
			return;
		}
		MeshAssets.FindOrAdd(MeshName) = Asset;

		int32 Underscore = INDEX_NONE;
		if (!MeshName.FindLastChar(TEXT('_'), Underscore) || Underscore <= 0)
		{
			return;
		}

		const FString Suffix = MeshName.Mid(Underscore + 1);
		if (!Suffix.IsNumeric())
		{
			return;
		}

		const FString BaseName = MeshName.Left(Underscore);
		MeshAssets.FindOrAdd(BaseName) = Asset;
		for (int32 Index = 0; Index < 64; ++Index)
		{
			MeshAssets.FindOrAdd(FString::Printf(TEXT("%s_%d"), *BaseName, Index)) = Asset;
		}
	};

	TArray<UMjGeom*> Geoms;
	Articulation->GetComponents<UMjGeom>(Geoms);
	for (UMjGeom* Geom : Geoms)
	{
		if (!Geom)
		{
			continue;
		}

		TArray<USceneComponent*> Children;
		Geom->GetChildrenComponents(true, Children);
		for (USceneComponent* Child : Children)
		{
			UStaticMeshComponent* MeshComponent = Cast<UStaticMeshComponent>(Child);
			if (!MeshComponent || !MeshComponent->GetStaticMesh())
			{
				continue;
			}

			UStaticMesh* Asset = MeshComponent->GetStaticMesh();
			RegisterMesh(Geom->MeshName, Asset);
			RegisterMesh(Asset->GetName(), Asset);
		}
	}

	if (!BuildFromModel(Model, Out, OutError, &MeshAssets))
	{
		return false;
	}

	// Remember the mesh scale per mesh geom, so copied convex hulls keep the right size.
	for (FMjTwinShape& Shape : Out.Shapes)
	{
		if (Shape.Type != EMjTwinShapeType::Mesh || Shape.MjGeomId < 0)
		{
			continue;
		}

		for (UMjGeom* Geom : Geoms)
		{
			if (!Geom || Geom->GetMjID() != Shape.MjGeomId)
			{
				continue;
			}

			TArray<USceneComponent*> Children;
			Geom->GetChildrenComponents(true, Children);
			for (USceneComponent* Child : Children)
			{
				if (const UStaticMeshComponent* MeshComponent = Cast<UStaticMeshComponent>(Child))
				{
					Shape.MeshScale = MeshComponent->GetComponentScale();
					break;
				}
			}
			break;
		}
	}

	return true;
}
