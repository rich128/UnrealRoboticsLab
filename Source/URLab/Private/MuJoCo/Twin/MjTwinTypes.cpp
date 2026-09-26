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

#include "MuJoCo/Twin/MjTwinTypes.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace MjTwinNames
{
static const TCHAR* FidelityToString(EMjTwinFidelity Fidelity)
{
	switch (Fidelity)
	{
		case EMjTwinFidelity::Exact:
			return TEXT("Exact");
		case EMjTwinFidelity::Approximate:
			return TEXT("Approximate");
		case EMjTwinFidelity::Unsupported:
			return TEXT("Unsupported");
	}
	return TEXT("Unknown");
}

static int32 FidelityRank(EMjTwinFidelity Fidelity)
{
	// Sorting key: least severe first so the eye lands on the problems last.
	switch (Fidelity)
	{
		case EMjTwinFidelity::Exact:
			return 0;
		case EMjTwinFidelity::Approximate:
			return 1;
		case EMjTwinFidelity::Unsupported:
			return 2;
	}
	return 3;
}
} // namespace MjTwinNames

void FMjTwinReport::Add(const FString& Element, EMjTwinFidelity Fidelity, const FString& Detail)
{
	FMjTwinNote& Note = Notes.Emplace_GetRef();
	Note.Element = Element;
	Note.Fidelity = Fidelity;
	Note.Detail = Detail;
}

int32 FMjTwinReport::CountOf(EMjTwinFidelity Fidelity) const
{
	int32 Count = 0;
	for (const FMjTwinNote& Note : Notes)
	{
		if (Note.Fidelity == Fidelity)
		{
			++Count;
		}
	}
	return Count;
}

bool FMjTwinReport::HasUnsupported() const
{
	return CountOf(EMjTwinFidelity::Unsupported) > 0;
}

FString FMjTwinReport::SummaryLine() const
{
	return FString::Printf(TEXT("%d exact, %d approximate, %d unsupported"),
		CountOf(EMjTwinFidelity::Exact),
		CountOf(EMjTwinFidelity::Approximate),
		CountOf(EMjTwinFidelity::Unsupported));
}

FString FMjTwinReport::ToText() const
{
	TArray<FMjTwinNote> Sorted = Notes;
	Sorted.Sort([](const FMjTwinNote& A, const FMjTwinNote& B) {
		const int32 RankA = MjTwinNames::FidelityRank(A.Fidelity);
		const int32 RankB = MjTwinNames::FidelityRank(B.Fidelity);
		if (RankA != RankB)
		{
			return RankA < RankB;
		}
		return A.Element < B.Element;
	});

	FString Out;
	Out += FString::Printf(TEXT("=== URLab UE physics twin report ===\n"));
	Out += FString::Printf(TEXT("Model: %s\n"), ModelName.IsEmpty() ? TEXT("<unnamed>") : *ModelName);
	Out += FString::Printf(TEXT("Bodies: %d source -> %d rigid bodies   Geoms: %d source -> %d shapes\n"),
		NumSourceBodies, NumTwinBodies, NumGeoms, NumTwinShapes);
	Out += FString::Printf(TEXT("Joints: %d   Actuators: %d source -> %d mirrored\n"),
		NumJoints, NumActuators, NumTwinActuators);
	Out += FString::Printf(TEXT("Fidelity: %s\n"), *SummaryLine());

	for (const FMjTwinNote& Note : Sorted)
	{
		Out += FString::Printf(TEXT("[%s] %s: %s\n"),
			MjTwinNames::FidelityToString(Note.Fidelity), *Note.Element, *Note.Detail);
	}
	return Out;
}

FString FMjTwinReport::ToJson() const
{
	TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
	Root->SetStringField(TEXT("model"), ModelName);
	Root->SetNumberField(TEXT("source_bodies"), NumSourceBodies);
	Root->SetNumberField(TEXT("twin_bodies"), NumTwinBodies);
	Root->SetNumberField(TEXT("geoms"), NumGeoms);
	Root->SetNumberField(TEXT("twin_shapes"), NumTwinShapes);
	Root->SetNumberField(TEXT("joints"), NumJoints);
	Root->SetNumberField(TEXT("actuators"), NumActuators);
	Root->SetNumberField(TEXT("twin_actuators"), NumTwinActuators);
	Root->SetNumberField(TEXT("exact"), CountOf(EMjTwinFidelity::Exact));
	Root->SetNumberField(TEXT("approximate"), CountOf(EMjTwinFidelity::Approximate));
	Root->SetNumberField(TEXT("unsupported"), CountOf(EMjTwinFidelity::Unsupported));

	TArray<TSharedPtr<FJsonValue>> NotesJson;
	NotesJson.Reserve(Notes.Num());
	for (const FMjTwinNote& Note : Notes)
	{
		TSharedRef<FJsonObject> NoteJson = MakeShared<FJsonObject>();
		NoteJson->SetStringField(TEXT("element"), Note.Element);
		NoteJson->SetStringField(TEXT("fidelity"), MjTwinNames::FidelityToString(Note.Fidelity));
		NoteJson->SetStringField(TEXT("detail"), Note.Detail);
		NotesJson.Add(MakeShared<FJsonValueObject>(NoteJson));
	}
	Root->SetArrayField(TEXT("notes"), NotesJson);

	FString Out;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Out);
	FJsonSerializer::Serialize(Root, Writer);
	return Out;
}

FString FMjTwinDivergence::ToText() const
{
	if (!bValid)
	{
		return TEXT("no comparison (twin or MuJoCo model unavailable)");
	}

	FString Out = FString::Printf(TEXT("t=%.3fs  max position error %.3f cm  max orientation error %.3f deg (%d bodies)"),
		SimTime, MaxLinearErrorCm, MaxAngularErrorDeg, Bodies.Num());

	for (const FMjTwinBodyDivergence& Body : Bodies)
	{
		Out += FString::Printf(TEXT("\n  %s: %.3f cm, %.3f deg"), *Body.Body, Body.LinearErrorCm, Body.AngularErrorDeg);
	}
	return Out;
}
