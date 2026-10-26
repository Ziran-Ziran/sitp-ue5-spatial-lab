// Copyright Epic Games, Inc. All Rights Reserved.

#include "VisionProbes.h"

#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GSVSLAM.h"
#include "Materials/MaterialInterface.h"

namespace
{
	constexpr int32 ProbeSeed = 20261025;
}

void VisionProbes::SpawnAround(UWorld* World, APawn* Pawn, int32 Count)
{
	if (!World || !Pawn || Count <= 0)
	{
		return;
	}

	UStaticMesh* CubeMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (!CubeMesh)
	{
		UE_LOG(LogGSVSLAM, Warning, TEXT("VisionProbes: cube mesh not found."));
		return;
	}

	// The world grid material gives each face a fine pattern, so probes contribute
	// many trackable corners instead of four silhouette edges.
	UMaterialInterface* GridMaterial = LoadObject<UMaterialInterface>(
		nullptr, TEXT("/Engine/EngineMaterials/WorldGridMaterial.WorldGridMaterial"));

	const FVector Origin = Pawn->GetActorLocation();
	FRandomStream Random(ProbeSeed);

	for (int32 Index = 0; Index < Count; ++Index)
	{
		const float Angle = Random.FRandRange(0.0f, 2.0f * PI);
		const float Radius = Random.FRandRange(250.0f, 1100.0f);
		const float Height = Random.FRandRange(-100.0f, 150.0f);
		const FVector Location = Origin + FVector(
			FMath::Cos(Angle) * Radius,
			FMath::Sin(Angle) * Radius,
			Height);

		FActorSpawnParameters SpawnParameters;
		SpawnParameters.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		AActor* Probe = World->SpawnActor<AActor>(
			AActor::StaticClass(), Location,
			FRotator(Random.FRandRange(0.0f, 360.0f), Random.FRandRange(0.0f, 360.0f), 0.0f),
			SpawnParameters);
		if (!Probe)
		{
			continue;
		}

		UStaticMeshComponent* Mesh = NewObject<UStaticMeshComponent>(Probe);
		Mesh->SetStaticMesh(CubeMesh);
		Mesh->SetMobility(EComponentMobility::Movable);
		Mesh->RegisterComponent();
		Probe->SetRootComponent(Mesh);
		Probe->SetActorLocation(Location);

		const float Scale = Random.FRandRange(0.25f, 0.9f);
		Mesh->SetWorldScale3D(FVector(Scale));
		Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		if (GridMaterial)
		{
			Mesh->SetMaterial(0, GridMaterial);
		}

		// Collision off: the character must be able to walk its normal route and
		// pass near the probes without being blocked by them.
		Probe->SetActorEnableCollision(false);
	}

	UE_LOG(LogGSVSLAM, Log, TEXT("VisionProbes: spawned %d probes around the pawn."), Count);
}

TArray<FVector> VisionProbes::SpawnLabels(UWorld* World, APawn* Pawn, int32 Count)
{
	TArray<FVector> Positions;
	if (!World || !Pawn || Count <= 0)
	{
		return Positions;
	}

	const FVector Origin = Pawn->GetActorLocation();
	FRandomStream Random(ProbeSeed + 1);
	const TCHAR* Alphabet = TEXT("ABCDEFGHJKLMNPQRSTUVWXYZ23456789");

	// A grid that covers the whole scripted area (routes reach ~20 m ahead and
	// ~6 m sideways), with alternating sides and heights so no single plane owns
	// the features.
	for (int32 Index = 0; Index < Count; ++Index)
	{
		const int32 Column = Index % 12;
		const int32 Row = Index / 12;
		const float Forward = 200.0f + Column * 165.0f;
		const float Side = (Row % 2 == 0) ? 1.0f : -1.0f;
		const float Lateral = Side * (240.0f + (Row % 3) * 110.0f);
		const float Height = 70.0f + (Index % 4) * 40.0f;

		const FVector Location = Origin + FVector(Forward, Lateral, Height);

		FString Label;
		for (int32 Character = 0; Character < 5; ++Character)
		{
			Label.AppendChar(Alphabet[Random.RandRange(0, 30)]);
		}

		// Face the start point. Front-facing text from any point on a route matters:
		// a label seen edge-on or from behind contributes nothing, and that is exactly
		// what killed the first closed-loop run.
		const FRotator Facing = (Origin - Location).Rotation();
		const FRotator YawOnly(0.0f, Facing.Yaw, 0.0f);

		FActorSpawnParameters SpawnParameters;
		SpawnParameters.SpawnCollisionHandlingOverride =
			ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		AActor* Sign = World->SpawnActor<AActor>(AActor::StaticClass(), Location,
			YawOnly, SpawnParameters);
		if (!Sign)
		{
			continue;
		}

		UTextRenderComponent* Text = NewObject<UTextRenderComponent>(Sign);
		Text->SetText(FText::FromString(Label));
		Text->SetWorldSize(55.0f);
		Text->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);
		Text->SetTextRenderColor(FColor(
			static_cast<uint8>(Random.RandRange(64, 255)),
			static_cast<uint8>(Random.RandRange(64, 255)),
			static_cast<uint8>(Random.RandRange(64, 255))));
		Text->RegisterComponent();
		Sign->SetRootComponent(Text);
		Sign->SetActorLocation(Location);
		Sign->SetActorRotation(YawOnly);
		Sign->SetActorEnableCollision(false);

		Positions.Add(Location);
	}

	UE_LOG(LogGSVSLAM, Log, TEXT("VisionProbes: spawned %d unique labels."), Positions.Num());
	return Positions;
}
