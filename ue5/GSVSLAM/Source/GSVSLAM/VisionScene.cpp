// Copyright Epic Games, Inc. All Rights Reserved.

#include "VisionScene.h"

#include "Components/CapsuleComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/TextRenderComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GSVSLAM.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"

namespace
{
	constexpr float GroundThickness = 200.0f;

	AActor* SpawnBox(UWorld* World, UStaticMesh* Mesh, UMaterialInterface* Material,
		const FVector& Centre, const FVector& Size, float Yaw, const FLinearColor& Colour)
	{
		FActorSpawnParameters Parameters;
		Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		AActor* Actor = World->SpawnActor<AActor>(AActor::StaticClass(), Centre,
			FRotator(0.0f, Yaw, 0.0f), Parameters);
		if (!Actor)
		{
			return nullptr;
		}

		UStaticMeshComponent* Piece = NewObject<UStaticMeshComponent>(Actor);
		Piece->SetStaticMesh(Mesh);
		Piece->SetMobility(EComponentMobility::Movable);
		Piece->RegisterComponent();
		Actor->SetRootComponent(Piece);
		Actor->SetActorLocation(Centre);
		// The engine cube is 100 cm on a side, so scale in metres / 100.
		Piece->SetWorldScale3D(Size / 100.0f);

		if (Material)
		{
			if (UMaterialInstanceDynamic* Dynamic = UMaterialInstanceDynamic::Create(Material, Piece))
			{
				Dynamic->SetVectorParameterValue(TEXT("Color"), Colour);
				Piece->SetMaterial(0, Dynamic);
			}
			else
			{
				Piece->SetMaterial(0, Material);
			}
		}
		return Actor;
	}

	void SpawnLabel(UWorld* World, const FVector& Location, const FRotator& Rotation,
		const FString& Text, float WorldSize, const FColor& Colour)
	{
		FActorSpawnParameters Parameters;
		Parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		AActor* Sign = World->SpawnActor<AActor>(AActor::StaticClass(), Location, Rotation, Parameters);
		if (!Sign)
		{
			return;
		}
		UTextRenderComponent* Label = NewObject<UTextRenderComponent>(Sign);
		Label->SetText(FText::FromString(Text));
		Label->SetWorldSize(WorldSize);
		Label->SetHorizontalAlignment(EHorizTextAligment::EHTA_Center);
		Label->SetTextRenderColor(Colour);
		Label->RegisterComponent();
		Sign->SetRootComponent(Label);
		Sign->SetActorLocation(Location);
		Sign->SetActorRotation(Rotation);
		Sign->SetActorEnableCollision(false);
	}

	FString RandomLabel(FRandomStream& Random)
	{
		static const TCHAR* Alphabet = TEXT("ABCDEFGHJKLMNPQRSTUVWXYZ23456789");
		FString Label;
		for (int32 Index = 0; Index < 5; ++Index)
		{
			Label.AppendChar(Alphabet[Random.RandRange(0, 30)]);
		}
		return Label;
	}
}

VisionScene::FCityLayout VisionScene::BuildCityBlock(UWorld* World, APawn* Pawn, int32 Seed, int32 LabelCount, bool bWindows)
{
	FCityLayout Layout;
	if (!World || !Pawn)
	{
		return Layout;
	}

	UStaticMesh* Cube = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
	UStaticMesh* Cylinder = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	UMaterialInterface* BaseMaterial = LoadObject<UMaterialInterface>(
		nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
	if (!Cube)
	{
		UE_LOG(LogGSVSLAM, Error, TEXT("VisionScene: cube mesh missing."));
		return Layout;
	}

	// Far from the template courtyard: its walls and pillars must not appear in view.
	Layout.Origin = FVector(20000.0f, 20000.0f, 0.0f);
	const FVector Origin = Layout.Origin;
	FRandomStream Random(Seed);

	// Ground slab, large enough that no edge is visible from the route.
	SpawnBox(World, Cube, BaseMaterial,
		Origin + FVector(Layout.Length * 0.5f, Layout.Width * 0.5f, -GroundThickness * 0.5f),
		FVector(9000.0f, 9000.0f, GroundThickness), 0.0f, FLinearColor(0.10f, 0.11f, 0.12f));

	TArray<FVector> FacadeLabels;
	TArray<FRotator> FacadeRotations;
	TArray<FVector> BigLabels;
	TArray<FRotator> BigLabelRotations;

	// Four rows of buildings, each row faces the street loop.
	struct FRow { FVector Start; FVector Direction; FVector Outward; float Length; };
	const FRow Rows[] = {
		{ Origin + FVector(0.0f, Layout.Street, 0.0f),              FVector(1, 0, 0),  FVector(0, 1, 0),  Layout.Length },
		{ Origin + FVector(Layout.Length, Layout.Width + Layout.Street, 0.0f), FVector(-1, 0, 0), FVector(0, 1, 0), Layout.Length },
		{ Origin + FVector(Layout.Street, Layout.Width, 0.0f),      FVector(0, 1, 0),  FVector(-1, 0, 0), Layout.Width },
		{ Origin + FVector(Layout.Length + Layout.Street, 0.0f, 0.0f), FVector(0, 1, 0), FVector(1, 0, 0), Layout.Width },
	};

	for (const FRow& Row : Rows)
	{
		float Travelled = 0.0f;
		while (Travelled < Row.Length)
		{
			const float Width = Random.FRandRange(350.0f, 700.0f);
			const float Depth = Random.FRandRange(600.0f, 1100.0f);
			const float Height = Random.FRandRange(600.0f, 1600.0f);
			const float Gap = Random.FRandRange(60.0f, 220.0f);

			if (Random.FRand() < 0.12f)
			{
				// Occasional gap: alleys make the skyline irregular and create occlusion.
				Travelled += Width * 0.6f;
				continue;
			}

			const FVector Along = Row.Direction * (Travelled + Width * 0.5f);
			const FVector Out = Row.Outward * (Layout.Street + Depth * 0.5f);
			const FVector Centre = Row.Start + Along + Out + FVector(0, 0, Height * 0.5f);

			const FLinearColor Colour = FLinearColor::MakeFromHSV8(
				static_cast<uint8>(Random.RandRange(0, 255)), 190, static_cast<uint8>(Random.RandRange(70, 165)));

			SpawnBox(World, Cube, BaseMaterial, Centre, FVector(Width, Depth, Height),
				FRandomStream(Random.RandRange(0, 0)).FRandRange(0.0f, 0.0f), Colour);
			++Layout.Buildings;

			// Window grid on the street-facing facade. Real buildings are trackable
			// because of this kind of regular structure; without it each facade is one
			// featureless plane, which is exactly what broke the earlier scene.
			// -VisionWindows=0 removes them, which is the control condition for
			// "repetitive structure versus unique labels".
			if (bWindows)
			{
				// Vary the grid per building. A uniform grid makes every facade match
				// every other facade, and a closed loop then "closes" onto the wrong
				// side of the block and collapses the trajectory.
				const int32 Columns = FMath::Max(2, FMath::FloorToInt(Width / Random.FRandRange(150.0f, 280.0f)));
				const int32 WindowRows = FMath::Max(2, FMath::FloorToInt((Height - 140.0f) / Random.FRandRange(190.0f, 290.0f)));
				const float CellWidth = Width / Columns;
				const float CellHeight = (Height - 140.0f) / WindowRows;
				const bool bAlongX = FMath::Abs(Row.Direction.X) > 0.5f;
				const float Fill = Random.FRandRange(0.40f, 0.68f);
				const FVector WindowSize = bAlongX
					? FVector(CellWidth * Fill, 14.0f, CellHeight * 0.5f)
					: FVector(14.0f, CellWidth * Fill, CellHeight * 0.5f);
				const FLinearColor Glass = FLinearColor(
					Random.FRandRange(0.02f, 0.10f),
					Random.FRandRange(0.03f, 0.12f),
					Random.FRandRange(0.06f, 0.22f));

				for (int32 RowIndex = 0; RowIndex < WindowRows; ++RowIndex)
				{
					for (int32 Column = 0; Column < Columns; ++Column)
					{
						const float OffsetAlong = -Width * 0.5f + CellWidth * (Column + 0.5f);
						const float OffsetUp = 100.0f + CellHeight * (RowIndex + 0.5f);
						const FVector WindowCentre = Centre
							+ Row.Direction * OffsetAlong
							- Row.Outward * (Depth * 0.5f - 6.0f)
							+ FVector(0.0f, 0.0f, OffsetUp - Height * 0.5f);
						SpawnBox(World, Cube, BaseMaterial, WindowCentre, WindowSize, 0.0f, Glass);
					}
				}
			}

			// One large label high on every facade. Unlike the small ones it is legible
			// from across the block, so the loop has a unique thing to close on.
			{
				const FVector BigLabelPoint = Row.Start + Along
					+ Row.Outward * (Layout.Street - 10.0f)
					+ FVector(0.0f, 0.0f, FMath::Max(420.0f, Height * 0.55f));
				BigLabels.Add(BigLabelPoint);
				BigLabelRotations.Add((-Row.Outward).Rotation());
			}

			// Two labels per facade, at eye level and above.
			const FVector FacadePoint = Row.Start + Along + Row.Outward * (Layout.Street - 8.0f);
			const FRotator Facing = (-Row.Outward).Rotation();
			FacadeLabels.Add(FacadePoint + FVector(0, 0, Random.FRandRange(140.0f, 260.0f)));
			FacadeRotations.Add(Facing);
			FacadeLabels.Add(FacadePoint + FVector(0, 0, Random.FRandRange(320.0f, 520.0f)));
			FacadeRotations.Add(Facing);

			Travelled += Width + Gap;
		}
	}

	// Kerbside clutter: changes silhouettes and creates near/far depth mixture.
	for (int32 Index = 0; Index < 26; ++Index)
	{
		const float AlongX = Random.FRandRange(-300.0f, Layout.Length + 300.0f);
		const float AlongY = Random.FRandRange(-300.0f, Layout.Width + 300.0f);
		const bool bOnHorizontal = Random.FRand() < 0.5f;
		const FVector Location = bOnHorizontal
			? Origin + FVector(AlongX, Random.FRand() < 0.5f ? -Layout.Street * 0.6f : Layout.Width + Layout.Street * 0.6f, 0.0f)
			: Origin + FVector(Random.FRand() < 0.5f ? -Layout.Street * 0.6f : Layout.Length + Layout.Street * 0.6f, AlongY, 0.0f);
		const float Side = Random.FRandRange(25.0f, 55.0f);
		const float Height = Random.FRandRange(40.0f, 110.0f);
		SpawnBox(World, Random.FRand() < 0.25f ? Cylinder : Cube, BaseMaterial,
			Location + FVector(0, 0, Height * 0.5f), FVector(Side, Side, Height), Random.FRandRange(0.0f, 360.0f),
			FLinearColor(0.6f, 0.45f, 0.2f));
	}

	// Large unique label per building first: these are what a loop closure can use.
	for (int32 Index = 0; Index < BigLabels.Num(); ++Index)
	{
		SpawnLabel(World, BigLabels[Index], BigLabelRotations[Index], RandomLabel(Random), 170.0f,
			FColor(static_cast<uint8>(Random.RandRange(140, 255)),
				static_cast<uint8>(Random.RandRange(140, 255)),
				static_cast<uint8>(Random.RandRange(140, 255))));
		Layout.LabelPositions.Add(BigLabels[Index]);
		++Layout.Labels;
	}

	// Labels: unique text on the facades, plus a few free-standing ones so the street
	// itself is trackable when a building wall fills the view.
	const int32 FacadeCount = FMath::Min(LabelCount, FacadeLabels.Num());
	for (int32 Index = 0; Index < FacadeCount; ++Index)
	{
		SpawnLabel(World, FacadeLabels[Index], FacadeRotations[Index], RandomLabel(Random), 55.0f,
			FColor(static_cast<uint8>(Random.RandRange(80, 255)),
				static_cast<uint8>(Random.RandRange(80, 255)),
				static_cast<uint8>(Random.RandRange(80, 255))));
		Layout.LabelPositions.Add(FacadeLabels[Index]);
		++Layout.Labels;
	}

	// Free-standing labels along the street centre line, alternating sides.
	for (int32 Index = 0; Index < 24; ++Index)
	{
		const float Angle = static_cast<float>(Index) / 24.0f * 2.0f * PI;
		const FVector OnLoop(
			Layout.Length * 0.5f + FMath::Cos(Angle) * (Layout.Length * 0.5f),
			Layout.Width * 0.5f + FMath::Sin(Angle) * (Layout.Width * 0.5f),
			0.0f);
		const FVector Offset = FVector(FMath::Cos(Angle), FMath::Sin(Angle), 0.0f).GetSafeNormal() * 220.0f;
		const FVector Location = Origin + OnLoop + Offset + FVector(0, 0, Random.FRandRange(60.0f, 300.0f));
		const FRotator Facing = (Origin + OnLoop - Location).Rotation();
		SpawnLabel(World, Location, FRotator(0.0f, Facing.Yaw, 0.0f), RandomLabel(Random), 50.0f,
			FColor(static_cast<uint8>(Random.RandRange(80, 255)),
				static_cast<uint8>(Random.RandRange(80, 255)),
				static_cast<uint8>(Random.RandRange(80, 255))));
		Layout.LabelPositions.Add(Location);
		++Layout.Labels;
	}

	// Corner billboards: 4.5 m tall, unique, facing the turn. Loop closure needs
	// something that is still readable from across the block; a 55 cm label is about
	// four pixels at twenty metres, which is why every earlier loop attempt failed.
	{
		struct FCorner { FVector Point; FVector Outward; };
		const FCorner Corners[] = {
			{ Origin, FVector(-1.0f, -1.0f, 0.0f).GetSafeNormal() },
			{ Origin + FVector(Layout.Length, 0.0f, 0.0f), FVector(1.0f, -1.0f, 0.0f).GetSafeNormal() },
			{ Origin + FVector(Layout.Length, Layout.Width, 0.0f), FVector(1.0f, 1.0f, 0.0f).GetSafeNormal() },
			{ Origin + FVector(0.0f, Layout.Width, 0.0f), FVector(-1.0f, 1.0f, 0.0f).GetSafeNormal() },
		};

		for (const FCorner& Corner : Corners)
		{
			// Two boards per corner, side by side: one is enough geometrically, two give
			// a parallax pair that survives a bad angle.
			for (int32 Side = 0; Side < 2; ++Side)
			{
				const FVector Lateral = FVector(-Corner.Outward.Y, Corner.Outward.X, 0.0f);
				const FVector Location = Corner.Point + Corner.Outward * 300.0f
					+ Lateral * (Side == 0 ? -140.0f : 140.0f)
					+ FVector(0.0f, 0.0f, 260.0f);
				const FRotator Facing = (-Corner.Outward).Rotation();
				SpawnLabel(World, Location, FRotator(0.0f, Facing.Yaw, 0.0f), RandomLabel(Random), 450.0f,
					FColor(250, 250, 250));
				Layout.LabelPositions.Add(Location);
				++Layout.Labels;
			}
		}
	}
	// Put the pawn on the ground at the start corner, facing along +X.
	if (ACharacter* Character = Cast<ACharacter>(Pawn))
	{
		const float HalfHeight = Character->GetCapsuleComponent()
			? Character->GetCapsuleComponent()->GetScaledCapsuleHalfHeight() : 88.0f;
		Character->SetActorLocation(Origin + FVector(0.0f, 0.0f, HalfHeight + 2.0f), false, nullptr,
			ETeleportType::TeleportPhysics);
		Character->SetActorRotation(FRotator(0.0f, 0.0f, 0.0f));
	}

	UE_LOG(LogGSVSLAM, Log, TEXT("VisionScene: city block %d buildings, %d labels."),
		Layout.Buildings, Layout.Labels);
	return Layout;
}

TArray<FVector> VisionScene::LoopWaypoints(const FCityLayout& Layout)
{
	TArray<FVector> Points;
	Points.Add(Layout.Origin + FVector(Layout.Length, 0.0f, 0.0f));
	Points.Add(Layout.Origin + FVector(Layout.Length, Layout.Width, 0.0f));
	Points.Add(Layout.Origin + FVector(0.0f, Layout.Width, 0.0f));
	Points.Add(Layout.Origin + FVector(0.0f, 0.0f, 0.0f));
	return Points;
}
