// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class UWorld;
class APawn;

/**
 * Procedurally builds a small city block far from the template courtyard.
 *
 * The template map is a grey box twenty metres across; it cannot express occlusion,
 * real parallax at different depths, or a closed loop. This generator creates a block
 * of buildings around a street loop, with obstacles on the kerb and unique labels on
 * the facades facing the street. Everything is seeded, so every run builds the same
 * place.
 *
 * The point is not visual beauty. It is a scene where depth varies between 2 m and
 * 40 m, where buildings hide each other, and where a route can come back to its own
 * start - the conditions under which loop closure and drift actually matter.
 */
namespace VisionScene
{
	struct FCityLayout
	{
		FVector Origin = FVector::ZeroVector;   // street corner, ground level
		float Length = 1400.0f;                 // along +X
		float Width = 700.0f;                   // along +Y
		float Street = 450.0f;                  // half-width of the street
		int32 Buildings = 0;
		int32 Labels = 0;
		TArray<FVector> LabelPositions;
	};

	/** Builds the block, moves the pawn to the start corner and returns the layout. */
	FCityLayout BuildCityBlock(UWorld* World, APawn* Pawn, int32 Seed, int32 LabelCount, bool bWindows = true);

	/** Closed loop around the block, in world space. */
	TArray<FVector> LoopWaypoints(const FCityLayout& Layout);
}
