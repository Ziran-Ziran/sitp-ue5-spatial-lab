// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"

class UWorld;
class APawn;

/**
 * Deterministic synthetic texture for pipeline validation.
 *
 * The template map is a grey box: flat walls, tiled floor, a few yellow pillars.
 * Worse than merely sparse, its grid material is periodic, so every local patch
 * looks like every other one. RANSAC then explains a thousand matches with one
 * homography and no usable geometry comes out — the reconstruction registers a
 * handful of images and stops.
 *
 * These signs are unique by construction (random five-character labels from a fixed
 * seed), non-colliding, and spread over the area the scripted routes cover. They are
 * a stand-in for the textured scene the research plan calls S2, not a claim about
 * what a game environment looks like.
 */
namespace VisionProbes
{
	/** Scatters textured cubes around the pawn (legacy proximity texture). */
	void SpawnAround(UWorld* World, APawn* Pawn, int32 Count);

	/** Places unique labels across the route area. Returns their world positions. */
	TArray<FVector> SpawnLabels(UWorld* World, APawn* Pawn, int32 Count);
}
