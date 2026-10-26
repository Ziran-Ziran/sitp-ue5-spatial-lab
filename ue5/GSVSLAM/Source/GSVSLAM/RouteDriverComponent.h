// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RouteDriverComponent.generated.h"

/**
 * Drives the player pawn along one fixed route while recording.
 *
 * Routes exist to validate the recording and SLAM pipeline without a human at the
 * keyboard: same map, same start, same speed, identical every run. They do not
 * replace the human sessions; they are the "known input" case.
 */
UCLASS()
class GSVSLAM_API URouteDriverComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	URouteDriverComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** line, weave, loop, square, circle. Returns false for an unknown name. */
	bool StartRoute(const FString& RouteName, float SpeedCmPerSecond);

	/**
	 * Same as StartRoute, repeated Laps times.
	 *
	 * One lap is a thirty-metre walk that any estimator survives. Several laps are
	 * the case loop closure exists for: the camera comes back to the start and the
	 * drift accumulated over the first lap has to be corrected by the picture, not
	 * by luck.
	 */
	bool StartRouteLaps(const FString& RouteName, float SpeedCmPerSecond, int32 Laps);

	/** Follow explicit world-space waypoints (used by the generated city block). */
	bool StartRouteWithWaypoints(const TArray<FVector>& InWaypoints, float SpeedCmPerSecond);

	bool IsActive() const { return bActive; }
	float GetDurationSeconds() const { return DurationSeconds; }

	/** World-space route the pawn will follow, for placing markers along it. */
	const TArray<FVector>& GetWaypoints() const { return Waypoints; }

private:
	void BuildWaypoints(const FString& RouteName);

	FString Route;
	FVector Origin = FVector::ZeroVector;
	TArray<FVector> Waypoints;
	int32 CurrentWaypoint = 0;
	float Elapsed = 0.0f;
	float Speed = 200.0f;
	float DurationSeconds = 12.0f;
	float Heading = 0.0f;
	float CurrentYaw = 0.0f;
	bool bActive = false;
	bool bHeadingMode = false;
};
