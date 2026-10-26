// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "TimerManager.h"
#include "VisionRecorderSubsystem.generated.h"

class UVisionRecorderComponent;

/**
 * Attaches the recorder to the local player pawn and exposes console commands.
 *
 * The template project's character is a Blueprint, so the component is added at
 * runtime instead of editing that asset. F9 starts, F10 stops; the console
 * commands VisionRec.Start / VisionRec.Stop / VisionRec.Status do the same.
 */
UCLASS()
class GSVSLAM_API UVisionRecorderSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;

	UVisionRecorderComponent* GetRecorder() const { return Recorder.Get(); }

private:
	void TryAttach();
	void StartAfterWarmup(FTimerDelegate Start);

	TWeakObjectPtr<UVisionRecorderComponent> Recorder;
	FTimerHandle AttachTimer;
	FTimerHandle WarmupTimer;
	float WarmupSeconds = 5.0f;
	FString SessionNameOverride;
	bool bKeysBound = false;

	/** -VisionSmoke=<seconds> records a fixed run and exits; used for unattended checks. */
	bool bSmokeMode = false;
	float SmokeSeconds = 0.0f;

	/** -VisionAuto=<seconds> records a fixed run but keeps the game running. */
	bool bAutoMode = false;
	float AutoSeconds = 0.0f;

	/** -VisionRoute=circle|line|weave drives the pawn on a reproducible route. */
	FString RouteName;

	/** -VisionScene=city generates a procedural block instead of using the template map. */
	FString SceneName;
	float RouteSpeed = 150.0f;
	/** -VisionLaps=<n> repeats the route; more than one lap is what makes drift matter. */
	int32 RouteLaps = 1;
	bool bQuitWhenDone = false;
};
