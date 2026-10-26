// Copyright Epic Games, Inc. All Rights Reserved.

#include "RouteDriverComponent.h"

#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "GSVSLAM.h"

URouteDriverComponent::URouteDriverComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
	bAutoActivate = true;
}

void URouteDriverComponent::BeginPlay()
{
	Super::BeginPlay();
	SetComponentTickEnabled(false);
}

void URouteDriverComponent::BuildWaypoints(const FString& RouteName)
{
	Waypoints.Reset();

	if (RouteName == TEXT("line"))
	{
		Waypoints.Add(Origin + FVector(1400.0f, 0.0f, 0.0f));
	}
	else if (RouteName == TEXT("weave"))
	{
		// Lateral sinusoid: forces the camera to rotate, so rotation is observable.
		constexpr int32 Steps = 10;
		for (int32 Step = 1; Step <= Steps; ++Step)
		{
			const float Forward = 150.0f * Step;
			const float Lateral = 280.0f * FMath::Sin(static_cast<float>(Step) / Steps * 2.0f * PI);
			Waypoints.Add(Origin + FVector(Forward, Lateral, 0.0f));
		}
	}
	else if (RouteName == TEXT("loop") || RouteName == TEXT("square"))
	{
		// Closed rectangle: comes back to the start, which also gives the estimator a
		// loop-closure opportunity and a non-degenerate rotation for evaluation.
		const float Width = RouteName == TEXT("loop") ? 800.0f : 700.0f;
		const float Depth = RouteName == TEXT("loop") ? 300.0f : 700.0f;
		// The open, textured side of the courtyard is -Y; +Y runs into a large
		// featureless wall that no visual odometry can track.
		Waypoints.Add(Origin + FVector(Width, 0.0f, 0.0f));
		Waypoints.Add(Origin + FVector(Width, -Depth, 0.0f));
		Waypoints.Add(Origin + FVector(0.0f, -Depth, 0.0f));
		Waypoints.Add(Origin + FVector(0.0f, 0.0f, 0.0f));
	}
	else if (RouteName == TEXT("circle"))
	{
		bHeadingMode = true;
	}
}

bool URouteDriverComponent::StartRouteWithWaypoints(const TArray<FVector>& InWaypoints, float SpeedCmPerSecond)
{
	ACharacter* Character = Cast<ACharacter>(GetOwner());
	if (!Character || InWaypoints.Num() == 0)
	{
		return false;
	}

	Route = TEXT("waypoints");
	Speed = FMath::Clamp(SpeedCmPerSecond, 20.0f, 600.0f);
	Elapsed = 0.0f;
	Heading = 0.0f;
	CurrentYaw = 0.0f;
	CurrentWaypoint = 0;
	bHeadingMode = false;
	Waypoints = InWaypoints;
	Origin = Character->GetActorLocation();

	if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
	{
		Movement->MaxWalkSpeed = Speed;
		Movement->bOrientRotationToMovement = false;
	}

	float Length = 0.0f;
	FVector Previous = Origin;
	for (const FVector& Waypoint : Waypoints)
	{
		Length += FVector::Dist2D(Previous, Waypoint);
		Previous = Waypoint;
	}
	DurationSeconds = Length / Speed + 2.0f;

	bActive = true;
	SetComponentTickEnabled(true);
	UE_LOG(LogGSVSLAM, Log, TEXT("RouteDriver: %s, %.0f m at %.0f cm/s, %.1f s."),
		*Route, Length / 100.0f, Speed, DurationSeconds);
	return true;
}

bool URouteDriverComponent::StartRoute(const FString& RouteName, float SpeedCmPerSecond)
{
	ACharacter* Character = Cast<ACharacter>(GetOwner());
	if (!Character)
	{
		UE_LOG(LogGSVSLAM, Error, TEXT("RouteDriver: owner is not a character."));
		return false;
	}

	Route = RouteName.ToLower();
	Speed = FMath::Clamp(SpeedCmPerSecond, 20.0f, 600.0f);
	Elapsed = 0.0f;
	Heading = 0.0f;
	CurrentYaw = 0.0f;
	CurrentWaypoint = 0;
	bHeadingMode = false;
	Origin = Character->GetActorLocation();

	if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
	{
		Movement->MaxWalkSpeed = Speed;
		Movement->bOrientRotationToMovement = false;
	}

	BuildWaypoints(Route);
	if (!bHeadingMode && Waypoints.Num() == 0)
	{
		UE_LOG(LogGSVSLAM, Warning, TEXT("RouteDriver: unknown route '%s'."), *RouteName);
		return false;
	}

	// Duration is the path length divided by speed, plus a small allowance.
	float Length = 0.0f;
	FVector Previous = Origin;
	for (const FVector& Waypoint : Waypoints)
	{
		Length += FVector::Dist(Previous, Waypoint);
		Previous = Waypoint;
	}
	DurationSeconds = bHeadingMode ? 20.0f : FMath::Max(2.0f, Length / Speed + 1.0f);

	bActive = true;
	SetComponentTickEnabled(true);
	UE_LOG(LogGSVSLAM, Log, TEXT("RouteDriver: %s, %.0f m at %.0f cm/s, %.1f s."),
		*Route, Length / 100.0f, Speed, DurationSeconds);
	return true;
}

bool URouteDriverComponent::StartRouteLaps(const FString& RouteName, float SpeedCmPerSecond, int32 Laps)
{
	if (!StartRoute(RouteName, SpeedCmPerSecond))
	{
		return false;
	}
	if (Laps <= 1)
	{
		return true;
	}

	// Every named route except "circle" ends back at the start, so repeating the
	// waypoint list is a genuine multi-lap closed route.
	const TArray<FVector> OneLap = Waypoints;
	for (int32 Lap = 1; Lap < Laps; ++Lap)
	{
		Waypoints.Append(OneLap);
	}

	float Length = 0.0f;
	FVector Previous = Origin;
	for (const FVector& Waypoint : Waypoints)
	{
		Length += FVector::Dist(Previous, Waypoint);
		Previous = Waypoint;
	}
	DurationSeconds = FMath::Max(2.0f, Length / Speed + 1.0f);

	UE_LOG(LogGSVSLAM, Log, TEXT("RouteDriver: %s x%d laps, %.0f m at %.0f cm/s, %.1f s."),
		*Route, Laps, Length / 100.0f, Speed, DurationSeconds);
	return true;
}

void URouteDriverComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bActive)
	{
		return;
	}

	ACharacter* Character = Cast<ACharacter>(GetOwner());
	if (!Character)
	{
		bActive = false;
		return;
	}

	Elapsed += DeltaTime;

	float DesiredYaw = FMath::RadiansToDegrees(Heading);

	if (bHeadingMode)
	{
		// Constant yaw rate: a circle of radius speed / omega.
		Heading += 0.5f * DeltaTime;
		DesiredYaw = FMath::RadiansToDegrees(Heading);
	}
	else if (CurrentWaypoint < Waypoints.Num())
	{
		const FVector ToWaypoint = Waypoints[CurrentWaypoint] - Character->GetActorLocation();
		DesiredYaw = FMath::RadiansToDegrees(FMath::Atan2(ToWaypoint.Y, ToWaypoint.X));
		if (ToWaypoint.Size2D() < 40.0f)
		{
			++CurrentWaypoint;
		}
	}
	else
	{
		bActive = false;
		SetComponentTickEnabled(false);
		if (UCharacterMovementComponent* Movement = Character->GetCharacterMovement())
		{
			Movement->StopMovementImmediately();
		}
		UE_LOG(LogGSVSLAM, Log, TEXT("RouteDriver: route finished after %.1fs."), Elapsed);
		return;
	}

	const FRotator Desired(0.0f, DesiredYaw, 0.0f);

	// Turning instantaneously would be a 90 degree jump between two frames, which is
	// both unrealistic and a hard case for any odometry. Cap the turn rate.
	constexpr float MaxTurnRateDegrees = 90.0f;
	float DeltaYaw = FMath::FindDeltaAngleDegrees(CurrentYaw, DesiredYaw);
	const float MaxStep = MaxTurnRateDegrees * DeltaTime;
	DeltaYaw = FMath::Clamp(DeltaYaw, -MaxStep, MaxStep);
	CurrentYaw = FMath::UnwindDegrees(CurrentYaw + DeltaYaw);

	const FRotator Smoothed(0.0f, CurrentYaw, 0.0f);
	if (APlayerController* PlayerController = Cast<APlayerController>(Character->GetController()))
	{
		// The template character follows control yaw, so set both to be sure.
		PlayerController->SetControlRotation(Smoothed);
	}
	Character->SetActorRotation(Smoothed);
	Character->AddMovementInput(Character->GetActorForwardVector(), 1.0f);
}
