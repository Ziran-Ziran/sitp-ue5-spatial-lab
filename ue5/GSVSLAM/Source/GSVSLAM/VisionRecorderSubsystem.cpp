// Copyright Epic Games, Inc. All Rights Reserved.

#include "VisionRecorderSubsystem.h"

#include "Camera/CameraComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "GSVSLAM.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformMisc.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "TimerManager.h"
#include "RouteDriverComponent.h"
#include "VisionProbes.h"
#include "VisionScene.h"
#include "VisionRecorderComponent.h"

bool UVisionRecorderSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
	const UWorld* World = Cast<UWorld>(Outer);
	return World && World->IsGameWorld();
}

void UVisionRecorderSubsystem::OnWorldBeginPlay(UWorld& InWorld)
{
	Super::OnWorldBeginPlay(InWorld);

	// Unattended verification: record for N seconds, then quit.
	float Seconds = 0.0f;
	if (FParse::Value(FCommandLine::Get(), TEXT("VisionSmoke="), Seconds) && Seconds > 0.0f)
	{
		bSmokeMode = true;
		SmokeSeconds = Seconds;
	}

	// Human session with a guaranteed start and stop, independent of key bindings.
	if (FParse::Value(FCommandLine::Get(), TEXT("VisionAuto="), Seconds) && Seconds > 0.0f)
	{
		bAutoMode = true;
		AutoSeconds = Seconds;
	}

	FParse::Value(FCommandLine::Get(), TEXT("VisionWarmup="), WarmupSeconds);
	WarmupSeconds = FMath::Clamp(WarmupSeconds, 0.0f, 120.0f);
	FParse::Value(FCommandLine::Get(), TEXT("VisionSession="), SessionNameOverride);
	FParse::Value(FCommandLine::Get(), TEXT("VisionRoute="), RouteName);
	FParse::Value(FCommandLine::Get(), TEXT("VisionScene="), SceneName);
	FParse::Value(FCommandLine::Get(), TEXT("VisionRouteSpeed="), RouteSpeed);
	FParse::Value(FCommandLine::Get(), TEXT("VisionLaps="), RouteLaps);
	RouteLaps = FMath::Max(1, RouteLaps);
	bQuitWhenDone = FParse::Param(FCommandLine::Get(), TEXT("VisionQuit"));

	// The pawn may be spawned after world begin play, so retry briefly.
	InWorld.GetTimerManager().SetTimer(
		AttachTimer,
		FTimerDelegate::CreateUObject(this, &UVisionRecorderSubsystem::TryAttach),
		0.25f, true, 0.0f);
}

void UVisionRecorderSubsystem::TryAttach()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	APlayerController* PlayerController = World->GetFirstPlayerController();
	APawn* Pawn = PlayerController ? PlayerController->GetPawn() : nullptr;
	UCameraComponent* Camera = Pawn ? Pawn->FindComponentByClass<UCameraComponent>() : nullptr;
	if (!Pawn || !Camera)
	{
		return;
	}

	UVisionRecorderComponent* Component = Pawn->FindComponentByClass<UVisionRecorderComponent>();
	if (!Component)
	{
		Component = NewObject<UVisionRecorderComponent>(Pawn, TEXT("VisionRecorder"));
		Component->RegisterComponent();
	}
	Component->AttachToCamera(Camera);
	Recorder = Component;

	if (!bKeysBound && PlayerController->InputComponent)
	{
		PlayerController->InputComponent->BindKey(
			EKeys::F9, IE_Pressed, Component, &UVisionRecorderComponent::StartRecordingDefault);
		PlayerController->InputComponent->BindKey(
			EKeys::F10, IE_Pressed, Component, &UVisionRecorderComponent::StopRecording);
		bKeysBound = true;
	}

	World->GetTimerManager().ClearTimer(AttachTimer);
	UE_LOG(LogGSVSLAM, Log, TEXT("VisionRecorder attached to %s. F9 start, F10 stop."), *Pawn->GetName());

	if (bSmokeMode)
	{
		StartAfterWarmup(FTimerDelegate::CreateWeakLambda(this, [this, Component]()
		{
			UWorld* World = GetWorld();
			Component->StartRecording(SessionNameOverride.IsEmpty() ? TEXT("smoke") : SessionNameOverride);
			FTimerHandle SmokeTimer;
			World->GetTimerManager().SetTimer(SmokeTimer, FTimerDelegate::CreateLambda([this]()
			{
				if (Recorder.IsValid())
				{
					Recorder->StopRecording();
				}
				UE_LOG(LogGSVSLAM, Log, TEXT("SMOKE COMPLETE"));
				FPlatformMisc::RequestExit(false);
			}), SmokeSeconds, false);
		}));
	}
	else if (bAutoMode)
	{
		StartAfterWarmup(FTimerDelegate::CreateWeakLambda(this, [this, Component]()
		{
			UWorld* World = GetWorld();
			Component->StartRecording(SessionNameOverride);
			FTimerHandle AutoTimer;
			World->GetTimerManager().SetTimer(AutoTimer, FTimerDelegate::CreateLambda([this]()
			{
				if (Recorder.IsValid())
				{
					Recorder->StopRecording();
					UE_LOG(LogGSVSLAM, Log, TEXT("AUTO SESSION COMPLETE"));
				}
			}), AutoSeconds, false);
		}));
	}
	else if (!RouteName.IsEmpty() || !SceneName.IsEmpty())
	{
		TArray<FVector> Landmarks;
		TArray<FVector> Waypoints;
		FString SessionLabel = RouteName;
		float Speed = RouteSpeed;

		if (SceneName == TEXT("city"))
		{
			// A block of buildings with a street loop: occlusion, mixed depths and a
			// closed route, none of which the template courtyard can offer.
			int32 WindowsOn = 1;
			FParse::Value(FCommandLine::Get(), TEXT("VisionWindows="), WindowsOn);
			const VisionScene::FCityLayout Layout =
				VisionScene::BuildCityBlock(World, Pawn, 20261025, 120, WindowsOn != 0);
			Landmarks = Layout.LabelPositions;
			Waypoints = VisionScene::LoopWaypoints(Layout);
			SessionLabel = RouteLaps > 1
				? FString::Printf(TEXT("city-lap%d"), RouteLaps)
				: TEXT("city");
			Speed = FMath::Max(RouteSpeed, 180.0f);

			// Repeat the block loop in place: the same waypoints, walked again, so the
			// second lap revisits every viewpoint of the first.
			if (RouteLaps > 1 && Waypoints.Num() > 0)
			{
				const TArray<FVector> OneLap = Waypoints;
				for (int32 Lap = 1; Lap < RouteLaps; ++Lap)
				{
					Waypoints.Append(OneLap);
				}
			}
		}
		else
		{
			int32 LabelCount = 0;
			FParse::Value(FCommandLine::Get(), TEXT("VisionLabels="), LabelCount);
			if (LabelCount > 0)
			{
				Landmarks = VisionProbes::SpawnLabels(World, Pawn, LabelCount);
			}

			int32 ProbeCount = 0;
			FParse::Value(FCommandLine::Get(), TEXT("VisionProbes="), ProbeCount);
			if (ProbeCount > 0)
			{
				VisionProbes::SpawnAround(World, Pawn, ProbeCount);
			}
		}

		URouteDriverComponent* Driver = NewObject<URouteDriverComponent>(Pawn, TEXT("RouteDriver"));
		Driver->RegisterComponent();
		if (RouteLaps > 1 && RouteName.Len() > 0 && SceneName != TEXT("city"))
		{
			SessionLabel = FString::Printf(TEXT("%s-lap%d"), *RouteName, RouteLaps);
		}
		StartAfterWarmup(FTimerDelegate::CreateWeakLambda(this, [this, Driver, Component, Waypoints, Speed, SessionLabel, Landmarks]()
		{
			UWorld* World = GetWorld();
			const bool bStarted = Waypoints.Num() > 0
				? Driver->StartRouteWithWaypoints(Waypoints, Speed)
				: Driver->StartRouteLaps(RouteName, RouteSpeed, RouteLaps);
			if (bStarted)
			{
				Component->StartRecording(SessionNameOverride.IsEmpty() ? SessionLabel : SessionNameOverride);

				// Landmark positions are the only way to score the map itself: trajectory
				// error says where the camera went, not whether the points landed in the
				// right place. They are written next to the frames but never read by the
				// reconstruction.
				if (Landmarks.Num() > 0)
				{
					FString Json = TEXT("{\n  \"kind\": \"unique_text_labels\",\n");
					Json += TEXT("  \"units\": \"UE world, centimetres\",\n  \"points\": [\n");
					for (int32 Index = 0; Index < Landmarks.Num(); ++Index)
					{
						Json += FString::Printf(TEXT("    [%.2f, %.2f, %.2f]%s\n"),
							Landmarks[Index].X, Landmarks[Index].Y, Landmarks[Index].Z,
							Index + 1 < Landmarks.Num() ? TEXT(",") : TEXT(""));
					}
					Json += TEXT("  ]\n}\n");

					FFileHelper::SaveStringToFile(Json,
						*FPaths::Combine(Component->GetSessionDirectory(), TEXT("landmarks.json")),
						FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
				}

				FTimerHandle RouteTimer;
				World->GetTimerManager().SetTimer(RouteTimer, FTimerDelegate::CreateLambda([this]()
				{
					if (Recorder.IsValid())
					{
						Recorder->StopRecording();
						UE_LOG(LogGSVSLAM, Log, TEXT("ROUTE SESSION COMPLETE"));
					}
					if (bQuitWhenDone)
					{
						FPlatformMisc::RequestExit(false);
					}
				}), Driver->GetDurationSeconds() + 0.5f, false);
			}
		}));
	}
}


void UVisionRecorderSubsystem::StartAfterWarmup(FTimerDelegate Start)
{
    if (!Recorder.IsValid() || !GetWorld())
    {
        return;
    }
    Recorder->BeginWarmup(WarmupSeconds);
    UE_LOG(LogGSVSLAM, Log, TEXT("VisionRecorder: fixed warmup %.2f s; route and recording have not started."), WarmupSeconds);
    if (WarmupSeconds > 0.0f)
    {
        GetWorld()->GetTimerManager().SetTimer(WarmupTimer, Start, WarmupSeconds, false);
    }
    else
    {
        Start.ExecuteIfBound();
    }
}

namespace
{
	UVisionRecorderComponent* FindRecorder(UWorld* World)
	{
		if (!World)
		{
			return nullptr;
		}
		UVisionRecorderSubsystem* Subsystem = World->GetSubsystem<UVisionRecorderSubsystem>();
		return Subsystem ? Subsystem->GetRecorder() : nullptr;
	}

	FAutoConsoleCommandWithWorldAndArgs GVisionRecStart(
		TEXT("VisionRec.Start"),
		TEXT("Start recording. Usage: VisionRec.Start [session-name]"),
		FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(
			[](const TArray<FString>& Args, UWorld* World)
			{
				if (UVisionRecorderComponent* Recorder = FindRecorder(World))
				{
					Recorder->StartRecording(Args.Num() > 0 ? Args[0] : FString());
				}
				else
				{
					UE_LOG(LogGSVSLAM, Warning, TEXT("VisionRec.Start: no recorder on the current pawn yet."));
				}
			}));

	FAutoConsoleCommandWithWorld GVisionRecStop(
		TEXT("VisionRec.Stop"),
		TEXT("Stop recording and write session.json"),
		FConsoleCommandWithWorldDelegate::CreateStatic(
			[](UWorld* World)
			{
				if (UVisionRecorderComponent* Recorder = FindRecorder(World))
				{
					Recorder->StopRecording();
				}
			}));

	FAutoConsoleCommandWithWorld GVisionRecStatus(
		TEXT("VisionRec.Status"),
		TEXT("Print recorder status"),
		FConsoleCommandWithWorldDelegate::CreateStatic(
			[](UWorld* World)
			{
				const FString Status = FindRecorder(World) ? FindRecorder(World)->GetStatusString() : FString(TEXT("no recorder"));
				UE_LOG(LogGSVSLAM, Log, TEXT("VisionRec: %s"), *Status);
				if (GEngine)
				{
					GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Cyan, FString::Printf(TEXT("VisionRec: %s"), *Status));
				}
			}));
}
