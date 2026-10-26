#include "SpatialLabRuntime.h"
#include "SpatialLabVisibility.h"
#include "SpatialLabSearch.h"
#include "AIController.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Navigation/PathFollowingComponent.h"
#include "NavigationSystem.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    FVector ReadVector(const TArray<TSharedPtr<FJsonValue>>& A)
    {
        return A.Num() == 3 ? FVector(A[0]->AsNumber(), A[1]->AsNumber(), A[2]->AsNumber()) : FVector(NAN);
    }
    TSharedPtr<FJsonValue> JsonVector(FVector P)
    {
        return MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{
            MakeShared<FJsonValueNumber>(P.X), MakeShared<FJsonValueNumber>(P.Y), MakeShared<FJsonValueNumber>(P.Z)});
    }
}

ASpatialLabGameMode::ASpatialLabGameMode()
{
    DefaultPawnClass = nullptr;
    HUDClass = nullptr;
}

void ASpatialLabGameMode::StartPlay()
{
    Super::StartPlay();
    FString Input;
    if (FParse::Value(FCommandLine::Get(), TEXT("SITPSearch="), Input))
    {
        GetWorld()->SpawnActor<ASpatialLabSearchRunner>();
        return;
    }
    if (FParse::Value(FCommandLine::Get(), TEXT("SITPPlan="), Input))
        GetWorld()->SpawnActor<ASpatialLabEpisodeRunner>();
}

ASpatialLabEpisodeRunner::ASpatialLabEpisodeRunner()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PostPhysics;
}

void ASpatialLabEpisodeRunner::BeginPlay()
{
    Super::BeginPlay();
    FString Input, Text;
    if (!FParse::Value(FCommandLine::Get(), TEXT("SITPPlan="), Input)
        || !FParse::Value(FCommandLine::Get(), TEXT("SITPResult="), ResultPath)
        || IFileManager::Get().FileExists(*ResultPath)
        || !FFileHelper::LoadFileToString(Text, *Input)
        || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Plan)
        || !Plan.IsValid() || Plan->GetStringField(TEXT("condition")) != TEXT("engine_oracle")
        || Plan->GetStringField(TEXT("map")) != GetWorld()->GetOutermost()->GetName())
    {
        UE_LOG(LogTemp, Error, TEXT("SITP_RUNTIME_INPUT_REJECTED"));
        Done = true;
        FPlatformMisc::RequestExitWithStatus(false, 2);
        return;
    }
    const TArray<TSharedPtr<FJsonValue>>* InputRuns = nullptr;
    if (!Plan->TryGetArrayField(TEXT("runs"), InputRuns) || !InputRuns || InputRuns->Num() == 0 || InputRuns->Num() > 100)
    {
        Done = true;
        FPlatformMisc::RequestExitWithStatus(false, 2);
        return;
    }
    Runs = *InputRuns;
    AttemptId = FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
    FString ObservationPath;
    if (FParse::Value(FCommandLine::Get(),TEXT("SITPObservations="),ObservationPath))
    {
        Visibility = MakeShared<FSpatialLabVisibilityRecorder>();
        if (!Visibility->Open(GetWorld(),ObservationPath,Plan->GetStringField(TEXT("manifest_sha256")),AttemptId))
        {
            Done = true;
            UE_LOG(LogTemp,Error,TEXT("SITP_OBSERVATION_OPEN_FAILED"));
            FPlatformMisc::RequestExitWithStatus(false,3);
            return;
        }
    }
    NextStartSim = GetWorld()->GetTimeSeconds() + 1;
    Save(false);
}

FVector ASpatialLabEpisodeRunner::Feet() const
{
    if (!Agent) return Previous;
    return Agent->GetActorLocation() - FVector(0,0,Agent->GetCapsuleComponent()->GetScaledCapsuleHalfHeight());
}

void ASpatialLabEpisodeRunner::BeginNext()
{
    ++Index;
    if (Index >= Runs.Num())
    {
        Done = true;
        const bool Complete = !Visibility || Visibility->Finish();
        Save(Complete);
        UE_LOG(LogTemp, Display, TEXT("SITP_RUNTIME_COMPLETE episodes=%d"), Results.Num());
        FPlatformMisc::RequestExitWithStatus(false, Complete ? 0 : 3);
        return;
    }
    Current = Runs[Index]->AsObject();
    const TSharedPtr<FJsonObject> Task = Current->GetObjectField(TEXT("task"));
    const TSharedPtr<FJsonObject> Decision = Current->GetObjectField(TEXT("decision"));
    Previous = ReadVector(Task->GetArrayField(TEXT("start_cm")));
    InitialFeet = Previous;
    StartedWall = FPlatformTime::Seconds();
    StartedSim = GetWorld()->GetTimeSeconds();
    if (Visibility) Visibility->BeginEpisode(Current->GetStringField(TEXT("run_id")),StartedSim);
    Trajectory.Empty();
    Trajectory.Add(JsonVector(Previous));
    CollisionEvents = 0;
    LastCollisionSim = -1;
    WalkedCm = 0;
    LastMoveResult.Empty();
    MaxSim = Task->GetNumberField(TEXT("max_sim_s"));
    MaxPath = Task->GetNumberField(TEXT("max_path_cm"));
    if (Decision->GetStringField(TEXT("status")) != TEXT("complete"))
    {
        FinishRun(TEXT("selection_") + Decision->GetStringField(TEXT("status")));
        return;
    }
    Goal = ReadVector(Decision->GetArrayField(TEXT("goal_cm")));
    if (Previous.ContainsNaN() || Goal.ContainsNaN() || MaxSim <= 0 || MaxSim > 120 || MaxPath <= 0 || MaxPath > 100000)
    {
        FinishRun(TEXT("invalid_task"));
        return;
    }
    const FVector Center = Previous + FVector(0,0,90);
    UNavigationSystemV1* Nav = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    FNavLocation Projected;
    if (!Nav || !Nav->ProjectPointToNavigation(Previous, Projected, FVector(5,5,50))
        || FVector::Dist2D(Projected.Location, Previous) > 5)
    {
        FinishRun(TEXT("spawn_not_on_navmesh"));
        return;
    }
    if (GetWorld()->OverlapBlockingTestByChannel(Center, FQuat::Identity, ECC_Pawn, FCollisionShape::MakeCapsule(34,88)))
    {
        FinishRun(TEXT("spawn_blocked"));
        return;
    }
    const FTransform Transform(FQuat::Identity, Center);
    Agent = GetWorld()->SpawnActorDeferred<ACharacter>(ACharacter::StaticClass(), Transform, nullptr, nullptr,
                                                      ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Agent) { FinishRun(TEXT("spawn_failed")); return; }
    Agent->GetCapsuleComponent()->InitCapsuleSize(34,88);
    Agent->GetCapsuleComponent()->SetCanEverAffectNavigation(false);
    Agent->GetCapsuleComponent()->SetNotifyRigidBodyCollision(true);
    Agent->GetCharacterMovement()->MaxWalkSpeed = 300;
    Agent->GetCharacterMovement()->MaxStepHeight = 35;
    Agent->GetCharacterMovement()->SetWalkableFloorAngle(45);
    Agent->FinishSpawning(Transform);
    Agent->OnActorHit.AddDynamic(this, &ASpatialLabEpisodeRunner::OnImpact);
    Controller = GetWorld()->SpawnActor<AAIController>();
    if (!Controller) { FinishRun(TEXT("controller_spawn_failed")); return; }
    Controller->Possess(Agent);
    InitialFeet = Feet();
    Previous = InitialFeet;
    Trajectory[0] = JsonVector(InitialFeet);
    if (Visibility) Visibility->Sample(Agent,Feet(),true);
    Controller->GetPathFollowingComponent()->OnRequestFinished.AddUObject(this, &ASpatialLabEpisodeRunner::OnMoveFinished);
    FString Executor;
    Decision->TryGetStringField(TEXT("executor"), Executor);
    const bool UseNavMesh = Executor != TEXT("direct_line");
    const auto Request = Controller->MoveToLocation(Goal, Task->GetNumberField(TEXT("arrival_tolerance_cm")),
                                                   false, UseNavMesh, false, true, nullptr, false);
    if (Request == EPathFollowingRequestResult::Failed)
        FinishRun(TEXT("runtime_path_rejected"));
    else
        Active = true;
}

void ASpatialLabEpisodeRunner::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (Done) return;
    if (Visibility && !Visibility->IsHealthy())
    {
        if (Active) FinishRun(TEXT("observation_io_failure"));
        Done = true;
        Save(false);
        FPlatformMisc::RequestExitWithStatus(false,3);
        return;
    }
    if (!Active)
    {
        if (GetWorld()->GetTimeSeconds() >= NextStartSim) BeginNext();
        return;
    }
    const FVector CurrentFeet = Feet();
    WalkedCm += FVector::Distance(Previous, CurrentFeet);
    Previous = CurrentFeet;
    Trajectory.Add(JsonVector(CurrentFeet));
    if (Visibility) Visibility->Sample(Agent,CurrentFeet);
    const double Elapsed = GetWorld()->GetTimeSeconds() - StartedSim;
    if (!Agent || CurrentFeet.ContainsNaN()) FinishRun(TEXT("agent_lost"));
    else if (WalkedCm > MaxPath) FinishRun(TEXT("path_budget"));
    else if (Elapsed >= MaxSim) FinishRun(TEXT("timeout"));
    else if (FPlatformTime::Seconds() - StartedWall > 120) FinishRun(TEXT("wall_timeout"));
    else if (Controller->GetMoveStatus() == EPathFollowingStatus::Idle)
    {
        const double Tolerance = Current->GetObjectField(TEXT("task"))->GetNumberField(TEXT("arrival_tolerance_cm"));
        FinishRun(FVector::Dist2D(CurrentFeet, Goal) <= Tolerance + 5 && FMath::Abs(CurrentFeet.Z - Goal.Z) <= 50
                  ? TEXT("arrived") : (LastMoveResult.IsEmpty() ? TEXT("movement_stopped") : TEXT("movement_") + LastMoveResult));
    }
}

void ASpatialLabEpisodeRunner::OnMoveFinished(FAIRequestID Request, const FPathFollowingResult& Result)
{
    switch (Result.Code)
    {
        case EPathFollowingResult::Success: LastMoveResult = TEXT("success"); break;
        case EPathFollowingResult::Blocked: LastMoveResult = TEXT("blocked"); break;
        case EPathFollowingResult::OffPath: LastMoveResult = TEXT("off_path"); break;
        case EPathFollowingResult::Aborted: LastMoveResult = TEXT("aborted"); break;
        case EPathFollowingResult::Invalid: LastMoveResult = TEXT("invalid"); break;
        default: LastMoveResult = TEXT("unknown"); break;
    }
}

void ASpatialLabEpisodeRunner::OnImpact(AActor* Self, AActor* Other, FVector Impulse, const FHitResult& Hit)
{
    // Count non-floor blocking contact events, with 0.1 simulated seconds of
    // debounce. This is an event metric, not penetration distance or impulse.
    const double Time = GetWorld()->GetTimeSeconds();
    if (Active && Hit.bBlockingHit && Hit.ImpactNormal.Z < 0.7071 && Time - LastCollisionSim >= .1)
    {
        ++CollisionEvents;
        LastCollisionSim = Time;
    }
}

void ASpatialLabEpisodeRunner::FinishRun(const FString& Status)
{
    if (Visibility)
    {
        if (Agent) Visibility->Sample(Agent,Feet(),true);
        Visibility->EndEpisode(Status);
    }
    const bool HadAgent = Agent != nullptr;
    TSharedRef<FJsonObject> Execution = MakeShared<FJsonObject>();
    Execution->SetStringField(TEXT("status"), Status);
    Execution->SetStringField(TEXT("path_following_result"), LastMoveResult);
    FString Entity;
    if (Current->GetObjectField(TEXT("decision"))->TryGetStringField(TEXT("entity_id"), Entity))
        Execution->SetStringField(TEXT("entity_id"), Entity);
    else Execution->SetField(TEXT("entity_id"), MakeShared<FJsonValueNull>());
    Execution->SetField(TEXT("final_position_cm"), JsonVector(Feet()));
    Execution->SetField(TEXT("initial_position_cm"), JsonVector(InitialFeet));
    Execution->SetArrayField(TEXT("trajectory_cm"), Trajectory);
    Execution->SetNumberField(TEXT("continuous_path_length_cm"), WalkedCm);
    Execution->SetNumberField(TEXT("wall_s"), FPlatformTime::Seconds() - StartedWall);
    Execution->SetNumberField(TEXT("sim_s"), GetWorld()->GetTimeSeconds() - StartedSim);
    if (HadAgent) Execution->SetNumberField(TEXT("collisions"), CollisionEvents);
    else Execution->SetField(TEXT("collisions"), MakeShared<FJsonValueNull>());
    TSharedRef<FJsonObject> Row = MakeShared<FJsonObject>();
    Row->SetStringField(TEXT("run_id"), Current->GetStringField(TEXT("run_id")));
    Row->SetStringField(TEXT("policy"), Current->GetStringField(TEXT("policy")));
    Row->SetObjectField(TEXT("task"), Current->GetObjectField(TEXT("task")));
    Row->SetObjectField(TEXT("decision"), Current->GetObjectField(TEXT("decision")));
    Row->SetObjectField(TEXT("execution"), Execution);
    Results.Add(MakeShared<FJsonValueObject>(Row));
    UE_LOG(LogTemp, Display, TEXT("SITP_EPISODE_END index=%d status=%s sim_s=%.3f"), Index, *Status,
           GetWorld()->GetTimeSeconds() - StartedSim);
    Active = false;
    if (Controller) { Controller->UnPossess(); Controller->Destroy(); Controller = nullptr; }
    if (Agent) { Agent->Destroy(); Agent = nullptr; }
    NextStartSim = GetWorld()->GetTimeSeconds() + .25;
    Save(false);
}

void ASpatialLabEpisodeRunner::Save(bool Finished)
{
    TSharedRef<FJsonObject> Report = MakeShared<FJsonObject>();
    Report->SetNumberField(TEXT("schema_version"), 1);
    Report->SetStringField(TEXT("condition"), TEXT("engine_oracle"));
    Report->SetStringField(TEXT("evidence_kind"), TEXT("ue_game_world_character_movement"));
    Report->SetStringField(TEXT("units"), TEXT("centimeters"));
    Report->SetStringField(TEXT("position_reference"), TEXT("capsule_feet"));
    Report->SetStringField(TEXT("clock"), TEXT("engine_simulated_seconds_and_process_wall_seconds"));
    Report->SetStringField(TEXT("collision_definition"), TEXT("blocking non-floor impact events, 0.1 sim-s debounce"));
    Report->SetStringField(TEXT("manifest_sha256"), Plan->GetStringField(TEXT("manifest_sha256")));
    Report->SetStringField(TEXT("attempt_id"), AttemptId);
    Report->SetStringField(TEXT("map"), GetWorld()->GetOutermost()->GetName());
    Report->SetBoolField(TEXT("finished"), Finished);
    Report->SetNumberField(TEXT("expected_runs"), Runs.Num());
    Report->SetArrayField(TEXT("runs"), Results);
    FString Text;
    FJsonSerializer::Serialize(Report, TJsonWriterFactory<>::Create(&Text));
    const FString Temporary = ResultPath + TEXT(".tmp");
    if (!FFileHelper::SaveStringToFile(Text, *Temporary) || !IFileManager::Get().Move(*ResultPath, *Temporary, true))
    {
        UE_LOG(LogTemp, Error, TEXT("SITP_RESULT_WRITE_FAILED"));
        Done = true;
        FPlatformMisc::RequestExitWithStatus(false, 3);
    }
}
