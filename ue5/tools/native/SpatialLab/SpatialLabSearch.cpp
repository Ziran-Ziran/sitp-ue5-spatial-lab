#include "SpatialLabSearch.h"
#include "SpatialLabVisibility.h"
#include "SpatialLabPolicyBridge.h"
#include "AIController.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Navigation/PathFollowingComponent.h"
#include "NavigationSystem.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    FVector Vector(const TArray<TSharedPtr<FJsonValue>>& A)
    {
        return A.Num()==3 ? FVector(A[0]->AsNumber(),A[1]->AsNumber(),A[2]->AsNumber()) : FVector(NAN);
    }
    TSharedPtr<FJsonValue> JsonVector(FVector P)
    {
        return MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{
            MakeShared<FJsonValueNumber>(P.X),MakeShared<FJsonValueNumber>(P.Y),MakeShared<FJsonValueNumber>(P.Z)});
    }
}

ASpatialLabSearchRunner::ASpatialLabSearchRunner()
{
    PrimaryActorTick.bCanEverTick=true;
    PrimaryActorTick.TickGroup=TG_PostPhysics;
}

void ASpatialLabSearchRunner::BeginPlay()
{
    Super::BeginPlay();
    FString InputPath,Text;
    const TArray<TSharedPtr<FJsonValue>>* Rows=nullptr;
    if (!FParse::Value(FCommandLine::Get(),TEXT("SITPSearch="),InputPath)
        || !FParse::Value(FCommandLine::Get(),TEXT("SITPResult="),ResultPath)
        || !FParse::Value(FCommandLine::Get(),TEXT("SITPPolicyTools="),ToolsDirectory)
        || !FParse::Value(FCommandLine::Get(),TEXT("SITPObservations="),ObservationPath)
        || IFileManager::Get().FileExists(*ResultPath)
        || !FFileHelper::LoadFileToString(Text,*InputPath)
        || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Input) || !Input.IsValid()
        || Input->GetStringField(TEXT("condition"))!=TEXT("engine_local_visibility_supervision")
        || Input->GetStringField(TEXT("map"))!=GetWorld()->GetOutermost()->GetName()
        || !Input->TryGetArrayField(TEXT("missions"),Rows) || !Rows || Rows->Num()<1 || Rows->Num()>100)
    {
        Done=true;
        UE_LOG(LogTemp,Error,TEXT("SITP_SEARCH_INPUT_REJECTED"));
        FPlatformMisc::RequestExitWithStatus(false,2);
        return;
    }
    Missions=*Rows;
    AttemptId=FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
    Visibility=MakeShared<FSpatialLabVisibilityRecorder>();
    if (!Visibility->Open(GetWorld(),ObservationPath,Input->GetStringField(TEXT("manifest_sha256")),AttemptId,
                          TEXT("observation_dependent_engine_navmesh_geometry")))
    {
        Done=true;
        FPlatformMisc::RequestExitWithStatus(false,3);
        return;
    }
    NextStart=GetWorld()->GetTimeSeconds()+1;
    Save(false);
}

FVector ASpatialLabSearchRunner::Feet() const
{
    return Agent ? Agent->GetActorLocation()-FVector(0,0,Agent->GetCapsuleComponent()->GetScaledCapsuleHalfHeight()) : Previous;
}

void ASpatialLabSearchRunner::BeginNext()
{
    ++Index;
    if (Index>=Missions.Num())
    {
        Done=true;
        const bool Complete=Visibility->Finish();
        Save(Complete);
        UE_LOG(LogTemp,Display,TEXT("SITP_SEARCH_COMPLETE episodes=%d"),Results.Num());
        FPlatformMisc::RequestExitWithStatus(false,Complete?0:3);
        return;
    }
    Mission=Missions[Index]->AsObject();
    Previous=Vector(Mission->GetArrayField(TEXT("start_cm")));
    InitialFeet=Previous;
    StartedSim=GetWorld()->GetTimeSeconds();
    StartedWall=FPlatformTime::Seconds();
    WalkedCm=PolicyWall=0;
    Collisions=NavigationRequests=0;
    LastCollision=-1;
    LastFrame=LastAction=-1;
    SelectedEntity.Empty();
    FeedbackState=TEXT("none");
    Trajectory.Empty();
    Actions.Empty();
    Trajectory.Add(JsonVector(Previous));
    Visibility->BeginEpisode(Mission->GetStringField(TEXT("run_id")),StartedSim);
    Policy=MakeShared<FSpatialLabPolicyBridge>();
    if (!Policy->Initialize(ToolsDirectory,Mission,ResultPath+TEXT(".policy-response.json"),AttemptId))
    { FinishRun(TEXT("policy_initialization_failed")); return; }
    const FVector Center=Previous+FVector(0,0,90);
    UNavigationSystemV1* Nav=FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
    FNavLocation Projected;
    if (!Nav || Previous.ContainsNaN() || !Nav->ProjectPointToNavigation(Previous,Projected,FVector(5,5,50))
        || FVector::Dist2D(Projected.Location,Previous)>5)
    { FinishRun(TEXT("spawn_not_on_navmesh")); return; }
    if (GetWorld()->OverlapBlockingTestByChannel(Center,FQuat::Identity,ECC_Pawn,FCollisionShape::MakeCapsule(34,88)))
    { FinishRun(TEXT("spawn_blocked")); return; }
    const FTransform Transform(FQuat::Identity,Center);
    Agent=GetWorld()->SpawnActorDeferred<ACharacter>(ACharacter::StaticClass(),Transform,nullptr,nullptr,
                                                      ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Agent) { FinishRun(TEXT("spawn_failed")); return; }
    Agent->GetCapsuleComponent()->InitCapsuleSize(34,88);
    Agent->GetCapsuleComponent()->SetCanEverAffectNavigation(false);
    Agent->GetCapsuleComponent()->SetNotifyRigidBodyCollision(true);
    Agent->GetCharacterMovement()->MaxWalkSpeed=300;
    Agent->GetCharacterMovement()->MaxStepHeight=35;
    Agent->GetCharacterMovement()->SetWalkableFloorAngle(45);
    Agent->FinishSpawning(Transform);
    Agent->OnActorHit.AddDynamic(this,&ASpatialLabSearchRunner::OnImpact);
    Controller=GetWorld()->SpawnActor<AAIController>();
    if (!Controller) { FinishRun(TEXT("controller_spawn_failed")); return; }
    Controller->Possess(Agent);
    InitialFeet=Feet();
    Previous=InitialFeet;
    Trajectory[0]=JsonVector(InitialFeet);
    Controller->GetPathFollowingComponent()->OnRequestFinished.AddUObject(this,&ASpatialLabSearchRunner::OnMoveFinished);
    Active=true;
    Visibility->Sample(Agent,Feet(),true);
    Decide();
}

void ASpatialLabSearchRunner::Decide()
{
    const TSharedPtr<FJsonObject> Frame=Visibility->GetLatestFrame();
    if (!Active || !Frame.IsValid() || Frame->GetIntegerField(TEXT("frame_id"))==LastFrame) return;
    LastFrame=Frame->GetIntegerField(TEXT("frame_id"));
    TSharedPtr<FJsonObject> Feedback=MakeShared<FJsonObject>();
    if (LastAction<0) Feedback->SetField(TEXT("action_id"),MakeShared<FJsonValueNull>());
    else Feedback->SetNumberField(TEXT("action_id"),LastAction);
    Feedback->SetStringField(TEXT("state"),FeedbackState);
    TSharedPtr<FJsonObject> Action;
    const double Started=FPlatformTime::Seconds();
    const bool Okay=Policy->Step(Frame,Feedback,Action);
    const double Elapsed=FPlatformTime::Seconds()-Started;
    PolicyWall+=Elapsed;
    if (!Okay) { FinishRun(TEXT("policy_step_failed")); return; }
    TSharedRef<FJsonObject> Trace=MakeShared<FJsonObject>();
    Trace->SetNumberField(TEXT("frame_id"),LastFrame);
    Trace->SetNumberField(TEXT("t_sim_s"),Frame->GetNumberField(TEXT("t_sim_s")));
    Trace->SetNumberField(TEXT("policy_wall_s"),Elapsed);
    Trace->SetObjectField(TEXT("feedback"),Feedback);
    Trace->SetObjectField(TEXT("action"),Action);
    Actions.Add(MakeShared<FJsonValueObject>(Trace));
    const FString Kind=Action->GetStringField(TEXT("kind"));
    if (Kind==TEXT("hold")) return;
    LastAction=Action->GetIntegerField(TEXT("action_id"));
    if (Kind==TEXT("stop"))
    {
        Action->TryGetStringField(TEXT("entity_id"),SelectedEntity);
        FinishRun(Action->GetStringField(TEXT("reason")));
    }
    else if (Kind==TEXT("cancel"))
    {
        Controller->StopMovement();
        FeedbackState=TEXT("cancelled");
    }
    else if (Kind==TEXT("move"))
    {
        const FVector Goal=Vector(Action->GetArrayField(TEXT("goal_cm")));
        if (Goal.ContainsNaN()) { FinishRun(TEXT("invalid_policy_action")); return; }
        FAIMoveRequest Request(Goal);
        Request.SetAcceptanceRadius(Mission->GetNumberField(TEXT("arrival_tolerance_cm")));
        Request.SetReachTestIncludesAgentRadius(false);
        Request.SetReachTestIncludesGoalRadius(false);
        Request.SetProjectGoalLocation(true);
        Request.SetUsePathfinding(true);
        Request.SetAllowPartialPath(false);
        ++NavigationRequests;
        const FPathFollowingRequestResult Result=Controller->MoveTo(Request);
        MoveId=Result.MoveId;
        FeedbackState=Result.Code==EPathFollowingRequestResult::Failed?TEXT("rejected"):
                      Result.Code==EPathFollowingRequestResult::AlreadyAtGoal?TEXT("arrived"):TEXT("moving");
    }
    else FinishRun(TEXT("invalid_policy_action"));
}

void ASpatialLabSearchRunner::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (Done) return;
    if (!Visibility->IsHealthy())
    {
        if (Active) FinishRun(TEXT("observation_io_failure"));
        Done=true;
        Save(false);
        FPlatformMisc::RequestExitWithStatus(false,3);
        return;
    }
    if (!Active)
    {
        if (GetWorld()->GetTimeSeconds()>=NextStart) BeginNext();
        return;
    }
    const FVector Current=Feet();
    WalkedCm+=FVector::Distance(Previous,Current);
    Previous=Current;
    Trajectory.Add(JsonVector(Current));
    Visibility->Sample(Agent,Current);
    if (!Agent || Current.ContainsNaN()) FinishRun(TEXT("agent_lost"));
    else if (WalkedCm>Mission->GetNumberField(TEXT("max_path_cm"))) FinishRun(TEXT("path_budget"));
    else if (GetWorld()->GetTimeSeconds()-StartedSim>=Mission->GetNumberField(TEXT("max_sim_s"))) FinishRun(TEXT("timeout"));
    else if (FPlatformTime::Seconds()-StartedWall>120) FinishRun(TEXT("wall_timeout"));
    else Decide();
}

void ASpatialLabSearchRunner::OnMoveFinished(FAIRequestID Request,const FPathFollowingResult& Result)
{
    if (Request!=MoveId) return;
    FeedbackState=Result.Code==EPathFollowingResult::Success?TEXT("arrived"):TEXT("blocked");
}

void ASpatialLabSearchRunner::OnImpact(AActor* Self,AActor* Other,FVector Impulse,const FHitResult& Hit)
{
    const double Time=GetWorld()->GetTimeSeconds();
    if (Active && Hit.bBlockingHit && Hit.ImpactNormal.Z<0.7071 && Time-LastCollision>=.1)
    { ++Collisions; LastCollision=Time; }
}

void ASpatialLabSearchRunner::FinishRun(const FString& Status)
{
    if (Agent) Visibility->Sample(Agent,Feet(),true);
    Visibility->EndEpisode(Status);
    TSharedRef<FJsonObject> Execution=MakeShared<FJsonObject>();
    Execution->SetStringField(TEXT("status"),Status);
    if (SelectedEntity.IsEmpty()) Execution->SetField(TEXT("entity_id"),MakeShared<FJsonValueNull>());
    else Execution->SetStringField(TEXT("entity_id"),SelectedEntity);
    Execution->SetField(TEXT("initial_position_cm"),JsonVector(InitialFeet));
    Execution->SetField(TEXT("final_position_cm"),JsonVector(Feet()));
    Execution->SetArrayField(TEXT("trajectory_cm"),Trajectory);
    Execution->SetArrayField(TEXT("policy_trace"),Actions);
    Execution->SetNumberField(TEXT("continuous_path_length_cm"),WalkedCm);
    Execution->SetNumberField(TEXT("sim_s"),GetWorld()->GetTimeSeconds()-StartedSim);
    Execution->SetNumberField(TEXT("wall_s"),FPlatformTime::Seconds()-StartedWall);
    Execution->SetNumberField(TEXT("policy_wall_s"),PolicyWall);
    Execution->SetNumberField(TEXT("navigation_requests"),NavigationRequests);
    if (Agent) Execution->SetNumberField(TEXT("collisions"),Collisions);
    else Execution->SetField(TEXT("collisions"),MakeShared<FJsonValueNull>());
    TSharedRef<FJsonObject> Row=MakeShared<FJsonObject>();
    Row->SetStringField(TEXT("run_id"),Mission->GetStringField(TEXT("run_id")));
    Row->SetStringField(TEXT("policy"),Mission->GetStringField(TEXT("policy")));
    Row->SetObjectField(TEXT("mission"),Mission);
    Row->SetObjectField(TEXT("execution"),Execution);
    Results.Add(MakeShared<FJsonValueObject>(Row));
    UE_LOG(LogTemp,Display,TEXT("SITP_SEARCH_END index=%d status=%s"),Index,*Status);
    Active=false;
    if (Controller) { Controller->StopMovement(); Controller->UnPossess(); Controller->Destroy(); Controller=nullptr; }
    if (Agent) { Agent->Destroy(); Agent=nullptr; }
    Policy.Reset();
    NextStart=GetWorld()->GetTimeSeconds()+.25;
    Save(false);
}

void ASpatialLabSearchRunner::Save(bool Finished)
{
    TSharedRef<FJsonObject> Report=MakeShared<FJsonObject>();
    Report->SetNumberField(TEXT("schema_version"),1);
    Report->SetStringField(TEXT("condition"),TEXT("engine_local_visibility_supervision"));
    Report->SetStringField(TEXT("evidence_kind"),TEXT("ue_game_world_observation_dependent_search"));
    Report->SetStringField(TEXT("units"),TEXT("centimeters"));
    Report->SetStringField(TEXT("position_reference"),TEXT("capsule_feet"));
    Report->SetStringField(TEXT("map"),GetWorld()->GetOutermost()->GetName());
    Report->SetStringField(TEXT("manifest_sha256"),Input->GetStringField(TEXT("manifest_sha256")));
    Report->SetStringField(TEXT("attempt_id"),AttemptId);
    Report->SetStringField(TEXT("geometry_source"),TEXT("engine_navmesh_exploration_and_paths"));
    Report->SetBoolField(TEXT("finished"),Finished);
    Report->SetNumberField(TEXT("expected_runs"),Missions.Num());
    Report->SetArrayField(TEXT("runs"),Results);
    FString Text;
    FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Text));
    const FString Temporary=ResultPath+TEXT(".tmp");
    if (!FFileHelper::SaveStringToFile(Text,*Temporary) || !IFileManager::Get().Move(*ResultPath,*Temporary,true))
    {
        Done=true;
        FPlatformMisc::RequestExitWithStatus(false,3);
    }
}
