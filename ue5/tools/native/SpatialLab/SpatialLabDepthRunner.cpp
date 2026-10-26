#include "SpatialLabDepthRunner.h"
#include "AIController.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    FVector Vector(const TArray<TSharedPtr<FJsonValue>>& Values)
    {
        double X,Y,Z;
        if (Values.Num()!=3 || !Values[0]->TryGetNumber(X) || !Values[1]->TryGetNumber(Y) || !Values[2]->TryGetNumber(Z))
            return FVector(NAN);
        return FVector(X,Y,Z);
    }
    TSharedPtr<FJsonValue> JsonVector(const FVector& P)
    {
        return MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{
            MakeShared<FJsonValueNumber>(P.X),MakeShared<FJsonValueNumber>(P.Y),MakeShared<FJsonValueNumber>(P.Z)});
    }
    double SegmentDistance(const FVector& P,const FVector& A,const FVector& B)
    {
        const FVector2D Delta(B.X-A.X,B.Y-A.Y),Offset(P.X-A.X,P.Y-A.Y);
        const double T=Delta.SizeSquared()>1e-12 ? FMath::Clamp(FVector2D::DotProduct(Offset,Delta)/Delta.SizeSquared(),0.,1.) : 0.;
        return (Offset-Delta*T).Size();
    }
}

ASpatialLabDepthRunner::ASpatialLabDepthRunner()
{
    PrimaryActorTick.bCanEverTick=true;
    PrimaryActorTick.TickGroup=TG_PostPhysics;
}

void ASpatialLabDepthRunner::BeginPlay()
{
    Super::BeginPlay();
    FString Path,Text;
    const TArray<TSharedPtr<FJsonValue>>* Rows=nullptr;
    bool Valid=FParse::Value(FCommandLine::Get(),TEXT("SITPDepthPlan="),Path)
        && FParse::Value(FCommandLine::Get(),TEXT("SITPResult="),ResultPath)
        && !IFileManager::Get().FileExists(*ResultPath) && FFileHelper::LoadFileToString(Text,*Path)
        && FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Input) && Input.IsValid();
    if (Valid)
    {
        FString Condition,Map;
        Valid=Input->TryGetStringField(TEXT("condition"),Condition)
            && Condition==TEXT("depth_grid_pose_supervised_frozen_path_execution")
            && Input->TryGetStringField(TEXT("map"),Map) && Map==GetWorld()->GetOutermost()->GetName()
            && Input->TryGetArrayField(TEXT("runs"),Rows) && Rows && Rows->Num()>0 && Rows->Num()<=200;
    }
    if (Valid)
    {
        Settings=Input->GetObjectField(TEXT("settings"));
        Valid=Settings.IsValid() && Settings->GetNumberField(TEXT("radius_cm"))==34
            && Settings->GetNumberField(TEXT("half_height_cm"))==88 && Settings->GetNumberField(TEXT("speed_cm_s"))==300
            && Settings->GetNumberField(TEXT("waypoint_tolerance_cm"))==5
            && Settings->GetNumberField(TEXT("tracking_limit_cm"))==10
            && Settings->GetNumberField(TEXT("max_sim_s"))>0 && Settings->GetNumberField(TEXT("max_sim_s"))<=60
            && Settings->GetNumberField(TEXT("max_path_cm"))>0 && Settings->GetNumberField(TEXT("max_path_cm"))<=10000;
    }
    if (Valid)
    {
        TSet<FString> Seen;
        for (const auto& Value:*Rows)
        {
            const auto Row=Value->AsObject();
            const TArray<TSharedPtr<FJsonValue>>* Start=nullptr;
            const TArray<TSharedPtr<FJsonValue>>* Points=nullptr;
            FString Key,Gate;
            double Heading;
            if (!Row.IsValid() || !Row->TryGetStringField(TEXT("run_id"),Key) || Key.IsEmpty() || Seen.Contains(Key)
                || !Row->TryGetStringField(TEXT("gate_status"),Gate) || !Row->TryGetNumberField(TEXT("heading_deg"),Heading)
                || !FMath::IsFinite(Heading) || !Row->TryGetArrayField(TEXT("start_cm"),Start) || Vector(*Start).ContainsNaN()
                || !Row->TryGetArrayField(TEXT("world_waypoints_cm"),Points) || Points->Num()>256
                || (Gate==TEXT("approved") && Points->Num()==0)) { Valid=false; break; }
            for (const auto& Point:*Points) if (Vector(Point->AsArray()).ContainsNaN()) Valid=false;
            Seen.Add(Key);
        }
    }
    if (!Valid)
    {
        Done=true;UE_LOG(LogTemp,Error,TEXT("SITP_DEPTH_INPUT_REJECTED"));
        FPlatformMisc::RequestExitWithStatus(false,2);return;
    }
    Runs=*Rows;AttemptId=FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
    NextStart=GetWorld()->GetTimeSeconds()+1;Save(false);
}

FVector ASpatialLabDepthRunner::Feet() const
{
    return Agent ? Agent->GetActorLocation()-FVector(0,0,Agent->GetCapsuleComponent()->GetScaledCapsuleHalfHeight()) : Previous;
}

void ASpatialLabDepthRunner::BeginNext()
{
    if (++Index>=Runs.Num())
    {
        Done=true;Save(true);UE_LOG(LogTemp,Display,TEXT("SITP_DEPTH_COMPLETE runs=%d"),Results.Num());
        FPlatformMisc::RequestExitWithStatus(false,0);return;
    }
    Current=Runs[Index]->AsObject();RequestedStart=Vector(Current->GetArrayField(TEXT("start_cm")));
    Previous=InitialFeet=RequestedStart;StartedSim=GetWorld()->GetTimeSeconds();StartedWall=FPlatformTime::Seconds();
    ResetWall=StartedWall;SpawnedSim=StartedSim;Measured=false;Cursor=0;Collisions=0;WalkedCm=0;MaxTracking=0;
    ResetXY=ResetZ=0;LastCollision=-1;Trajectory.Empty();WaypointEvents.Empty();Waypoints.Empty();
    if (Current->GetStringField(TEXT("gate_status"))!=TEXT("approved"))
    { Finish(TEXT("not_executed_")+Current->GetStringField(TEXT("gate_status")));return; }
    for (const auto& Point:Current->GetArrayField(TEXT("world_waypoints_cm"))) Waypoints.Add(Vector(Point->AsArray()));
    const FVector Center=RequestedStart+FVector(0,0,90);
    const FCollisionShape Shape=FCollisionShape::MakeCapsule(34,88);
    const bool Overlap=GetWorld()->OverlapBlockingTestByChannel(Center,FQuat::Identity,ECC_Pawn,Shape);
    // CharacterMovement resolves initial penetration using sweeps. Check that
    // same initial-penetration condition before spawning, including deep overlap
    // cases where the overlap-only query did not flag the contained capsule.
    FHitResult InitialHit;
    FCollisionQueryParams Query(SCENE_QUERY_STAT(SpatialLabDepthSpawn),false);
    const bool Sweep=GetWorld()->SweepSingleByChannel(InitialHit,Center,Center+FVector(.01,0,0),FQuat::Identity,ECC_Pawn,Shape,Query);
    UE_LOG(LogTemp,Display,TEXT("SITP_DEPTH_SPAWN_CHECK index=%d overlap=%d sweep=%d penetrating=%d"),Index,Overlap,Sweep,InitialHit.bStartPenetrating);
    if (Overlap || (Sweep && InitialHit.bStartPenetrating))
    { Finish(TEXT("spawn_blocked"));return; }
    const FTransform Transform(FRotator(0,Current->GetNumberField(TEXT("heading_deg")),0),Center);
    Agent=GetWorld()->SpawnActorDeferred<ACharacter>(ACharacter::StaticClass(),Transform,nullptr,nullptr,
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Agent) { Finish(TEXT("spawn_failed"));return; }
    Agent->GetCapsuleComponent()->InitCapsuleSize(34,88);
    Agent->GetCapsuleComponent()->SetCanEverAffectNavigation(false);
    Agent->GetCapsuleComponent()->SetNotifyRigidBodyCollision(true);
    auto* Movement=Agent->GetCharacterMovement();
    Movement->MaxWalkSpeed=300;Movement->MaxStepHeight=35;Movement->SetWalkableFloorAngle(45);
    Movement->bRunPhysicsWithNoController=true;
    Agent->FinishSpawning(Transform);Agent->OnActorHit.AddDynamic(this,&ASpatialLabDepthRunner::OnImpact);
    Controller=GetWorld()->SpawnActor<AAIController>();
    if (!Controller) { Finish(TEXT("controller_spawn_failed"));return; }
    Controller->Possess(Agent);
    // Let floor settling finish without motion before starting route metrics.
    Settling=true;
}

void ASpatialLabDepthRunner::InitializeMovement()
{
    Settling=false;InitialFeet=Previous=Feet();ResetXY=FVector::Dist2D(InitialFeet,RequestedStart);
    ResetZ=FMath::Abs(InitialFeet.Z-RequestedStart.Z);Measured=true;
    StartedSim=GetWorld()->GetTimeSeconds();StartedWall=FPlatformTime::Seconds();
    ResetWall=StartedWall-ResetWall;Trajectory.Add(JsonVector(InitialFeet));
    if (ResetXY>5 || ResetZ>10) { Finish(TEXT("reset_pose_drift"));return; }
    LastProgress=StartedSim;BestDistance=TNumericLimits<double>::Max();Active=true;Drive();
}

void ASpatialLabDepthRunner::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (Done) return;
    if (Settling)
    {
        if (!IsValid(Agent)) { Finish(TEXT("reset_agent_lost"));return; }
        if (GetWorld()->GetTimeSeconds()-SpawnedSim>=.25) InitializeMovement();
        return;
    }
    if (!Active) { if (GetWorld()->GetTimeSeconds()>=NextStart) BeginNext();return; }
    if (!IsValid(Agent) || Feet().ContainsNaN()) { Finish(TEXT("agent_lost"));return; }
    const FVector Position=Feet();WalkedCm+=FVector::Distance(Previous,Position);Previous=Position;Trajectory.Add(JsonVector(Position));
    if (WalkedCm>Settings->GetNumberField(TEXT("max_path_cm"))) Finish(TEXT("path_budget"));
    else if (GetWorld()->GetTimeSeconds()-StartedSim>=Settings->GetNumberField(TEXT("max_sim_s"))) Finish(TEXT("timeout"));
    else if (FPlatformTime::Seconds()-StartedWall>=30) Finish(TEXT("wall_timeout"));
    else if (FMath::Abs(Position.Z-InitialFeet.Z)>50) Finish(TEXT("vertical_excursion"));
    else Drive();
}

void ASpatialLabDepthRunner::Drive()
{
    const FVector Position=Feet();const double Now=GetWorld()->GetTimeSeconds();
    while (Cursor<Waypoints.Num() && FVector::Dist2D(Position,Waypoints[Cursor])<=5)
    {
        Agent->GetCharacterMovement()->StopMovementImmediately();
        TSharedRef<FJsonObject> Event=MakeShared<FJsonObject>();Event->SetNumberField(TEXT("waypoint_index"),Cursor);
        Event->SetNumberField(TEXT("t_sim_s"),Now-StartedSim);Event->SetField(TEXT("position_cm"),JsonVector(Position));
        WaypointEvents.Add(MakeShared<FJsonValueObject>(Event));++Cursor;
        LastProgress=Now;BestDistance=TNumericLimits<double>::Max();
    }
    if (Cursor==Waypoints.Num())
    { Finish(FMath::Abs(Position.Z-Waypoints.Last().Z)<=20 ? TEXT("arrived") : TEXT("endpoint_height_mismatch"));return; }
    const double Error=SegmentDistance(Position,Cursor>0 ? Waypoints[Cursor-1] : InitialFeet,Waypoints[Cursor]);
    MaxTracking=FMath::Max(MaxTracking,Error);
    if (Error>10) { Finish(TEXT("tracking_deviation"));return; }
    const double Distance=FVector::Dist2D(Position,Waypoints[Cursor]);
    if (Distance<BestDistance-1) { BestDistance=Distance;LastProgress=Now; }
    if (Now-LastProgress>=2) { Finish(TEXT("blocked_no_progress"));return; }
    FVector Direction=Waypoints[Cursor]-Position;Direction.Z=0;
    // Direct world-space input. No MoveTo, pathfinding, NavMesh projection or query.
    Agent->AddMovementInput(Direction.GetSafeNormal(),1,true);
}

void ASpatialLabDepthRunner::OnImpact(AActor* Self,AActor* Other,FVector Impulse,const FHitResult& Hit)
{
    const double Now=GetWorld()->GetTimeSeconds();
    if (Active && Hit.bBlockingHit && Hit.ImpactNormal.Z<.7071 && Now-LastCollision>=.1)
    { ++Collisions;LastCollision=Now; }
}

void ASpatialLabDepthRunner::Finish(const FString& Status)
{
    TSharedRef<FJsonObject> Execution=MakeShared<FJsonObject>();
    Execution->SetStringField(TEXT("status"),Status);Execution->SetBoolField(TEXT("agent_spawned"),Agent!=nullptr);
    Execution->SetBoolField(TEXT("measurement_started"),Measured);
    Execution->SetField(TEXT("initial_position_cm"),Measured ? JsonVector(InitialFeet) : MakeShared<FJsonValueNull>());
    Execution->SetField(TEXT("final_position_cm"),Agent ? JsonVector(Feet()) : MakeShared<FJsonValueNull>());
    Execution->SetArrayField(TEXT("trajectory_cm"),Trajectory);Execution->SetArrayField(TEXT("waypoint_events"),WaypointEvents);
    if (Measured)
    {
        Execution->SetNumberField(TEXT("continuous_path_length_cm"),WalkedCm);Execution->SetNumberField(TEXT("collisions"),Collisions);
        Execution->SetNumberField(TEXT("sim_s"),GetWorld()->GetTimeSeconds()-StartedSim);
        Execution->SetNumberField(TEXT("wall_s"),FPlatformTime::Seconds()-StartedWall);Execution->SetNumberField(TEXT("reset_wall_s"),ResetWall);
        Execution->SetNumberField(TEXT("reset_xy_cm"),ResetXY);Execution->SetNumberField(TEXT("reset_z_cm"),ResetZ);
        Execution->SetNumberField(TEXT("max_tracking_error_cm"),MaxTracking);
    }
    else for (const TCHAR* Key:{TEXT("continuous_path_length_cm"),TEXT("collisions"),TEXT("sim_s"),TEXT("wall_s"),
        TEXT("reset_wall_s"),TEXT("reset_xy_cm"),TEXT("reset_z_cm"),TEXT("max_tracking_error_cm")})
        Execution->SetField(Key,MakeShared<FJsonValueNull>());
    Execution->SetNumberField(TEXT("engine_path_requests"),0);
    TSharedRef<FJsonObject> Row=MakeShared<FJsonObject>();Row->SetStringField(TEXT("run_id"),Current->GetStringField(TEXT("run_id")));
    Row->SetObjectField(TEXT("input"),Current);Row->SetObjectField(TEXT("execution"),Execution);Results.Add(MakeShared<FJsonValueObject>(Row));
    UE_LOG(LogTemp,Display,TEXT("SITP_DEPTH_RUN index=%d status=%s waypoints=%d/%d"),Index,*Status,Cursor,Waypoints.Num());
    Active=Settling=false;
    if (Agent) Agent->GetCharacterMovement()->StopMovementImmediately();
    if (Controller) { Controller->UnPossess();Controller->Destroy();Controller=nullptr; }
    if (Agent) { Agent->Destroy();Agent=nullptr; }
    NextStart=GetWorld()->GetTimeSeconds()+.1;Save(false);
}

void ASpatialLabDepthRunner::Save(bool Finished)
{
    TSharedRef<FJsonObject> Report=MakeShared<FJsonObject>();Report->SetNumberField(TEXT("schema_version"),1);
    Report->SetStringField(TEXT("condition"),TEXT("depth_grid_pose_supervised_frozen_path_execution"));
    Report->SetStringField(TEXT("evidence_kind"),TEXT("ue_game_world_supplied_depth_waypoint_character_movement"));
    Report->SetStringField(TEXT("executor"),TEXT("CharacterMovement_AddMovementInput"));
    Report->SetStringField(TEXT("position_feedback_source"),TEXT("engine_ground_truth_for_actuation"));
    Report->SetStringField(TEXT("execution_scope"),Input->GetStringField(TEXT("execution_scope")));
    Report->SetStringField(TEXT("manifest_sha256"),Input->GetStringField(TEXT("manifest_sha256")));
    Report->SetStringField(TEXT("attempt_id"),AttemptId);Report->SetStringField(TEXT("map"),GetWorld()->GetOutermost()->GetName());
    Report->SetStringField(TEXT("position_reference"),TEXT("capsule_feet_cm"));
    Report->SetStringField(TEXT("collision_definition"),TEXT("blocking non-floor impacts, 0.1 simulated-s debounce"));
    Report->SetBoolField(TEXT("finished"),Finished);Report->SetNumberField(TEXT("expected_runs"),Runs.Num());Report->SetArrayField(TEXT("runs"),Results);
    FString Text;FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Text));
    const FString Temporary=ResultPath+TEXT(".tmp");
    if (!FFileHelper::SaveStringToFile(Text,*Temporary) || !IFileManager::Get().Move(*ResultPath,*Temporary,true))
    { Done=true;UE_LOG(LogTemp,Error,TEXT("SITP_DEPTH_RESULT_WRITE_FAILED"));FPlatformMisc::RequestExitWithStatus(false,3); }
}
