#include "SpatialLabOnlineDepthRunner.h"
#include "SpatialLabRgbd.h"
#include "AIController.h"
#include "Components/CapsuleComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#if WITH_EDITOR
#include "IPythonScriptPlugin.h"
#endif

namespace
{
    FVector Vector(const TArray<TSharedPtr<FJsonValue>>& V)
    {
        double X,Y,Z;
        if (V.Num()!=3 || !V[0]->TryGetNumber(X) || !V[1]->TryGetNumber(Y) || !V[2]->TryGetNumber(Z)) return FVector(NAN);
        return FVector(X,Y,Z);
    }
    TSharedPtr<FJsonValue> JsonVector(FVector P)
    {
        return MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{
            MakeShared<FJsonValueNumber>(P.X),MakeShared<FJsonValueNumber>(P.Y),MakeShared<FJsonValueNumber>(P.Z)});
    }
    FString Json(const TSharedPtr<FJsonObject>& Object)
    {
        FString Text;FJsonSerializer::Serialize(Object.ToSharedRef(),TJsonWriterFactory<>::Create(&Text));return Text;
    }
    FString Quote(const FString& Text)
    {
        FString Result;auto Writer=TJsonWriterFactory<>::Create(&Result);Writer->WriteValue(Text);Writer->Close();return Result;
    }
    bool Execute(const FString& Command)
    {
#if WITH_EDITOR
        auto* Python=FModuleManager::LoadModulePtr<IPythonScriptPlugin>(TEXT("PythonScriptPlugin"));
        if (!Python || (!Python->IsPythonInitialized() && !Python->ForceEnablePythonAtRuntime())) return false;
        FPythonCommandEx Request;Request.Flags=EPythonCommandFlags::Unattended;
        Request.FileExecutionScope=EPythonFileExecutionScope::Public;Request.Command=Command;
        return Python->ExecPythonCommandEx(Request);
#else
        return false;
#endif
    }
    double SegmentDistance(FVector P,FVector A,FVector B)
    {
        const FVector2D D(B.X-A.X,B.Y-A.Y),O(P.X-A.X,P.Y-A.Y);
        const double T=D.SizeSquared()>1e-12 ? FMath::Clamp(FVector2D::DotProduct(O,D)/D.SizeSquared(),0.,1.) : 0;
        return (O-D*T).Size();
    }
}

ASpatialLabOnlineDepthRunner::ASpatialLabOnlineDepthRunner()
{
    PrimaryActorTick.bCanEverTick=true;PrimaryActorTick.TickGroup=TG_PostPhysics;
}

void ASpatialLabOnlineDepthRunner::BeginPlay()
{
    Super::BeginPlay();FString Path,Text;
    const TArray<TSharedPtr<FJsonValue>>* Rows=nullptr;
    VisualTask=FParse::Value(FCommandLine::Get(),TEXT("SITPVisualMarker="),Path);
    bool Valid=(VisualTask || FParse::Value(FCommandLine::Get(),TEXT("SITPOnlineDepth="),Path))
        && FParse::Value(FCommandLine::Get(),TEXT("SITPResult="),ResultPath)
        && FParse::Value(FCommandLine::Get(),TEXT("SITPOnlineOutput="),OutputDirectory)
        && FParse::Value(FCommandLine::Get(),TEXT("SITPTools="),ToolsDirectory)
        && FParse::Value(FCommandLine::Get(),TEXT("SITPPython="),PythonExecutable)
        && !IFileManager::Get().FileExists(*ResultPath) && !IFileManager::Get().DirectoryExists(*OutputDirectory)
        && FFileHelper::LoadFileToString(Text,*Path)
        && FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Input) && Input.IsValid();
    if (Valid)
    {
        const int32 Version=Input->GetIntegerField(TEXT("schema_version"));
        Valid=(Version==1 || Version==2 || (Version==3 && VisualTask))
            && Input->GetStringField(TEXT("condition"))==(VisualTask?TEXT("rgbd_color_marker_target_search_pose_actuation_supervision"):TEXT("online_stopped_depth_geometry_pose_supervision"))
            && Input->GetStringField(TEXT("map"))==GetWorld()->GetOutermost()->GetName()
            && Input->TryGetArrayField(TEXT("runs"),Rows) && Rows && Rows->Num()>0 && Rows->Num()<=200;
        if (Valid) for (auto& Value:*Rows)
        {
            auto Row=Value->AsObject();FString Mode;
            if (!Row.IsValid() || !Row->TryGetStringField(TEXT("mode"),Mode)
                || (VisualTask ? Mode!=TEXT("marker_receding") : (Mode!=TEXT("initial_sweep_once") && Mode!=TEXT("receding_sweep")))
                || Vector(Row->GetArrayField(TEXT("start_cm"))).ContainsNaN()
                || (!VisualTask && Vector(Row->GetArrayField(TEXT("goal_cm"))).ContainsNaN())
                || !FMath::IsFinite(Row->GetNumberField(TEXT("heading_deg")))) Valid=false;
            if (Valid && Version>=2)
            {
                FString Variant,TopVariant;
                const TCHAR* Key=VisualTask?TEXT("planner_variant"):TEXT("query_cell_policy");
                Valid=Row->TryGetStringField(Key,Variant) && Input->TryGetStringField(Key,TopVariant) && Variant==TopVariant
                    && (VisualTask ? (Variant==TEXT("legacy_centers") || Variant==TEXT("continuous_square") || Variant==TEXT("continuous_square_shortcuts"))
                                   : (Variant==TEXT("legacy_floor") || Variant==TEXT("boundary_snap_1e-9_cell")));
            }
            if (Valid && Version==3)
            {
                FString Horizon,TopHorizon;
                Valid=Row->TryGetStringField(TEXT("goal_horizon_policy"),Horizon)
                    && Input->TryGetStringField(TEXT("goal_horizon_policy"),TopHorizon) && Horizon==TopHorizon
                    && (Horizon==TEXT("fixed_three_meters") || Horizon==TEXT("execution_prefix_one_meter"));
            }
        }
    }
    if (!Valid) { Done=true;UE_LOG(LogTemp,Error,TEXT("SITP_ONLINE_DEPTH_INPUT_REJECTED"));FPlatformMisc::RequestExitWithStatus(false,2);return; }
    auto* Extended=IConsoleManager::Get().FindConsoleVariable(TEXT("r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange"));
    if (!Extended) { Done=true;FPlatformMisc::RequestExitWithStatus(false,2);return; }
    Extended->Set(1,ECVF_SetByCode);
    if (!IFileManager::Get().MakeDirectory(*OutputDirectory,true)) { Done=true;FPlatformMisc::RequestExitWithStatus(false,3);return; }
    for (TActorIterator<AActor> It(GetWorld());It;++It)
    {
        if (It->ActorHasTag(TEXT("SpatialLabInterventionWall"))) InterventionWall=*It;
        // Fixture interventions only. No actor names, positions or labels are
        // read for perception/goal selection or included in worker requests.
        if (It->ActorHasTag(TEXT("SpatialLabMarkerRed"))) RedMarkers.Add(*It);
        if (It->ActorHasTag(TEXT("SpatialLabMarkerBlue"))) BlueMarkers.Add(*It);
    }
    Runs=*Rows;AttemptId=FGuid::NewGuid().ToString(EGuidFormats::DigitsWithHyphens).ToLower();
    NextStart=GetWorld()->GetTimeSeconds()+1;Save(false);
}

FVector ASpatialLabOnlineDepthRunner::Feet() const
{
    return Agent ? Agent->GetActorLocation()-FVector(0,0,Agent->GetCapsuleComponent()->GetScaledCapsuleHalfHeight()) : Previous;
}

void ASpatialLabOnlineDepthRunner::BeginNext()
{
    if (++Index>=Runs.Num())
    {
        Done=true;Save(true);UE_LOG(LogTemp,Display,TEXT("SITP_ONLINE_DEPTH_COMPLETE runs=%d"),Results.Num());
        FPlatformMisc::RequestExitWithStatus(false,0);return;
    }
    Current=Runs[Index]->AsObject();RequestedStart=Vector(Current->GetArrayField(TEXT("start_cm")));
    Goal=VisualTask?RequestedStart:Vector(Current->GetArrayField(TEXT("goal_cm")));
    CurrentHeading=Current->GetNumberField(TEXT("heading_deg"));EmptyViews=0;PendingObservation=false;
    Previous=InitialFeet=RequestedStart;StartedSim=GetWorld()->GetTimeSeconds();StartedWall=FPlatformTime::Seconds();
    ResetWall=StartedWall;SpawnedSim=StartedSim;Measured=false;Cursor=0;Collisions=CaptureCount=0;WalkedCm=MaxTracking=0;
    ResetXY=ResetZ=CaptureWall=PolicyWall=0;InterventionSim=-1;LastCollision=-1;
    Trajectory.Empty();Actions.Empty();WaypointEvents.Empty();Waypoints.Empty();
    if (InterventionWall) { InterventionWall->SetActorEnableCollision(false);InterventionWall->SetActorHiddenInGame(true); }
    if (!VisualTask && Current->GetStringField(TEXT("intervention"))==TEXT("wall_after_50cm") && !InterventionWall)
    { Finish(TEXT("intervention_missing"));return; }
    if (VisualTask)
    {
        const FString Case=Current->GetStringField(TEXT("environment_case"));
        if (Case!=TEXT("all_markers") && Case!=TEXT("red_hidden") && Case!=TEXT("wall_on")) { Finish(TEXT("invalid_fixture_case"));return; }
        for (AActor* Marker:RedMarkers) if (IsValid(Marker))
        { Marker->SetActorHiddenInGame(Case==TEXT("red_hidden"));Marker->SetActorEnableCollision(Case!=TEXT("red_hidden")); }
        for (AActor* Marker:BlueMarkers) if (IsValid(Marker)) { Marker->SetActorHiddenInGame(false);Marker->SetActorEnableCollision(true); }
        if (Case==TEXT("wall_on"))
        {
            if (!InterventionWall) { Finish(TEXT("intervention_missing"));return; }
            InterventionWall->SetActorEnableCollision(true);InterventionWall->SetActorHiddenInGame(false);
        }
    }
    const FVector Center=RequestedStart+FVector(0,0,90);const auto Shape=FCollisionShape::MakeCapsule(34,88);
    FHitResult Hit;
    const bool Blocked=GetWorld()->OverlapBlockingTestByChannel(Center,FQuat::Identity,ECC_Pawn,Shape)
        || (GetWorld()->SweepSingleByChannel(Hit,Center,Center+FVector(.01,0,0),FQuat::Identity,ECC_Pawn,Shape) && Hit.bStartPenetrating);
    if (Blocked) { Finish(TEXT("spawn_blocked"));return; }
    const FTransform Transform(FRotator(0,Current->GetNumberField(TEXT("heading_deg")),0),Center);
    Agent=GetWorld()->SpawnActorDeferred<ACharacter>(ACharacter::StaticClass(),Transform,nullptr,nullptr,ESpawnActorCollisionHandlingMethod::AlwaysSpawn);
    if (!Agent) { Finish(TEXT("spawn_failed"));return; }
    Agent->GetCapsuleComponent()->InitCapsuleSize(34,88);Agent->GetCapsuleComponent()->SetCanEverAffectNavigation(false);
    Agent->GetCapsuleComponent()->SetNotifyRigidBodyCollision(true);
    auto* Movement=Agent->GetCharacterMovement();Movement->MaxWalkSpeed=300;Movement->MaxStepHeight=35;Movement->SetWalkableFloorAngle(45);
    Movement->bRunPhysicsWithNoController=true;
    Agent->FinishSpawning(Transform);Agent->OnActorHit.AddDynamic(this,&ASpatialLabOnlineDepthRunner::OnImpact);
    Controller=GetWorld()->SpawnActor<AAIController>();if (!Controller) { Finish(TEXT("controller_spawn_failed"));return; }
    Controller->Possess(Agent);Settling=true;
}

void ASpatialLabOnlineDepthRunner::InitializeMovement()
{
    Settling=false;InitialFeet=Previous=Feet();ResetXY=FVector::Dist2D(InitialFeet,RequestedStart);ResetZ=FMath::Abs(InitialFeet.Z-RequestedStart.Z);
    Measured=true;StartedSim=GetWorld()->GetTimeSeconds();StartedWall=FPlatformTime::Seconds();ResetWall=StartedWall-ResetWall;
    Trajectory.Add(JsonVector(InitialFeet));Active=true;
    if (ResetXY>5 || ResetZ>10) { Finish(TEXT("reset_pose_drift"));return; }
    ObserveAndPlan();
}

void ASpatialLabOnlineDepthRunner::ObserveAndPlan()
{
    Agent->GetCharacterMovement()->StopMovementImmediately();
    const FVector Position=Feet();
    if (!VisualTask && FVector::Dist2D(Position,Goal)<=10 && FMath::Abs(Position.Z-Goal.Z)<=20) { Finish(TEXT("arrived"));return; }
    if (CaptureCount>=(VisualTask?12:8)) { Finish(TEXT("capture_budget"));return; }
    const int32 Step=CaptureCount++;const double Sim=GetWorld()->GetTimeSeconds()-StartedSim;
    const FString Folder=OutputDirectory/FString::Printf(TEXT("run_%03d/step_%03d"),Index,Step);
    const FString Pixels=Folder/TEXT("input"),RequestPath=Folder/TEXT("request.json"),ResponsePath=Folder/TEXT("response.json");
    TSharedRef<FJsonObject> Event=MakeShared<FJsonObject>();Event->SetNumberField(TEXT("step_id"),Step);
    Event->SetNumberField(TEXT("t_sim_s"),Sim);Event->SetField(TEXT("feet_cm"),JsonVector(Position));Event->SetStringField(TEXT("folder"),Folder);
    const double Heading=CurrentHeading;Event->SetNumberField(TEXT("heading_deg"),Heading);
    const FVector Eye=Position+FVector(0,0,150);
    TArray<TSharedPtr<FJsonValue>> Captures;const TCHAR* Names[]={TEXT("level"),TEXT("down45"),TEXT("nadir")};const double Pitches[]={0,45,90};
    bool Captured=true;const double CaptureStarted=FPlatformTime::Seconds();
    for (int32 View=0;View<3;++View)
    {
        const FString Directory=Pixels/Names[View];IFileManager::Get().MakeDirectory(*Directory,true);
        FString Text=USpatialLabRgbd::CapturePose(GetWorld(),Eye,FRotator(-Pitches[View],Heading,0),
            Directory/TEXT("rgb.png"),Directory/TEXT("depth.png"),Input->GetNumberField(TEXT("fixed_exposure_ev100")));
        TSharedPtr<FJsonObject> Native;
        if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Native) || !Native.IsValid()) { Captured=false;break; }
        Native->SetStringField(TEXT("view"),Names[View]);Captures.Add(MakeShared<FJsonValueObject>(Native));
        Captured&=Native->GetBoolField(TEXT("captured"));
    }
    const double Elapsed=FPlatformTime::Seconds()-CaptureStarted;CaptureWall+=Elapsed;
    Event->SetNumberField(TEXT("capture_wall_s"),Elapsed);Event->SetArrayField(TEXT("captures"),Captures);
    if (!Captured) { Event->SetStringField(TEXT("status"),TEXT("capture_failed"));Actions.Add(MakeShared<FJsonValueObject>(Event));Finish(TEXT("capture_failed"));return; }
    const double Angle=FMath::DegreesToRadians(Heading);const FVector Delta=(Goal-Position)/100;
    TSharedRef<FJsonObject> Request=MakeShared<FJsonObject>();Request->SetNumberField(TEXT("schema_version"),Input->GetIntegerField(TEXT("schema_version")));
    const TCHAR* PolicyKey=VisualTask?TEXT("planner_variant"):TEXT("query_cell_policy");
    if (Input->GetIntegerField(TEXT("schema_version"))>=2) Request->SetStringField(PolicyKey,Current->GetStringField(PolicyKey));
    if (Input->GetIntegerField(TEXT("schema_version"))==3) Request->SetStringField(TEXT("goal_horizon_policy"),Current->GetStringField(TEXT("goal_horizon_policy")));
    Request->SetStringField(TEXT("attempt_id"),AttemptId);Request->SetStringField(TEXT("run_id"),Current->GetStringField(TEXT("run_id")));
    Request->SetNumberField(TEXT("step_id"),Step);Request->SetNumberField(TEXT("t_sim_s"),Sim);
    Request->SetStringField(TEXT("input_dir"),Pixels);
    if (VisualTask)
    {
        Request->SetStringField(TEXT("category"),Current->GetStringField(TEXT("category")));
        Request->SetNumberField(TEXT("consecutive_empty_views"),EmptyViews);
    }
    else
    {
        Request->SetStringField(TEXT("mode"),Current->GetStringField(TEXT("mode")));
        Request->SetArrayField(TEXT("goal_right_forward_m"),{MakeShared<FJsonValueNumber>(-Delta.X*FMath::Sin(Angle)+Delta.Y*FMath::Cos(Angle)),
            MakeShared<FJsonValueNumber>(Delta.X*FMath::Cos(Angle)+Delta.Y*FMath::Sin(Angle))});
    }
    Event->SetObjectField(TEXT("request"),Request);
    if (!FFileHelper::SaveStringToFile(Json(Request),*RequestPath)) { Finish(TEXT("request_io_failed"));return; }
    // Embedded Python only launches the isolated numerical worker; no unreal or
    // evaluation state enters its request. Parent Job Object owns the child tree.
    const FString Command=TEXT("import subprocess\n_sitp_depth_child=subprocess.run([")+Quote(PythonExecutable)+TEXT(",")
        +Quote(ToolsDirectory/(VisualTask?TEXT("visual_marker_depth_policy.py"):TEXT("online_depth_policy.py")))+TEXT(",'--request',")+Quote(RequestPath)+TEXT(",'--out',")
        +Quote(ResponsePath)+TEXT(",'--prepare-observation'],capture_output=True,text=True,timeout=20)\n")
        +TEXT("assert _sitp_depth_child.returncode==0, _sitp_depth_child.stderr[-4000:]");
    const double PolicyStarted=FPlatformTime::Seconds();FString ResponseText;TSharedPtr<FJsonObject> Response;
    const bool Replied=Execute(Command) && FFileHelper::LoadFileToString(ResponseText,*ResponsePath)
        && FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(ResponseText),Response) && Response.IsValid();
    const double BridgeElapsed=FPlatformTime::Seconds()-PolicyStarted;PolicyWall+=BridgeElapsed;Event->SetNumberField(TEXT("bridge_wall_s"),BridgeElapsed);
    if (!Replied || Response->GetStringField(TEXT("attempt_id"))!=AttemptId
        || Response->GetStringField(TEXT("run_id"))!=Current->GetStringField(TEXT("run_id"))
        || Response->GetIntegerField(TEXT("step_id"))!=Step || Response->GetNumberField(TEXT("t_sim_s"))!=Sim
        || Response->GetIntegerField(TEXT("schema_version"))!=Input->GetIntegerField(TEXT("schema_version"))
        || (Input->GetIntegerField(TEXT("schema_version"))>=2 && Response->GetStringField(PolicyKey)!=Current->GetStringField(PolicyKey))
        || (Input->GetIntegerField(TEXT("schema_version"))==3 && Response->GetStringField(TEXT("goal_horizon_policy"))!=Current->GetStringField(TEXT("goal_horizon_policy"))))
    { Event->SetStringField(TEXT("status"),TEXT("policy_bridge_failed"));Actions.Add(MakeShared<FJsonValueObject>(Event));Finish(TEXT("policy_bridge_failed"));return; }
    Event->SetObjectField(TEXT("response"),Response);Event->SetStringField(TEXT("status"),TEXT("complete"));Actions.Add(MakeShared<FJsonValueObject>(Event));
    if (FPlatformTime::Seconds()-StartedWall>=60) { Finish(TEXT("wall_timeout"));return; }
    if (VisualTask)
    {
        const FString Action=Response->GetStringField(TEXT("action"));
        if (Action==TEXT("candidate_reached")) { Finish(TEXT("candidate_reached"));return; }
        if (Action==TEXT("stop")) { Finish(Response->GetStringField(TEXT("reason")));return; }
        if (Action==TEXT("rotate"))
        {
            if (Response->GetNumberField(TEXT("relative_yaw_delta_deg"))!=90 || EmptyViews>=3) { Finish(TEXT("invalid_visual_scan"));return; }
            ++EmptyViews;CurrentHeading=FMath::UnwindDegrees(CurrentHeading+90);PendingObservation=true;
            return;
        }
        EmptyViews=0;
    }
    if (Response->GetStringField(TEXT("action"))!=TEXT("move")) { Finish(TEXT("geometry_")+Response->GetStringField(TEXT("reason")));return; }
    Waypoints.Empty();Cursor=0;RouteStart=Position;
    const auto& Local=Response->GetArrayField(TEXT("waypoints_right_forward_m"));
    if (Local.Num()<1 || Local.Num()>256) { Finish(TEXT("invalid_policy_route"));return; }
    double Length=0;FVector Prior=Position;
    for (const auto& Item:Local)
    {
        const auto& P=Item->AsArray();
        if (P.Num()!=2 || !FMath::IsFinite(P[0]->AsNumber()) || !FMath::IsFinite(P[1]->AsNumber())) { Finish(TEXT("invalid_policy_route"));return; }
        const double R=P[0]->AsNumber(),F=P[1]->AsNumber();
        const FVector World=Position+FVector(100*(F*FMath::Cos(Angle)-R*FMath::Sin(Angle)),100*(F*FMath::Sin(Angle)+R*FMath::Cos(Angle)),0);
        Length+=FVector::Distance(Prior,World);Prior=World;Waypoints.Add(World);
    }
    if (FVector::Dist2D(Waypoints[0],Position)>.001 || Length>2000
        || ((VisualTask || Current->GetStringField(TEXT("mode"))==TEXT("receding_sweep")) && Length>100.001)) { Finish(TEXT("invalid_policy_route"));return; }
    LastProgress=GetWorld()->GetTimeSeconds();BestDistance=TNumericLimits<double>::Max();Drive();
}

void ASpatialLabOnlineDepthRunner::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);if (Done) return;
    if (Settling) { if (!IsValid(Agent)) Finish(TEXT("reset_agent_lost"));else if (GetWorld()->GetTimeSeconds()-SpawnedSim>=.25) InitializeMovement();return; }
    if (!Active) { if (GetWorld()->GetTimeSeconds()>=NextStart) BeginNext();return; }
    if (!IsValid(Agent) || Feet().ContainsNaN()) { Finish(TEXT("agent_lost"));return; }
    const FVector P=Feet();WalkedCm+=FVector::Distance(Previous,P);Previous=P;Trajectory.Add(JsonVector(P));
    if (!VisualTask && InterventionWall && InterventionSim<0 && Current->GetStringField(TEXT("intervention"))==TEXT("wall_after_50cm") && WalkedCm>=50)
    { InterventionWall->SetActorHiddenInGame(false);InterventionWall->SetActorEnableCollision(true);InterventionSim=GetWorld()->GetTimeSeconds()-StartedSim; }
    if (WalkedCm>2000) Finish(TEXT("path_budget"));
    else if (GetWorld()->GetTimeSeconds()-StartedSim>=30) Finish(TEXT("timeout"));
    else if (FPlatformTime::Seconds()-StartedWall>=60) Finish(TEXT("wall_timeout"));
    else if (FMath::Abs(P.Z-InitialFeet.Z)>50) Finish(TEXT("vertical_excursion"));
    else if (PendingObservation) { PendingObservation=false;ObserveAndPlan(); }
    else Drive();
}

void ASpatialLabOnlineDepthRunner::Drive()
{
    const FVector P=Feet();const double Now=GetWorld()->GetTimeSeconds();
    while (Cursor<Waypoints.Num() && FVector::Dist2D(P,Waypoints[Cursor])<=5)
    {
        Agent->GetCharacterMovement()->StopMovementImmediately();TSharedRef<FJsonObject> Event=MakeShared<FJsonObject>();
        Event->SetNumberField(TEXT("step_id"),CaptureCount-1);Event->SetNumberField(TEXT("waypoint_index"),Cursor);
        Event->SetNumberField(TEXT("t_sim_s"),Now-StartedSim);Event->SetField(TEXT("position_cm"),JsonVector(P));
        WaypointEvents.Add(MakeShared<FJsonValueObject>(Event));++Cursor;LastProgress=Now;BestDistance=TNumericLimits<double>::Max();
    }
    if (Cursor==Waypoints.Num())
    {
        if (VisualTask) { PendingObservation=true;Waypoints.Empty();Cursor=0; }
        else if (FVector::Dist2D(P,Goal)<=10 && FMath::Abs(P.Z-Goal.Z)<=20) Finish(TEXT("arrived"));
        else if (Current->GetStringField(TEXT("mode"))==TEXT("receding_sweep")) ObserveAndPlan();
        else Finish(TEXT("route_exhausted"));return;
    }
    const double Error=SegmentDistance(P,Cursor>0?Waypoints[Cursor-1]:RouteStart,Waypoints[Cursor]);MaxTracking=FMath::Max(MaxTracking,Error);
    if (Error>10) { Finish(TEXT("tracking_deviation"));return; }
    const double Distance=FVector::Dist2D(P,Waypoints[Cursor]);
    if (Distance<BestDistance-1) { BestDistance=Distance;LastProgress=Now; }
    if (Now-LastProgress>=2) { Finish(TEXT("blocked_no_progress"));return; }
    FVector Direction=Waypoints[Cursor]-P;Direction.Z=0;Agent->AddMovementInput(Direction.GetSafeNormal(),1,true);
}

void ASpatialLabOnlineDepthRunner::OnImpact(AActor* Self,AActor* Other,FVector Impulse,const FHitResult& Hit)
{
    const double Now=GetWorld()->GetTimeSeconds();if (Active && Hit.bBlockingHit && Hit.ImpactNormal.Z<.7071 && Now-LastCollision>=.1)
    { ++Collisions;LastCollision=Now; }
}

void ASpatialLabOnlineDepthRunner::Finish(const FString& Status)
{
    TSharedRef<FJsonObject> Execution=MakeShared<FJsonObject>();Execution->SetStringField(TEXT("status"),Status);
    Execution->SetBoolField(TEXT("agent_spawned"),Agent!=nullptr);Execution->SetBoolField(TEXT("measurement_started"),Measured);
    Execution->SetField(TEXT("initial_position_cm"),Measured?JsonVector(InitialFeet):MakeShared<FJsonValueNull>());
    Execution->SetField(TEXT("final_position_cm"),Agent?JsonVector(Feet()):MakeShared<FJsonValueNull>());
    Execution->SetArrayField(TEXT("trajectory_cm"),Trajectory);Execution->SetArrayField(TEXT("actions"),Actions);
    Execution->SetArrayField(TEXT("waypoint_events"),WaypointEvents);Execution->SetNumberField(TEXT("capture_count"),CaptureCount);
    if (Measured)
    {
        Execution->SetNumberField(TEXT("continuous_path_length_cm"),WalkedCm);Execution->SetNumberField(TEXT("collisions"),Collisions);
        Execution->SetNumberField(TEXT("sim_s"),GetWorld()->GetTimeSeconds()-StartedSim);Execution->SetNumberField(TEXT("wall_s"),FPlatformTime::Seconds()-StartedWall);
        Execution->SetNumberField(TEXT("reset_wall_s"),ResetWall);Execution->SetNumberField(TEXT("reset_xy_cm"),ResetXY);Execution->SetNumberField(TEXT("reset_z_cm"),ResetZ);
        Execution->SetNumberField(TEXT("capture_wall_s"),CaptureWall);Execution->SetNumberField(TEXT("policy_bridge_wall_s"),PolicyWall);
        Execution->SetNumberField(TEXT("max_tracking_error_cm"),MaxTracking);
    }
    else for (const TCHAR* Key:{TEXT("continuous_path_length_cm"),TEXT("collisions"),TEXT("sim_s"),TEXT("wall_s"),TEXT("reset_wall_s"),
        TEXT("reset_xy_cm"),TEXT("reset_z_cm"),TEXT("capture_wall_s"),TEXT("policy_bridge_wall_s"),TEXT("max_tracking_error_cm")}) Execution->SetField(Key,MakeShared<FJsonValueNull>());
    Execution->SetNumberField(TEXT("engine_path_requests"),0);Execution->SetNumberField(TEXT("intervention_sim_s"),InterventionSim);
    TSharedRef<FJsonObject> Row=MakeShared<FJsonObject>();Row->SetStringField(TEXT("run_id"),Current->GetStringField(TEXT("run_id")));
    Row->SetObjectField(TEXT("input"),Current);Row->SetObjectField(TEXT("execution"),Execution);Results.Add(MakeShared<FJsonValueObject>(Row));
    UE_LOG(LogTemp,Display,TEXT("SITP_ONLINE_DEPTH_RUN index=%d status=%s captures=%d path_cm=%.3f"),Index,*Status,CaptureCount,WalkedCm);
    Active=Settling=PendingObservation=false;if (Agent) Agent->GetCharacterMovement()->StopMovementImmediately();
    if (Controller) { Controller->UnPossess();Controller->Destroy();Controller=nullptr; }if (Agent) { Agent->Destroy();Agent=nullptr; }
    NextStart=GetWorld()->GetTimeSeconds()+.1;Save(false);
}

void ASpatialLabOnlineDepthRunner::Save(bool Finished)
{
    TSharedRef<FJsonObject> Report=MakeShared<FJsonObject>();Report->SetNumberField(TEXT("schema_version"),Input->GetIntegerField(TEXT("schema_version")));
    Report->SetStringField(TEXT("condition"),Input->GetStringField(TEXT("condition")));
    Report->SetStringField(TEXT("evidence_kind"),VisualTask?TEXT("ue_game_world_rgb_marker_depth_target_search"):TEXT("ue_game_world_stopped_rgbd_observation_dependent_direct_motion"));
    Report->SetStringField(TEXT("executor"),TEXT("CharacterMovement_AddMovementInput"));Report->SetStringField(TEXT("position_reference"),TEXT("capsule_feet_cm"));
    Report->SetStringField(TEXT("pose_source"),VisualTask?TEXT("engine_ground_truth_reset_and_actuation_only"):TEXT("engine_ground_truth_reset_relative_goal_and_actuation"));
    Report->SetStringField(TEXT("attempt_id"),AttemptId);Report->SetStringField(TEXT("manifest_sha256"),Input->GetStringField(TEXT("manifest_sha256")));
    Report->SetStringField(TEXT("scope"),Input->GetStringField(TEXT("scope")));Report->SetStringField(TEXT("map"),GetWorld()->GetOutermost()->GetName());
    Report->SetBoolField(TEXT("finished"),Finished);Report->SetNumberField(TEXT("expected_runs"),Runs.Num());Report->SetArrayField(TEXT("runs"),Results);
    const FString Temporary=ResultPath+TEXT(".tmp");
    if (!FFileHelper::SaveStringToFile(Json(Report),*Temporary) || !IFileManager::Get().Move(*ResultPath,*Temporary,true))
    { Done=true;FPlatformMisc::RequestExitWithStatus(false,3); }
}
