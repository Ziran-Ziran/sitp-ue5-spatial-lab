#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Dom/JsonObject.h"
#include "SpatialLabOnlineDepthRunner.generated.h"

class ACharacter;
class AAIController;

/** Actual stopped RGB-D -> estimated route -> direct motion -> new observation. */
UCLASS()
class SPATIALLAB_API ASpatialLabOnlineDepthRunner : public AActor
{
    GENERATED_BODY()
public:
    ASpatialLabOnlineDepthRunner();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    UFUNCTION(BlueprintPure,Category="SpatialLab|Capabilities")
    static bool SupportsVisualMarkerTasks() { return true; }
private:
    UPROPERTY() TObjectPtr<ACharacter> Agent;
    UPROPERTY() TObjectPtr<AAIController> Controller;
    UPROPERTY() TObjectPtr<AActor> InterventionWall;
    UPROPERTY() TArray<TObjectPtr<AActor>> RedMarkers;
    UPROPERTY() TArray<TObjectPtr<AActor>> BlueMarkers;
    TSharedPtr<FJsonObject> Input,Current;
    TArray<TSharedPtr<FJsonValue>> Runs,Results,Trajectory,Actions,WaypointEvents;
    TArray<FVector> Waypoints;
    FString ResultPath,AttemptId,OutputDirectory,ToolsDirectory,PythonExecutable;
    int32 Index=-1,Cursor=0,Collisions=0,CaptureCount=0;
    double StartedSim=0,StartedWall=0,SpawnedSim=0,ResetWall=0,NextStart=1,LastCollision=-1;
    double WalkedCm=0,MaxTracking=0,LastProgress=0,BestDistance=0,ResetXY=0,ResetZ=0;
    double CaptureWall=0,PolicyWall=0,InterventionSim=-1;
    double CurrentHeading=0;
    int32 EmptyViews=0;
    FVector RequestedStart=FVector::ZeroVector,InitialFeet=FVector::ZeroVector,Previous=FVector::ZeroVector;
    FVector Goal=FVector::ZeroVector,RouteStart=FVector::ZeroVector;
    bool Active=false,Settling=false,Measured=false,Done=false;
    bool VisualTask=false,PendingObservation=false;
    FVector Feet() const;
    void BeginNext();
    void InitializeMovement();
    void ObserveAndPlan();
    void Drive();
    void Finish(const FString& Status);
    void Save(bool Finished);
    UFUNCTION() void OnImpact(AActor* Self,AActor* Other,FVector Impulse,const FHitResult& Hit);
};
