#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Dom/JsonObject.h"
#include "SpatialLabDepthRunner.generated.h"

class ACharacter;
class AAIController;

/** Frozen depth waypoints, engine-pose actuation feedback, no NavMesh path calls. */
UCLASS()
class SPATIALLAB_API ASpatialLabDepthRunner : public AActor
{
    GENERATED_BODY()
public:
    ASpatialLabDepthRunner();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
private:
    UPROPERTY() TObjectPtr<ACharacter> Agent;
    UPROPERTY() TObjectPtr<AAIController> Controller;
    TSharedPtr<FJsonObject> Input,Current,Settings;
    TArray<TSharedPtr<FJsonValue>> Runs,Results,Trajectory,WaypointEvents;
    TArray<FVector> Waypoints;
    FString ResultPath,AttemptId;
    int32 Index=-1,Cursor=0,Collisions=0;
    double StartedSim=0,StartedWall=0,SpawnedSim=0,ResetWall=0,NextStart=1,LastCollision=-1;
    double WalkedCm=0,MaxTracking=0,LastProgress=0,BestDistance=0,ResetXY=0,ResetZ=0;
    FVector RequestedStart=FVector::ZeroVector,InitialFeet=FVector::ZeroVector,Previous=FVector::ZeroVector;
    bool Active=false,Settling=false,Measured=false,Done=false;
    FVector Feet() const;
    void BeginNext();
    void InitializeMovement();
    void Drive();
    void Finish(const FString& Status);
    void Save(bool Finished);
    UFUNCTION() void OnImpact(AActor* Self,AActor* Other,FVector Impulse,const FHitResult& Hit);
};
