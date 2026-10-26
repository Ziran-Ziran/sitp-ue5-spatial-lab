#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Dom/JsonObject.h"
#include "AITypes.h"
#include "SpatialLabSearch.generated.h"

class ACharacter;
class AAIController;
class FSpatialLabVisibilityRecorder;
class FSpatialLabPolicyBridge;
struct FPathFollowingResult;

/** Local-label/pose supervision plus engine-NavMesh geometry, observed-only policy. */
UCLASS()
class SPATIALLAB_API ASpatialLabSearchRunner : public AActor
{
    GENERATED_BODY()
public:
    ASpatialLabSearchRunner();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
private:
    UPROPERTY() TObjectPtr<ACharacter> Agent;
    UPROPERTY() TObjectPtr<AAIController> Controller;
    TSharedPtr<FJsonObject> Input;
    TSharedPtr<FJsonObject> Mission;
    TSharedPtr<FSpatialLabVisibilityRecorder> Visibility;
    TSharedPtr<FSpatialLabPolicyBridge> Policy;
    TArray<TSharedPtr<FJsonValue>> Missions,Results,Trajectory,Actions;
    FString ResultPath,ToolsDirectory,AttemptId,ObservationPath,FeedbackState,SelectedEntity;
    FAIRequestID MoveId;
    int32 Index=-1,LastFrame=-1,LastAction=-1,Collisions=0,NavigationRequests=0;
    double StartedSim=0,StartedWall=0,NextStart=1,LastCollision=-1,WalkedCm=0,PolicyWall=0;
    FVector Previous=FVector::ZeroVector,InitialFeet=FVector::ZeroVector;
    bool Active=false,Done=false;
    FVector Feet() const;
    void BeginNext();
    void Decide();
    void FinishRun(const FString& Status);
    void Save(bool Finished);
    void OnMoveFinished(FAIRequestID Request,const FPathFollowingResult& Result);
    UFUNCTION() void OnImpact(AActor* Self,AActor* Other,FVector Impulse,const FHitResult& Hit);
};
