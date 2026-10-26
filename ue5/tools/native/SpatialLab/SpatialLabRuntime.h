#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GameFramework/GameModeBase.h"
#include "Dom/JsonObject.h"
#include "AITypes.h"
#include "SpatialLabRuntime.generated.h"

class ACharacter;
class AAIController;
struct FPathFollowingResult;
class FSpatialLabVisibilityRecorder;

/** Headless experimental Game world, with no desktop input or recorder hooks. */
UCLASS()
class SPATIALLAB_API ASpatialLabGameMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASpatialLabGameMode();
    virtual void StartPlay() override;
};

/** Executes frozen engine-oracle decisions using real CharacterMovement + AI. */
UCLASS()
class SPATIALLAB_API ASpatialLabEpisodeRunner : public AActor
{
    GENERATED_BODY()
public:
    ASpatialLabEpisodeRunner();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
private:
    UPROPERTY() TObjectPtr<ACharacter> Agent;
    UPROPERTY() TObjectPtr<AAIController> Controller;
    TSharedPtr<FJsonObject> Plan;
    TSharedPtr<FJsonObject> Current;
    TArray<TSharedPtr<FJsonValue>> Runs;
    TArray<TSharedPtr<FJsonValue>> Results;
    TArray<TSharedPtr<FJsonValue>> Trajectory;
    FString ResultPath;
    FString AttemptId;
    int32 Index = -1;
    int32 CollisionEvents = 0;
    double StartedWall = 0;
    double StartedSim = 0;
    double NextStartSim = 1;
    double LastCollisionSim = -1;
    double MaxSim = 60;
    double MaxPath = 20000;
    double WalkedCm = 0;
    FVector Goal = FVector::ZeroVector;
    FVector Previous = FVector::ZeroVector;
    FVector InitialFeet = FVector::ZeroVector;
    bool Active = false;
    bool Done = false;
    FString LastMoveResult;
    TSharedPtr<FSpatialLabVisibilityRecorder> Visibility;
    void BeginNext();
    void FinishRun(const FString& Status);
    void Save(bool Finished);
    void OnMoveFinished(FAIRequestID Request, const FPathFollowingResult& Result);
    FVector Feet() const;
    UFUNCTION() void OnImpact(AActor* Self, AActor* Other, FVector Impulse, const FHitResult& Hit);
};
