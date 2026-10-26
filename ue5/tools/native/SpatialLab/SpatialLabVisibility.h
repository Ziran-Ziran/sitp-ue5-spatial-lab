#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

/** Geometry-ray visibility supervision. No image recognizer or full-catalog export. */
class FSpatialLabVisibilityRecorder
{
public:
    bool Open(UWorld* World, const FString& Path, const FString& ManifestHash, const FString& AttemptId,
              const FString& RouteSource=TEXT("global_engine_oracle_selected_routes"));
    void BeginEpisode(const FString& RunId, double SimStart);
    void Sample(AActor* Observer, FVector Feet, bool Force=false);
    void EndEpisode(const FString& Status);
    bool Finish();
    bool IsHealthy() const { return Healthy; }
    TSharedPtr<FJsonObject> GetLatestFrame() const { return LatestFrame; }
private:
    TWeakObjectPtr<UWorld> World;
    TArray<TWeakObjectPtr<AActor>> Targets;
    TMap<TWeakObjectPtr<AActor>, FString> Categories;
    TUniquePtr<FArchive> Archive;
    FString RunId;
    double EpisodeStart = 0;
    double StartedWall = 0;
    double LastSim = -1;
    int32 FrameIndex = 0;
    int32 TotalFrames = 0;
    int32 Episodes = 0;
    bool Healthy = true;
    bool Closed = false;
    TSharedPtr<FJsonObject> LatestFrame;
    void Write(const TSharedRef<FJsonObject>& Record);
};
