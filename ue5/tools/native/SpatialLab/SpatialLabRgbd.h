#pragma once
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SpatialLabRgbd.generated.h"

/** Synchronous static-pose rendering. Not a live sensor or semantic recognizer. */
UCLASS()
class SPATIALLAB_API USpatialLabRgbd : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable,Category="SpatialLab|Replay",meta=(WorldContext="WorldContext"))
    static FString CapturePose(UObject* WorldContext,FVector CameraPosition,FRotator CameraRotation,
                               const FString& ColorPath,const FString& DepthPath,float FixedExposureEV100=1.f);
};
