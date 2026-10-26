#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SpatialLabNavigation.generated.h"

/** Explicit privileged engine baseline helpers. Never a visual perception API. */
UCLASS()
class SPATIALLAB_API USpatialLabNavigation : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    /** Build one bounded editor NavMesh for a fixed capsule. Returns JSON evidence. */
    UFUNCTION(BlueprintCallable, Category="SpatialLab|Oracle", meta=(WorldContext="WorldContext"))
    static FString BuildOracleNavigation(UObject* WorldContext, FVector Center, FVector Extent,
                                         float RadiusCm=34.f, float HeightCm=176.f);
};
