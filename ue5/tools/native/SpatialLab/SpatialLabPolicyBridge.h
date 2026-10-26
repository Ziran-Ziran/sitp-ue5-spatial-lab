#pragma once
#include "CoreMinimal.h"
#include "Dom/JsonObject.h"

/** Calls the same observed-only Python policy used by offline unit tests. */
class FSpatialLabPolicyBridge
{
public:
    bool Initialize(const FString& ToolsDirectory, const TSharedPtr<FJsonObject>& Mission,
                    const FString& ResponsePath, const FString& Attempt);
    bool Step(const TSharedPtr<FJsonObject>& Frame, const TSharedPtr<FJsonObject>& Feedback,
              TSharedPtr<FJsonObject>& Action);
private:
    FString Path;
    FString AttemptId;
};
