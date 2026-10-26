#include "SpatialLabNavigation.h"
#include "NavigationSystem.h"
#include "NavigationData.h"
#include "NavMesh/NavMeshBoundsVolume.h"
#include "NavMesh/RecastNavMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Serialization/JsonSerializer.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#include "ActorFactories/ActorFactory.h"
#include "Builders/CubeBuilder.h"
#include "Components/BrushComponent.h"
#include "Containers/Ticker.h"
#include "UObject/UObjectGlobals.h"
#endif

FString USpatialLabNavigation::BuildOracleNavigation(UObject* WorldContext, FVector Center,
                                                     FVector Extent, float RadiusCm, float HeightCm)
{
    TSharedRef<FJsonObject> Result = MakeShared<FJsonObject>();
    Result->SetStringField(TEXT("condition"), TEXT("engine_navmesh_oracle"));
    Result->SetBoolField(TEXT("play_tested"), false);
#if WITH_EDITOR
    UWorld* World = WorldContext ? WorldContext->GetWorld() : nullptr;
    if (!World || World->IsGameWorld() || Center.ContainsNaN() || Extent.ContainsNaN()
        || Extent.GetMin() <= 0 || Extent.GetMax() > 50000 || RadiusCm <= 0 || HeightCm < RadiusCm * 2)
    {
        Result->SetStringField(TEXT("error"), TEXT("invalid editor world or bounded agent/volume"));
    }
    else
    {
        ANavMeshBoundsVolume* Volume = nullptr;
        for (TActorIterator<ANavMeshBoundsVolume> It(World); It; ++It)
            if (It->ActorHasTag(TEXT("SITP_OracleBounds"))) { Volume = *It; break; }
        if (!Volume)
        {
            Volume = World->SpawnActor<ANavMeshBoundsVolume>();
            Volume->Tags.Add(TEXT("SITP_OracleBounds"));
        }
        UCubeBuilder* Builder = NewObject<UCubeBuilder>(Volume);
        Builder->X = Extent.X * 2;
        Builder->Y = Extent.Y * 2;
        Builder->Z = Extent.Z * 2;
        Builder->Hollow = false;
        UActorFactory::CreateBrushForVolumeActor(Volume, Builder);
        if (Volume->GetComponentsBoundingBox(true).GetExtent().X < Extent.X * 0.99)
            Result->SetStringField(TEXT("error"), TEXT("navigation volume brush build failed"));
        else
        {
            Volume->SetActorLocation(Center);
            Volume->BrushBuilder = Builder;
            Volume->GetBrushComponent()->RecreatePhysicsState();
            Volume->PostEditChange();
            UNavigationSystemV1* Nav = FNavigationSystem::GetCurrent<UNavigationSystemV1>(World);
            if (!Nav)
                Result->SetStringField(TEXT("error"), TEXT("world has no navigation system"));
            else
            {
                const double Started = FPlatformTime::Seconds();
                // Python commandlets do not run the normal editor ticker. UE 5.8
                // releases its async-load navigation lock only after compilation
                // and a delayed ticker callback. Pump that lifecycle explicitly;
                // do not force-clear an arbitrary navigation build lock.
                FlushAsyncLoading();
                FAssetCompilingManager::Get().FinishAllCompilation();
                for (int32 Tick = 0; Tick < 32; ++Tick)
                    FTSTicker::GetCoreTicker().Tick(0.125f);
                Result->SetNumberField(TEXT("commandlet_initialization_ticks"), 32);
                Nav->OnNavigationBoundsUpdated(Volume);
                // Bounds updates and octree registration are queued until the
                // navigation tick. Building immediately otherwise yields zero tiles.
                Nav->Tick(0.016f);
                Result->SetNumberField(TEXT("volume_extent_x_cm"), Volume->GetComponentsBoundingBox(true).GetExtent().X);
                ANavigationData* Data = Nav->GetDefaultNavDataInstance(FNavigationSystem::Create);
                if (ARecastNavMesh* Recast = Cast<ARecastNavMesh>(Data))
                {
                    FNavDataConfig Config = Recast->GetConfig();
                    Config.AgentRadius = RadiusCm;
                    Config.AgentHeight = HeightCm;
                    Recast->SetConfig(Config);
                    Recast->AgentRadius = RadiusCm;
                    Recast->AgentHeight = HeightCm;
                    Recast->AgentMaxSlope = 45.f;
                    Nav->Build();
                    Recast->EnsureBuildCompletion();
                    Result->SetNumberField(TEXT("populated_tiles"), Recast->GetNumActiveTiles());
                    Result->SetNumberField(TEXT("agent_radius_cm"), Recast->AgentRadius);
                    Result->SetNumberField(TEXT("agent_height_cm"), Recast->AgentHeight);
                    Result->SetNumberField(TEXT("agent_max_slope_degrees"), Recast->AgentMaxSlope);
                    Result->SetNumberField(TEXT("agent_max_step_cm"), Recast->GetAgentMaxStepHeight(ENavigationDataResolution::Default));
                    Result->SetNumberField(TEXT("build_wall_s"), FPlatformTime::Seconds() - Started);
                    Result->SetStringField(TEXT("nav_data"), Recast->GetPathName());
                }
                else
                    Result->SetStringField(TEXT("error"), TEXT("no Recast navigation data created"));
            }
        }
    }
#else
    Result->SetStringField(TEXT("error"), TEXT("editor-only navigation preparation"));
#endif
    FString Text;
    TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Text);
    FJsonSerializer::Serialize(Result, Writer);
    return Text;
}
