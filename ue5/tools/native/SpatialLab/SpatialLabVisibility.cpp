#include "SpatialLabVisibility.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/FileManager.h"
#include "Policies/CondensedJsonPrintPolicy.h"
#include "Serialization/JsonSerializer.h"

namespace
{
    TSharedPtr<FJsonValue> VectorValue(FVector P)
    {
        return MakeShared<FJsonValueArray>(TArray<TSharedPtr<FJsonValue>>{
            MakeShared<FJsonValueNumber>(P.X),MakeShared<FJsonValueNumber>(P.Y),MakeShared<FJsonValueNumber>(P.Z)});
    }
}

void FSpatialLabVisibilityRecorder::Write(const TSharedRef<FJsonObject>& Record)
{
    if (!Healthy || !Archive) { Healthy = false; return; }
    FString Text;
    auto Writer = TJsonWriterFactory<TCHAR,TCondensedJsonPrintPolicy<TCHAR>>::Create(&Text);
    FJsonSerializer::Serialize(Record,Writer);
    Text += TEXT("\n");
    FTCHARToUTF8 Encoded(*Text);
    Archive->Serialize(const_cast<ANSICHAR*>(Encoded.Get()),Encoded.Length());
    Healthy = !Archive->IsError();
}

bool FSpatialLabVisibilityRecorder::Open(UWorld* InWorld, const FString& Path, const FString& ManifestHash, const FString& AttemptId)
{
#if WITH_EDITOR
    if (!InWorld || !InWorld->IsGameWorld() || IFileManager::Get().FileExists(*Path)) return false;
    World = InWorld;
    Archive.Reset(IFileManager::Get().CreateFileWriter(*Path,FILEWRITE_NoReplaceExisting));
    if (!Archive) return false;
    StartedWall = FPlatformTime::Seconds();
    // The backend may inspect engine labels; the exported stream contains only
    // observed entities. The policy never receives this internal target cache.
    for (TActorIterator<AActor> It(InWorld); It; ++It)
    {
        const FString Label = It->GetActorLabel();
        FString Category;
        if (Label.StartsWith(TEXT("Area_MdProps_Chair"))) Category = TEXT("chair");
        else if (Label.StartsWith(TEXT("Area_MdProps_Lamp13_"))) Category = TEXT("lamp");
        if (!Category.IsEmpty())
        {
            Targets.Add(*It);
            Categories.Add(*It,Category);
        }
    }
    Targets.Sort([](const TWeakObjectPtr<AActor>& A,const TWeakObjectPtr<AActor>& B) {
        return A->GetName() < B->GetName();
    });
    TSharedRef<FJsonObject> Header = MakeShared<FJsonObject>();
    Header->SetStringField(TEXT("record_type"),TEXT("stream"));
    Header->SetNumberField(TEXT("schema_version"),1);
    Header->SetStringField(TEXT("condition"),TEXT("engine_local_visibility_supervision"));
    Header->SetStringField(TEXT("semantic_source"),TEXT("engine_actor_label"));
    Header->SetStringField(TEXT("pose_source"),TEXT("engine_ground_truth"));
    Header->SetStringField(TEXT("route_source"),TEXT("global_engine_oracle_selected_routes"));
    Header->SetStringField(TEXT("visibility_method"),TEXT("panoramic_range_three_bbox_samples_first_hit_target"));
    Header->SetStringField(TEXT("units"),TEXT("centimeters"));
    Header->SetStringField(TEXT("coordinate_system"),TEXT("UE left-handed X-forward Y-right Z-up"));
    Header->SetStringField(TEXT("map"),InWorld->GetOutermost()->GetName());
    Header->SetStringField(TEXT("manifest_sha256"),ManifestHash);
    Header->SetStringField(TEXT("attempt_id"),AttemptId);
    Header->SetNumberField(TEXT("range_cm"),2000);
    Header->SetNumberField(TEXT("sample_period_sim_s"),0.1);
    Header->SetNumberField(TEXT("eye_height_cm"),150);
    Write(Header);
    return Healthy;
#else
    return false; // actor-label supervision requires the editor build, even in Game world
#endif
}

void FSpatialLabVisibilityRecorder::BeginEpisode(const FString& Id,double SimStart)
{
    RunId = Id;
    EpisodeStart = SimStart;
    LastSim = -1;
    FrameIndex = 0;
    ++Episodes;
    TSharedRef<FJsonObject> Record = MakeShared<FJsonObject>();
    Record->SetStringField(TEXT("record_type"),TEXT("episode_start"));
    Record->SetStringField(TEXT("run_id"),RunId);
    Write(Record);
}

void FSpatialLabVisibilityRecorder::Sample(AActor* Observer,FVector Feet,bool Force)
{
    UWorld* CurrentWorld = World.Get();
    if (!CurrentWorld || !Observer || Closed) return;
    const double Sim = CurrentWorld->GetTimeSeconds() - EpisodeStart;
    if (Sim <= LastSim || (!Force && LastSim >= 0 && Sim - LastSim < 0.1 - 1e-6)) return;
    if (Feet.ContainsNaN()) { Healthy = false; return; }
    const FVector Eye = Feet + FVector(0,0,150);
    FCollisionQueryParams Query(SCENE_QUERY_STAT(SpatialLabVisibility),true);
    Query.AddIgnoredActor(Observer);
    TArray<TSharedPtr<FJsonValue>> Entities;
    for (const TWeakObjectPtr<AActor>& WeakTarget : Targets)
    {
        AActor* Target = WeakTarget.Get();
        if (!Target || FVector::Distance(Eye,Target->GetActorLocation()) > 2000) continue;
        FVector Center,Extent;
        Target->GetActorBounds(false,Center,Extent);
        int32 VisibleSamples = 0;
        for (const double Offset : {0.0,0.5,-0.5})
        {
            FHitResult Hit;
            const FVector SamplePoint = Center + FVector(0,0,Offset * Extent.Z);
            // A free ray ending in an empty bounding-box gap is NOT a detection.
            // Require the first blocking geometry hit to be the target itself.
            if (CurrentWorld->LineTraceSingleByChannel(Hit,Eye,SamplePoint,ECC_Visibility,Query)
                && Hit.GetActor() == Target) ++VisibleSamples;
        }
        if (VisibleSamples == 0) continue;
        TSharedRef<FJsonObject> Entity = MakeShared<FJsonObject>();
        Entity->SetStringField(TEXT("entity_id"),Target->GetName());
        Entity->SetStringField(TEXT("category"),Categories[WeakTarget]);
        Entity->SetField(TEXT("position_cm"),VectorValue(Target->GetActorLocation()));
        Entity->SetNumberField(TEXT("visible_sample_count"),VisibleSamples);
        Entities.Add(MakeShared<FJsonValueObject>(Entity));
    }
    TSharedRef<FJsonObject> Record = MakeShared<FJsonObject>();
    Record->SetStringField(TEXT("record_type"),TEXT("frame"));
    Record->SetStringField(TEXT("run_id"),RunId);
    Record->SetNumberField(TEXT("frame_id"),FrameIndex++);
    Record->SetNumberField(TEXT("t_sim_s"),Sim);
    Record->SetNumberField(TEXT("t_monotonic_s"),FPlatformTime::Seconds() - StartedWall);
    Record->SetField(TEXT("observer_position_cm"),VectorValue(Feet));
    Record->SetField(TEXT("sensor_eye_cm"),VectorValue(Eye));
    Record->SetArrayField(TEXT("entities"),Entities);
    Write(Record);
    LastSim = Sim;
    ++TotalFrames;
}

void FSpatialLabVisibilityRecorder::EndEpisode(const FString& Status)
{
    TSharedRef<FJsonObject> Record = MakeShared<FJsonObject>();
    Record->SetStringField(TEXT("record_type"),TEXT("episode_end"));
    Record->SetStringField(TEXT("run_id"),RunId);
    Record->SetStringField(TEXT("status"),Status);
    Record->SetNumberField(TEXT("frames"),FrameIndex);
    Write(Record);
    if (Archive) Archive->Flush();
}

bool FSpatialLabVisibilityRecorder::Finish()
{
    TSharedRef<FJsonObject> Footer = MakeShared<FJsonObject>();
    Footer->SetStringField(TEXT("record_type"),TEXT("stream_end"));
    Footer->SetBoolField(TEXT("finished"),true);
    Footer->SetNumberField(TEXT("episodes"),Episodes);
    Footer->SetNumberField(TEXT("frames"),TotalFrames);
    Write(Footer);
    if (Archive) { Archive->Flush(); Healthy &= !Archive->IsError(); }
    Archive.Reset();
    Closed = true;
    return Healthy;
}
