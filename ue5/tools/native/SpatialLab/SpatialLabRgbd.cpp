#include "SpatialLabRgbd.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/SceneCapture2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/App.h"
#include "Misc/FileHelper.h"
#include "Misc/ScopeExit.h"
#include "RenderUtils.h"
#include "Serialization/JsonSerializer.h"
#include "TextureResource.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

namespace
{
    constexpr int32 Width=320,Height=240;
    bool SavePng(const void* Pixels,int64 Bytes,ERGBFormat Format,int32 Bits,const FString& Path)
    {
        IImageWrapperModule& Module=FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
        const TSharedPtr<IImageWrapper> Wrapper=Module.CreateImageWrapper(EImageFormat::PNG);
        return Wrapper.IsValid() && Wrapper->SetRaw(Pixels,Bytes,Width,Height,Format,Bits)
            && FFileHelper::SaveArrayToFile(Wrapper->GetCompressed(),*Path);
    }
    UTextureRenderTarget2D* Target(AActor* Owner,EPixelFormat Format)
    {
        UTextureRenderTarget2D* Texture=NewObject<UTextureRenderTarget2D>(Owner);
        Texture->ClearColor=FLinearColor::Black;
        Texture->bAutoGenerateMips=false;
        Texture->Filter=TF_Nearest;
        Texture->InitCustomFormat(Width,Height,Format,false);
        Texture->UpdateResourceImmediate(true);
        return Texture;
    }
    void Configure(USceneCaptureComponent2D* Capture,UTextureRenderTarget2D* Texture,ESceneCaptureSource Source,float Exposure)
    {
        Capture->TextureTarget=Texture;
        Capture->CaptureSource=Source;
        Capture->bCaptureEveryFrame=false;
        Capture->bCaptureOnMovement=false;
        Capture->bAlwaysPersistRenderingState=true;
        Capture->FOVAngle=90;
        Capture->ShowFlags.SetMotionBlur(false);
        Capture->ShowFlags.SetBloom(false);
        Capture->ShowFlags.SetTemporalAA(false);
        auto& PP=Capture->PostProcessSettings;
        PP.bOverride_MotionBlurAmount=true;PP.MotionBlurAmount=0;
        PP.bOverride_AutoExposureMinBrightness=true;PP.AutoExposureMinBrightness=Exposure;
        PP.bOverride_AutoExposureMaxBrightness=true;PP.AutoExposureMaxBrightness=Exposure;
        PP.bOverride_AutoExposureBias=true;PP.AutoExposureBias=0;
        PP.bOverride_BloomIntensity=true;PP.BloomIntensity=0;
    }
}

FString USpatialLabRgbd::CapturePose(UObject* WorldContext,FVector CameraPosition,FRotator CameraRotation,
                                    const FString& ColorPath,const FString& DepthPath,float FixedExposureEV100)
{
    TSharedRef<FJsonObject> Report=MakeShared<FJsonObject>();
    Report->SetStringField(TEXT("evidence_kind"),TEXT("synchronous_static_pose_rgbd_render"));
    Report->SetBoolField(TEXT("captured"),false);
    FString Error;
    UWorld* World=WorldContext?WorldContext->GetWorld():nullptr;
    const double Started=FPlatformTime::Seconds();
    const IConsoleVariable* Extended=IConsoleManager::Get().FindConsoleVariable(TEXT("r.DefaultFeature.AutoExposure.ExtendDefaultLuminanceRange"));
    if (!World || !FApp::CanEverRender() || CameraPosition.ContainsNaN() || CameraRotation.ContainsNaN()
        || !Extended || Extended->GetInt()!=1 || !FMath::IsFinite(FixedExposureEV100) || FixedExposureEV100 < -10 || FixedExposureEV100 > 20
        || ColorPath==DepthPath || IFileManager::Get().FileExists(*ColorPath) || IFileManager::Get().FileExists(*DepthPath))
        Error=TEXT("invalid pose/world/rendering or existing output");
    else
    {
#if WITH_EDITOR
        FlushAsyncLoading();
        FAssetCompilingManager::Get().FinishAllCompilation();
#endif
        ASceneCapture2D* Color=World->SpawnActor<ASceneCapture2D>();
        ASceneCapture2D* Depth=World->SpawnActor<ASceneCapture2D>();
        ON_SCOPE_EXIT { if (Color) Color->Destroy(); if (Depth) Depth->Destroy(); };
        if (!Color || !Depth) Error=TEXT("capture actor spawn failed");
        else
        {
            auto* ColorTexture=Target(Color,PF_B8G8R8A8);
            auto* DepthTexture=Target(Depth,PF_R32_FLOAT);
            Configure(Color->GetCaptureComponent2D(),ColorTexture,ESceneCaptureSource::SCS_FinalColorLDR,FixedExposureEV100);
            Configure(Depth->GetCaptureComponent2D(),DepthTexture,ESceneCaptureSource::SCS_SceneDepth,FixedExposureEV100);
            Color->SetActorLocationAndRotation(CameraPosition,CameraRotation);
            Depth->SetActorLocationAndRotation(CameraPosition,CameraRotation);
            // This function never moves physics or advances a source episode.
            // Both targets are rendered at the specified pose before readback.
            Color->GetCaptureComponent2D()->CaptureScene();
            Depth->GetCaptureComponent2D()->CaptureScene();
            FlushRenderingCommands();
            TArray<FColor> Colors;
            TArray<FLinearColor> DepthSamples;
            FReadSurfaceDataFlags Flags(RCM_MinMax,CubeFace_MAX);
            Flags.SetLinearToGamma(false);
            if (!ColorTexture->GameThread_GetRenderTargetResource()->ReadPixels(Colors,Flags)
                || !DepthTexture->GameThread_GetRenderTargetResource()->ReadLinearColorPixels(DepthSamples,Flags)
                || Colors.Num()!=Width*Height || DepthSamples.Num()!=Width*Height)
                Error=TEXT("incomplete RGB/depth readback");
            else
            {
                TArray<uint16> Millimeters;
                Millimeters.SetNumZeroed(DepthSamples.Num());
                int32 Valid=0;
                for (int32 Index=0;Index<DepthSamples.Num();++Index)
                {
                    Colors[Index].A=255;
                    const float Cm=DepthSamples[Index].R;
                    if (FMath::IsFinite(Cm) && Cm>0 && Cm<=3000)
                    {
                        Millimeters[Index]=static_cast<uint16>(FMath::RoundToInt32(Cm*10));
                        ++Valid;
                    }
                }
                const bool Okay=SavePng(Colors.GetData(),Colors.Num()*sizeof(FColor),ERGBFormat::BGRA,8,ColorPath)
                    && SavePng(Millimeters.GetData(),Millimeters.Num()*sizeof(uint16),ERGBFormat::Gray,16,DepthPath);
                if (!Okay) Error=TEXT("PNG write failed; partial outputs retained");
                else
                {
                    Report->SetBoolField(TEXT("captured"),true);
                    Report->SetNumberField(TEXT("fixed_exposure_ev100"),FixedExposureEV100);
                    const FVector Actual=Color->GetCaptureComponent2D()->GetComponentLocation();
                    const FRotator Rotation=Color->GetCaptureComponent2D()->GetComponentRotation();
                    Report->SetArrayField(TEXT("camera_position_cm"),{
                        MakeShared<FJsonValueNumber>(Actual.X),MakeShared<FJsonValueNumber>(Actual.Y),MakeShared<FJsonValueNumber>(Actual.Z)});
                    Report->SetArrayField(TEXT("camera_rotation_deg"),{
                        MakeShared<FJsonValueNumber>(Rotation.Pitch),MakeShared<FJsonValueNumber>(Rotation.Yaw),MakeShared<FJsonValueNumber>(Rotation.Roll)});
                    Report->SetNumberField(TEXT("valid_depth_pixels"),Valid);
                    Report->SetNumberField(TEXT("width"),Width);
                    Report->SetNumberField(TEXT("height"),Height);
                    Report->SetStringField(TEXT("depth_readback"),TEXT("SceneDepth R32 float cm; RCM_MinMax; rounded uint16 mm; >30m invalid0"));
                    Report->SetStringField(TEXT("color_readback"),TEXT("FinalColorLDR BGRA8 PNG; LinearToGamma false; no flips"));
                }
            }
        }
    }
    Report->SetStringField(TEXT("error"),Error);
    Report->SetNumberField(TEXT("render_and_readback_wall_s"),FPlatformTime::Seconds()-Started);
    FString Text;
    FJsonSerializer::Serialize(Report,TJsonWriterFactory<>::Create(&Text));
    return Text;
}
