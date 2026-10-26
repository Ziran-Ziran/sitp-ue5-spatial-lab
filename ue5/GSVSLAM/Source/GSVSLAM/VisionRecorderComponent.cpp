// Copyright Epic Games, Inc. All Rights Reserved.

#include "VisionRecorderComponent.h"

#include "Camera/CameraComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GSVSLAM.h"

#include "Async/Async.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "RenderUtils.h"
#include "TextureResource.h"

namespace
{
	constexpr int32 PoseFlushEvery = 50;
}

UVisionRecorderComponent::UVisionRecorderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	// The camera transform is final by PostUpdateWork; sampling here keeps the pose
	// matched to the frame that is about to be rendered from that pose.
	PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
	bAutoActivate = true;
	bTickInEditor = false;
}

void UVisionRecorderComponent::BeginPlay()
{
	Super::BeginPlay();

	if (!TargetCamera && GetOwner())
	{
		AttachToCamera(GetOwner()->FindComponentByClass<UCameraComponent>());
	}
}

void UVisionRecorderComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (bRecording)
	{
		CloseSession();
	}
	Super::EndPlay(EndPlayReason);
}

void UVisionRecorderComponent::AttachToCamera(UCameraComponent* InCamera)
{
	if (!InCamera)
	{
		UE_LOG(LogGSVSLAM, Warning, TEXT("VisionRecorder: no camera component found on owner."));
		return;
	}

	TargetCamera = InCamera;

	if (!Capture)
	{
		Capture = NewObject<USceneCaptureComponent2D>(GetOwner(), TEXT("VisionCapture"));
		Capture->SetupAttachment(TargetCamera);
		Capture->RegisterComponent();
		Capture->AttachToComponent(TargetCamera, FAttachmentTransformRules::SnapToTargetIncludingScale);
	}

	if (!DepthCapture)
	{
		DepthCapture = NewObject<USceneCaptureComponent2D>(GetOwner(), TEXT("VisionDepthCapture"));
		DepthCapture->SetupAttachment(TargetCamera);
		DepthCapture->RegisterComponent();
		DepthCapture->AttachToComponent(TargetCamera, FAttachmentTransformRules::SnapToTargetIncludingScale);
	}

	EnsureCaptureResources();
}

void UVisionRecorderComponent::EnsureCaptureResources()
{
	if (!TargetCamera || !Capture)
	{
		return;
	}

	if (!RenderTarget)
	{
		RenderTarget = NewObject<UTextureRenderTarget2D>(this);
		RenderTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
		RenderTarget->ClearColor = FLinearColor::Black;
		RenderTarget->bAutoGenerateMips = false;
		RenderTarget->Filter = TF_Nearest;
		RenderTarget->InitCustomFormat(CaptureWidth, CaptureHeight, PF_B8G8R8A8, false);
		RenderTarget->UpdateResourceImmediate(true);
	}

	Capture->TextureTarget = RenderTarget;
	Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	Capture->bCaptureEveryFrame = false;
	Capture->bCaptureOnMovement = false;
	Capture->bAlwaysPersistRenderingState = true;
	Capture->bUseCustomProjectionMatrix = false;
	Capture->FOVAngle = TargetCamera->FieldOfView;
	Capture->PostProcessBlendWeight = 1.0f;

	// Everything that makes consecutive frames differ for reasons other than motion
	// is forced off here; the project settings repeat the same overrides.
	FPostProcessSettings& PP = Capture->PostProcessSettings;
	PP.bOverride_MotionBlurAmount = true;
	PP.MotionBlurAmount = 0.0f;
	PP.bOverride_AutoExposureMinBrightness = true;
	PP.AutoExposureMinBrightness = 1.0f;
	PP.bOverride_AutoExposureMaxBrightness = true;
	PP.AutoExposureMaxBrightness = 1.0f;
	PP.bOverride_VignetteIntensity = true;
	PP.VignetteIntensity = 0.0f;
	PP.bOverride_BloomIntensity = true;
	PP.BloomIntensity = 0.0f;
	PP.bOverride_FilmGrainIntensity = true;
	PP.FilmGrainIntensity = 0.0f;
	PP.bOverride_SceneFringeIntensity = true;
	PP.SceneFringeIntensity = 0.0f;

	// A fixed -0.86 EV gain was measured too: it darkened the image (mean 179 -> 150)
	// but did not change the number of detectable corners, so the capture keeps the
	// scene exactly as rendered. Contrast was not what limited feature extraction.

	Capture->ShowFlags.SetMotionBlur(false);
	Capture->ShowFlags.SetBloom(false);

	// Diagnostic switch used while identifying scene geometry that must not reach
	// the dataset. -VisionHide=sky drops atmosphere and fog.
	FString HideMode;
	FParse::Value(FCommandLine::Get(), TEXT("VisionHide="), HideMode);
	if (HideMode.Contains(TEXT("sky")))
	{
		Capture->ShowFlags.SetAtmosphere(false);
		Capture->ShowFlags.SetFog(false);
	}

	if (bCaptureDepth && DepthCapture)
	{
		if (!DepthTarget)
		{
			DepthTarget = NewObject<UTextureRenderTarget2D>(this);
			DepthTarget->RenderTargetFormat = ETextureRenderTargetFormat::RTF_R32f;
			DepthTarget->ClearColor = FLinearColor::Black;
			DepthTarget->bAutoGenerateMips = false;
			DepthTarget->Filter = TF_Nearest;
			DepthTarget->InitCustomFormat(CaptureWidth, CaptureHeight, PF_R32_FLOAT, false);
			DepthTarget->UpdateResourceImmediate(true);
		}

		DepthCapture->TextureTarget = DepthTarget;
		DepthCapture->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
		DepthCapture->bCaptureEveryFrame = false;
		DepthCapture->bCaptureOnMovement = false;
		DepthCapture->bAlwaysPersistRenderingState = true;
		DepthCapture->FOVAngle = TargetCamera->FieldOfView;
		DepthCapture->PostProcessBlendWeight = 0.0f;
		DepthCapture->ShowFlags.SetMotionBlur(false);
		DepthCapture->ShowFlags.SetBloom(false);
	}
}

FString UVisionRecorderComponent::Num(double Value)
{
	// Keep the decimal separator invariant regardless of the host culture.
	FString Text = FString::Printf(TEXT("%.6f"), Value);
	Text.ReplaceInline(TEXT(","), TEXT("."));
	return Text;
}

void UVisionRecorderComponent::SetPawnVisualsVisible(bool bVisible)
{
	// The scene capture draws the player pawn even when the player cannot see it
	// (the body mesh is owner-no-see, the arms are only-owner-see). Either one puts
	// geometry that is rigidly attached to the camera into the dataset, which a
	// visual odometry system cannot explain. Hide every primitive on the pawn while
	// recording; the camera component itself is unaffected.
	if (!GetOwner())
	{
		return;
	}

	TArray<UPrimitiveComponent*> Primitives;
	GetOwner()->GetComponents<UPrimitiveComponent>(Primitives);
	for (UPrimitiveComponent* Primitive : Primitives)
	{
		if (Primitive)
		{
			// bPropagateToChildren must stay false. Propagating hides the camera and
			// the scene capture attached to it, and the capture then renders exactly
			// one frame and serves that same image forever.
			Primitive->SetVisibility(bVisible, false);
		}
	}
}

void UVisionRecorderComponent::BeginWarmup(float RequestedSeconds)
{
	if (bRecording || !TargetCamera)
	{
		return;
	}
	EnsureCaptureResources();
	SetPawnVisualsVisible(false);
	WarmupRequestedSeconds = RequestedSeconds;
	WarmupStartWall = FPlatformTime::Seconds();
	WarmupStartGame = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	WarmupRenderTicks = 0;
	bWarmingUp = RequestedSeconds > 0.0f;
	// Warm the actual capture views, not only the player's viewport. No readback,
	// PNGs, pose samples or route movement are produced during this stage.
	Capture->bCaptureEveryFrame = bWarmingUp;
	if (DepthCapture)
	{
		DepthCapture->bCaptureEveryFrame = bWarmingUp;
	}
}

bool UVisionRecorderComponent::StartRecording(const FString& InSessionName)
{
	if (bRecording)
	{
		UE_LOG(LogGSVSLAM, Warning, TEXT("VisionRecorder: already recording."));
		return false;
	}

	if (!TargetCamera && GetOwner())
	{
		AttachToCamera(GetOwner()->FindComponentByClass<UCameraComponent>());
	}
	if (!TargetCamera)
	{
		UE_LOG(LogGSVSLAM, Error, TEXT("VisionRecorder: cannot start without a camera."));
		return false;
	}
	if (!Capture || !RenderTarget)
	{
		EnsureCaptureResources();
	}
	WarmupActualWall = bWarmingUp ? FPlatformTime::Seconds() - WarmupStartWall : 0.0;
	WarmupActualGame = bWarmingUp && GetWorld() ? GetWorld()->GetTimeSeconds() - WarmupStartGame : 0.0;
	bWarmingUp = false;
	Capture->bCaptureEveryFrame = false;
	if (DepthCapture)
	{
		DepthCapture->bCaptureEveryFrame = false;
	}
	InitialFov = TargetCamera->FieldOfView;
	bCalibrationConstant = true;
	InitialCameraLocation = TargetCamera->GetComponentLocation();
	InitialCameraRotation = TargetCamera->GetComponentRotation();
	SessionStartUtc = FDateTime::UtcNow().ToIso8601();

	SessionLabel = InSessionName.IsEmpty()
		? FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S"))
		: InSessionName;

	const FString Root = OutputRootOverride.IsEmpty()
		? FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / TEXT("../../output/ue5-datasets"))
		: OutputRootOverride;

	// Never merge into an existing session: pick the first free numbered name.
	SessionDirectory = FPaths::Combine(Root, SessionLabel);
	for (int32 Suffix = 2; FPaths::DirectoryExists(SessionDirectory) && Suffix < 100; ++Suffix)
	{
		SessionDirectory = FPaths::Combine(Root, FString::Printf(TEXT("%s-%02d"), *SessionLabel, Suffix));
	}
	SessionLabel = FPaths::GetCleanFilename(SessionDirectory);
	FramesDirectory = FPaths::Combine(SessionDirectory, TEXT("frames"));
	PosesPath = FPaths::Combine(SessionDirectory, TEXT("poses.jsonl"));

	IFileManager::Get().MakeDirectory(*FramesDirectory, true);
	if (!FPaths::DirectoryExists(FramesDirectory))
	{
		UE_LOG(LogGSVSLAM, Error, TEXT("VisionRecorder: cannot create %s"), *FramesDirectory);
		return false;
	}

	DepthDirectory = FPaths::Combine(SessionDirectory, TEXT("depth"));
	if (bCaptureDepth)
	{
		IFileManager::Get().MakeDirectory(*DepthDirectory, true);
	}

	FrameIndex = 0;
	SampledFrames = 0;
	DroppedSamples = 0;
	MaxReadbackMs = 0.0f;
	SampleAccumulator = 0.0;
	LastSampleWall = 0.0;
	SampleIntervalsMs.Reset();
	PendingPoseLines.Reset();
	Pending = FPendingSample();
	SessionStartWall = FPlatformTime::Seconds();
	{
		FScopeLock Lock(&SaveStatsLock);
		SaveStats = FSaveStats();
	}
	PendingSaves = 0;

	// Load the module on the game thread; worker tasks only use it afterwards.
	FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));

	SetPawnVisualsVisible(false);

	WriteMeta();
	RequestCapture();

	bRecording = true;
	UE_LOG(LogGSVSLAM, Log, TEXT("VisionRecorder: recording into %s"), *SessionDirectory);
	return true;
}

void UVisionRecorderComponent::StartRecordingDefault()
{
	StartRecording(FString());
}

void UVisionRecorderComponent::StopRecording()
{
	if (!bRecording)
	{
		return;
	}
	CloseSession();
}

void UVisionRecorderComponent::RequestCapture()
{
	if (!Capture || !TargetCamera)
	{
		return;
	}
	if (bRecording && !FMath::IsNearlyEqual(TargetCamera->FieldOfView, InitialFov, 0.0001f))
	{
		bCalibrationConstant = false;
	}

	// The capture is attached to the camera, but the transform is pushed explicitly:
	// an attachment that silently fails leaves a frozen viewpoint whose images still
	// look plausible while every frame turns out byte-identical.
	Capture->SetWorldLocationAndRotation(
		TargetCamera->GetComponentLocation(),
		TargetCamera->GetComponentRotation());
	Capture->FOVAngle = TargetCamera->FieldOfView;
	Capture->CaptureScene();

	if (bCaptureDepth && DepthCapture)
	{
		DepthCapture->SetWorldLocationAndRotation(
			TargetCamera->GetComponentLocation(),
			TargetCamera->GetComponentRotation());
		DepthCapture->FOVAngle = TargetCamera->FieldOfView;
		DepthCapture->CaptureScene();
	}

	Pending.Index = FrameIndex;
	Pending.WallSeconds = FPlatformTime::Seconds();
	Pending.GameSeconds = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	Pending.Location = TargetCamera->GetComponentLocation();
	Pending.Rotation = TargetCamera->GetComponentQuat();
	Pending.Fov = TargetCamera->FieldOfView;
	Pending.bValid = true;

	if (LastSampleWall > 0.0)
	{
		SampleIntervalsMs.Add(static_cast<float>((Pending.WallSeconds - LastSampleWall) * 1000.0));
	}
	LastSampleWall = Pending.WallSeconds;
}

void UVisionRecorderComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (bWarmingUp && TargetCamera)
	{
		++WarmupRenderTicks;
		Capture->SetWorldLocationAndRotation(TargetCamera->GetComponentLocation(), TargetCamera->GetComponentRotation());
		if (DepthCapture)
		{
			DepthCapture->SetWorldLocationAndRotation(TargetCamera->GetComponentLocation(), TargetCamera->GetComponentRotation());
		}
	}

	if (!bRecording)
	{
		return;
	}

	if (!Capture || !RenderTarget)
	{
		EnsureCaptureResources();
		if (!Capture || !RenderTarget)
		{
			return;
		}
	}

	const float Interval = 1.0f / FMath::Max(CaptureHz, 1.0f);
	SampleAccumulator += DeltaTime;

	if (SampleAccumulator < Interval)
	{
		return;
	}

	const int32 Due = FMath::FloorToInt(SampleAccumulator / Interval);
	SampleAccumulator -= Due * Interval;
	if (Due > 1)
	{
		// A hitch cost us frames; count them instead of inventing samples.
		DroppedSamples += Due - 1;
	}

	// 1) Finish the sample requested on the previous tick.
	if (Pending.bValid)
	{
		TArray<FColor> Pixels;
		if (ReadRenderedFrame(Pixels))
		{
			SaveFrame(Pixels, Pending);
			++SampledFrames;

			if (bCaptureDepth && DepthTarget)
			{
				TArray<float> DepthCm;
				if (ReadDepthFrame(DepthCm))
				{
					SaveDepth(DepthCm, Pending);
				}
				else
				{
					FScopeLock Lock(&SaveStatsLock);
					++SaveStats.DepthFailed;
				}
			}
		}
		else
		{
			FScopeLock Lock(&SaveStatsLock);
			++SaveStats.Failed;
		}
		Pending.bValid = false;
	}

	// 2) Request the next one; it is read on the following tick.
	++FrameIndex;
	RequestCapture();
}

bool UVisionRecorderComponent::ReadRenderedFrame(TArray<FColor>& OutPixels)
{
	FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return false;
	}

	FReadSurfaceDataFlags Flags(RCM_UNorm, CubeFace_MAX);
	Flags.SetLinearToGamma(false);

	const double ReadStart = FPlatformTime::Seconds();
	const bool bOk = Resource->ReadPixels(OutPixels, Flags);
	const float ReadMs = static_cast<float>((FPlatformTime::Seconds() - ReadStart) * 1000.0);
	MaxReadbackMs = FMath::Max(MaxReadbackMs, ReadMs);

	return bOk && OutPixels.Num() == CaptureWidth * CaptureHeight;
}

void UVisionRecorderComponent::SaveFrame(const TArray<FColor>& Pixels, const FPendingSample& Sample)
{
	TArray<FColor> Buffer = Pixels;

	if ((Sample.Index % 10) == 0)
	{
		uint32 Hash = 2166136261u;
		for (int32 i = 0; i < Buffer.Num(); i += 997)
		{
			Hash = (Hash ^ (uint32(Buffer[i].R) | (uint32(Buffer[i].G) << 8) | (uint32(Buffer[i].B) << 16))) * 16777619u;
		}
		const FVector CapLoc = Capture->GetComponentLocation();
		UE_LOG(LogGSVSLAM, Log,
			TEXT("VisionDebug idx=%d hash=%u cam=(%.1f,%.1f,%.1f) cap=(%.1f,%.1f,%.1f)"),
			Sample.Index, Hash,
			Sample.Location.X, Sample.Location.Y, Sample.Location.Z,
			CapLoc.X, CapLoc.Y, CapLoc.Z);
	}

	if (bFlipVertical)
	{
		for (int32 Row = 0; Row < CaptureHeight / 2; ++Row)
		{
			FColor* Top = Buffer.GetData() + Row * CaptureWidth;
			FColor* Bottom = Buffer.GetData() + (CaptureHeight - 1 - Row) * CaptureWidth;
			for (int32 Col = 0; Col < CaptureWidth; ++Col)
			{
				Swap(Top[Col], Bottom[Col]);
			}
		}
	}

	// The pose line is written immediately so ordering never depends on how long the
	// background encoder takes; the PNG lands out of band under a fixed name.
	AppendPoseLine(Sample);
	const FString FilePath = FString::Printf(TEXT("%s/frame_%06d.png"), *FramesDirectory, Sample.Index);
	EnqueueSave(Buffer, FilePath);

	if ((SampledFrames % PoseFlushEvery) == 0)
	{
		FlushPoseLines();
	}
}

bool UVisionRecorderComponent::SavePngStatic(const TArray<FColor>& Pixels, int32 InWidth, int32 InHeight, const FString& FullPath)
{
	// Module is already loaded by StartRecording; background tasks must not load it.
	IImageWrapperModule& Module = FModuleManager::GetModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
	TSharedPtr<IImageWrapper> Wrapper = Module.CreateImageWrapper(EImageFormat::PNG);
	if (!Wrapper.IsValid())
	{
		return false;
	}

	if (!Wrapper->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor),
		InWidth, InHeight, ERGBFormat::BGRA, 8))
	{
		return false;
	}

	const TArray64<uint8>& Compressed = Wrapper->GetCompressed();
	return FFileHelper::SaveArrayToFile(Compressed, *FullPath);
}

bool UVisionRecorderComponent::ReadDepthFrame(TArray<float>& OutDepthCm)
{
	FTextureRenderTargetResource* Resource = DepthTarget->GameThread_GetRenderTargetResource();
	if (!Resource)
	{
		return false;
	}

	TArray<FLinearColor> Samples;
	FReadSurfaceDataFlags Flags(RCM_UNorm, CubeFace_MAX);
	if (!Resource->ReadLinearColorPixels(Samples, Flags))
	{
		return false;
	}
	if (Samples.Num() != CaptureWidth * CaptureHeight)
	{
		return false;
	}

	// SCS_SceneDepth writes metric depth (centimetres) into the red channel.
	OutDepthCm.SetNumUninitialized(Samples.Num());
	for (int32 Index = 0; Index < Samples.Num(); ++Index)
	{
		OutDepthCm[Index] = Samples[Index].R;
	}
	return true;
}

void UVisionRecorderComponent::SaveDepth(const TArray<float>& DepthCm, const FPendingSample& Sample)
{
	const FString FilePath = FString::Printf(TEXT("%s/frame_%06d.png"), *DepthDirectory, Sample.Index);
	EnqueueDepthSave(DepthCm, FilePath);
}

bool UVisionRecorderComponent::SaveDepthPngStatic(const TArray<float>& DepthCm, int32 InWidth, int32 InHeight, const FString& FullPath)
{
	// 16-bit PNG in millimetres: 65.5 m range at 1 mm steps. Depth in centimetres
	// would quantise to 1 cm, which is coarser than the reconstruction error we care
	// about; float32 raw would be 1.8 MB per frame.
	TArray<uint16> Millimetres;
	Millimetres.SetNumUninitialized(DepthCm.Num());
	for (int32 Index = 0; Index < DepthCm.Num(); ++Index)
	{
		Millimetres[Index] = static_cast<uint16>(FMath::Clamp(DepthCm[Index] * 10.0f, 0.0f, 65535.0f));
	}

	IImageWrapperModule& Module = FModuleManager::GetModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
	TSharedPtr<IImageWrapper> Wrapper = Module.CreateImageWrapper(EImageFormat::PNG);
	if (!Wrapper.IsValid())
	{
		return false;
	}

	if (!Wrapper->SetRaw(reinterpret_cast<const uint8*>(Millimetres.GetData()),
		Millimetres.Num() * sizeof(uint16), InWidth, InHeight, ERGBFormat::Gray, 16))
	{
		return false;
	}

	const TArray64<uint8>& Compressed = Wrapper->GetCompressed();
	return FFileHelper::SaveArrayToFile(Compressed, *FullPath);
}

void UVisionRecorderComponent::EnqueueDepthSave(const TArray<float>& DepthCm, const FString& FullPath)
{
	constexpr int32 MaxPendingSaves = 24;
	if (PendingSaves.load() >= MaxPendingSaves)
	{
		++DroppedSamples;
		return;
	}

	TSharedPtr<TArray<float>> Buffer = MakeShared<TArray<float>>(DepthCm);
	const int32 Width = CaptureWidth;
	const int32 Height = CaptureHeight;
	++PendingSaves;

	Async(EAsyncExecution::ThreadPool, [this, Buffer, Width, Height, FullPath]()
	{
		const bool bOk = SaveDepthPngStatic(*Buffer, Width, Height, FullPath);
		FScopeLock Lock(&SaveStatsLock);
		if (bOk)
		{
			++SaveStats.DepthSaved;
		}
		else
		{
			++SaveStats.DepthFailed;
		}
		--PendingSaves;
	});
}

void UVisionRecorderComponent::EnqueueSave(const TArray<FColor>& Pixels, const FString& FullPath)
{
	// At ten frames per second the encoder keeps up from background tasks, but a
	// stalled disk must not grow memory without bound.
	constexpr int32 MaxPendingSaves = 24;
	if (PendingSaves.load() >= MaxPendingSaves)
	{
		++DroppedSamples;
		return;
	}

	TSharedPtr<TArray<FColor>> Buffer = MakeShared<TArray<FColor>>(Pixels);
	const int32 Width = CaptureWidth;
	const int32 Height = CaptureHeight;
	++PendingSaves;

	Async(EAsyncExecution::ThreadPool, [this, Buffer, Width, Height, FullPath]()
	{
		const double Start = FPlatformTime::Seconds();
		const bool bOk = SavePngStatic(*Buffer, Width, Height, FullPath);
		const float Elapsed = static_cast<float>((FPlatformTime::Seconds() - Start) * 1000.0);

		{
			FScopeLock Lock(&SaveStatsLock);
			if (bOk)
			{
				++SaveStats.Saved;
				SaveStats.SumMs += Elapsed;
				SaveStats.MaxMs = FMath::Max(SaveStats.MaxMs, Elapsed);
			}
			else
			{
				++SaveStats.Failed;
			}
		}
		--PendingSaves;
	});
}

void UVisionRecorderComponent::DrainSaves()
{
	// Stop must not report a frame count before the last PNG actually exists.
	const double Deadline = FPlatformTime::Seconds() + 10.0;
	while (PendingSaves.load() > 0 && FPlatformTime::Seconds() < Deadline)
	{
		FPlatformProcess::Sleep(0.01f);
	}
}

void UVisionRecorderComponent::AppendPoseLine(const FPendingSample& Sample)
{
	const double Focal = (CaptureWidth * 0.5) / FMath::Tan(FMath::DegreesToRadians(Sample.Fov) * 0.5);

	const FString Line = FString::Printf(
		TEXT("{\"i\": %d, \"t_wall\": %s, \"t_game\": %s, \"loc\": [%s, %s, %s], "
			 "\"quat\": [%s, %s, %s, %s], \"fov\": %s, \"fx\": %s, \"fy\": %s, \"cx\": %s, \"cy\": %s, "
			 "\"w\": %d, \"h\": %d}"),
		Sample.Index,
		*Num(Sample.WallSeconds), *Num(Sample.GameSeconds),
		*Num(Sample.Location.X), *Num(Sample.Location.Y), *Num(Sample.Location.Z),
		*Num(Sample.Rotation.X), *Num(Sample.Rotation.Y), *Num(Sample.Rotation.Z), *Num(Sample.Rotation.W),
		*Num(Sample.Fov), *Num(Focal), *Num(Focal),
		*Num(CaptureWidth * 0.5), *Num(CaptureHeight * 0.5),
		CaptureWidth, CaptureHeight);

	PendingPoseLines.Add(Line);
}

void UVisionRecorderComponent::FlushPoseLines()
{
	if (PendingPoseLines.Num() == 0)
	{
		return;
	}

	FString Block;
	for (const FString& Line : PendingPoseLines)
	{
		Block += Line;
		Block += TEXT("\n");
	}
	PendingPoseLines.Reset();

	FFileHelper::SaveStringToFile(Block, *PosesPath,
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
		&IFileManager::Get(), EFileWrite::FILEWRITE_Append);
}

void UVisionRecorderComponent::WriteMeta()
{
	const UWorld* World = GetWorld();
	FString MapName = World ? World->GetMapName() : TEXT("unknown");
	MapName.RemoveFromStart(World ? World->StreamingLevelsPrefix : FString());

	const FVector CamLocation = InitialCameraLocation;
	const FRotator CamRotation = InitialCameraRotation;
	const double Focal = (CaptureWidth * 0.5) / FMath::Tan(FMath::DegreesToRadians(InitialFov) * 0.5);

	const FString Meta = FString::Printf(
		TEXT("{\n")
		TEXT("  \"session\": \"%s\",\n")
		TEXT("  \"engine\": \"%s\",\n")
		TEXT("  \"project\": \"GSVSLAM\",\n")
		TEXT("  \"map\": \"%s\",\n")
		TEXT("  \"start_utc\": \"%s\",\n")
		TEXT("  \"coordinate_system\": \"UE left-handed, X forward, Y right, Z up, centimetres\",\n")
		TEXT("  \"width\": %d,\n")
		TEXT("  \"height\": %d,\n")
		TEXT("  \"target_hz\": %s,\n")
		TEXT("  \"fov_degrees_initial\": %s,\n")
		TEXT("  \"calibration\": {\"fx\": %s, \"fy\": %s, \"cx\": %s, \"cy\": %s, \"w\": %d, \"h\": %d},\n")
		TEXT("  \"calibration_constant\": %s,\n")
		TEXT("  \"warmup\": {\"policy\": \"fixed delay; not a readiness detector\", \"requested_s\": %s, \"actual_wall_s\": %s, \"actual_game_s\": %s, \"render_ticks\": %d, \"start_wall_s\": %s, \"start_game_s\": %s},\n")
		TEXT("  \"flip_vertical\": %s,\n")
		TEXT("  \"capture_delay_samples\": 1,\n")
		TEXT("  \"depth\": {\"enabled\": %s, \"format\": \"png16_gray\", \"unit\": \"millimetres\",\n")
		TEXT("            \"meaning\": \"SCS_SceneDepth red channel, metric depth along the camera axis\"},\n")
		TEXT("  \"camera\": \"first-person, capture component exactly on player camera\",\n")
		TEXT("  \"start_pose\": {\"loc\": [%s, %s, %s], \"rot_pitch_yaw_roll\": [%s, %s, %s]}\n")
		TEXT("}\n"),
		*SessionLabel,
		*FEngineVersion::Current().ToString(),
		*MapName,
		*SessionStartUtc,
		CaptureWidth, CaptureHeight, *Num(CaptureHz),
		*Num(InitialFov),
		*Num(Focal), *Num(Focal), *Num(CaptureWidth * 0.5), *Num(CaptureHeight * 0.5), CaptureWidth, CaptureHeight,
		bCalibrationConstant ? TEXT("true") : TEXT("false"),
		*Num(WarmupRequestedSeconds), *Num(WarmupActualWall), *Num(WarmupActualGame), WarmupRenderTicks,
		*Num(WarmupStartWall), *Num(WarmupStartGame),
		bFlipVertical ? TEXT("true") : TEXT("false"),
		bCaptureDepth ? TEXT("true") : TEXT("false"),
		*Num(CamLocation.X), *Num(CamLocation.Y), *Num(CamLocation.Z),
		*Num(CamRotation.Pitch), *Num(CamRotation.Yaw), *Num(CamRotation.Roll));

	FFileHelper::SaveStringToFile(Meta, *FPaths::Combine(SessionDirectory, TEXT("meta.json")),
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}

void UVisionRecorderComponent::CloseSession()
{
	// Flush the frame that was requested last, so the session does not end one short.
	if (Pending.bValid)
	{
		TArray<FColor> Pixels;
		if (ReadRenderedFrame(Pixels))
		{
			SaveFrame(Pixels, Pending);
			++SampledFrames;

			if (bCaptureDepth && DepthTarget)
			{
				TArray<float> DepthCm;
				if (ReadDepthFrame(DepthCm))
				{
					SaveDepth(DepthCm, Pending);
				}
			}
		}
		Pending.bValid = false;
	}

	bRecording = false;
	FlushPoseLines();
	DrainSaves();
	// Preserve the initial pose/time while finalizing the independent calibration
	// validity flag, so changing FOV cannot silently become a fixed-FOV dataset.
	WriteMeta();

	TArray<float> Sorted = SampleIntervalsMs;
	Sorted.Sort();
	const float MinMs = Sorted.Num() > 0 ? Sorted[0] : 0.0f;
	const float MaxMs = Sorted.Num() > 0 ? Sorted.Last() : 0.0f;
	const float MedianMs = Sorted.Num() > 0 ? Sorted[Sorted.Num() / 2] : 0.0f;

	FSaveStats Final;
	{
		FScopeLock Lock(&SaveStatsLock);
		Final = SaveStats;
	}

	const float SaveMeanMs = Final.Saved > 0 ? static_cast<float>(Final.SumMs / Final.Saved) : 0.0f;
	const double DurationS = FPlatformTime::Seconds() - SessionStartWall;

	const FString Report = FString::Printf(
		TEXT("{\n")
		TEXT("  \"session\": \"%s\",\n")
		TEXT("  \"frames_sampled\": %d,\n")
		TEXT("  \"frames_saved\": %d,\n")
		TEXT("  \"seconds\": %s,\n")
		TEXT("  \"target_hz\": %s,\n")
		TEXT("  \"sample_interval_ms\": {\"min\": %s, \"median\": %s, \"max\": %s},\n")
		TEXT("  \"png_save_ms\": {\"mean\": %s, \"max\": %s},\n")
		TEXT("  \"dropped_samples\": %d,\n")
		TEXT("  \"save_failures\": %d,\n")
		TEXT("  \"depth_frames_saved\": %d,\n")
		TEXT("  \"depth_failures\": %d,\n")
		TEXT("  \"max_readback_ms\": %s\n")
		TEXT("}\n"),
		*SessionLabel, SampledFrames, Final.Saved, *Num(DurationS), *Num(CaptureHz),
		*Num(MinMs), *Num(MedianMs), *Num(MaxMs),
		*Num(SaveMeanMs), *Num(Final.MaxMs),
		DroppedSamples, Final.Failed, Final.DepthSaved, Final.DepthFailed, *Num(MaxReadbackMs));

	FFileHelper::SaveStringToFile(Report, *FPaths::Combine(SessionDirectory, TEXT("session.json")),
		FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);

	UE_LOG(LogGSVSLAM, Log, TEXT("VisionRecorder: stopped. sampled=%d saved=%d dropped=%d failed=%d"),
		SampledFrames, Final.Saved, DroppedSamples, Final.Failed);

	SetPawnVisualsVisible(true);
}

FString UVisionRecorderComponent::GetStatusString() const
{
	if (!bRecording)
	{
		return TEXT("idle");
	}
	int32 Saved = 0;
	int32 Failed = 0;
	{
		FScopeLock Lock(&SaveStatsLock);
		Saved = SaveStats.Saved;
		Failed = SaveStats.Failed;
	}
	return FString::Printf(TEXT("recording %s: %d sampled, %d saved, %d pending, %d dropped, %d failed"),
		*SessionDirectory, SampledFrames, Saved, PendingSaves.load(), DroppedSamples, Failed);
}
