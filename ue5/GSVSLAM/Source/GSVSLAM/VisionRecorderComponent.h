// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"

#include <atomic>

#include "VisionRecorderComponent.generated.h"

class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UCameraComponent;

/**
 * Records RGB frames plus the engine's own camera pose for every frame.
 *
 * Layout of one session directory:
 *   meta.json      parameters that must not change during a session
 *   poses.jsonl    one line per frame, engine values as-is (cm, UE axes)
 *   session.json   written on stop: counts, interval statistics, dropped samples
 *   frames/frame_000000.png ...
 *
 * Capture is deliberately one sample behind: asking for a render and reading the
 * target in the same tick returns the previous content — and on the first sample,
 * nothing at all. So each tick reads the frame requested on the previous tick and
 * requests the next one, always pairing an image with the pose it was rendered from.
 *
 * Coordinates are recorded untouched (UE left-handed, X forward, Y right, Z up,
 * centimetres). Any conversion to camera conventions belongs in the analysis
 * scripts, so a conversion bug can never be mistaken for a recording bug.
 */
UCLASS(ClassGroup = (Vision), meta = (BlueprintSpawnableComponent))
class GSVSLAM_API UVisionRecorderComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UVisionRecorderComponent();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/** Start a new session. Empty name means a timestamped default. */
	UFUNCTION(BlueprintCallable, Category = "Vision")
	bool StartRecording(const FString& InSessionName);

	UFUNCTION(BlueprintCallable, Category = "Vision")
	void StartRecordingDefault();

	UFUNCTION(BlueprintCallable, Category = "Vision")
	void StopRecording();

	UFUNCTION(BlueprintCallable, Category = "Vision")
	FString GetStatusString() const;

	/** Attach to the pawn's first camera component. Safe to call twice. */
	void AttachToCamera(UCameraComponent* InCamera);

	/** Render without recording or moving the route; not a scene-readiness claim. */
	void BeginWarmup(float RequestedSeconds);

	UPROPERTY(EditAnywhere, Category = "Vision")
	int32 CaptureWidth = 1280;

	UPROPERTY(EditAnywhere, Category = "Vision")
	int32 CaptureHeight = 720;

	UPROPERTY(EditAnywhere, Category = "Vision")
	float CaptureHz = 10.0f;

	/** Engine readback is bottom-up on D3D; kept configurable until verified on screen. */
	UPROPERTY(EditAnywhere, Category = "Vision")
	bool bFlipVertical = false;

	/** Empty means <Project>/../../output/ue5-datasets */
	UPROPERTY(EditAnywhere, Category = "Vision")
	FString OutputRootOverride;

	bool IsRecording() const { return bRecording; }

	/** Directory of the active (or last) session, used to co-locate derived files. */
	const FString& GetSessionDirectory() const { return SessionDirectory; }

	/** Also record a metric depth map next to each colour frame. */
	UPROPERTY(EditAnywhere, Category = "Vision")
	bool bCaptureDepth = true;

private:
	/** One sample: the pose is captured when the render is requested, not when read. */
	struct FPendingSample
	{
		int32 Index = 0;
		double WallSeconds = 0.0;
		double GameSeconds = 0.0;
		FVector Location = FVector::ZeroVector;
		FQuat Rotation = FQuat::Identity;
		float Fov = 90.0f;
		bool bValid = false;
	};

	void EnsureCaptureResources();
	void SetPawnVisualsVisible(bool bVisible);
	void RequestCapture();
	bool ReadRenderedFrame(TArray<FColor>& OutPixels);
	bool ReadDepthFrame(TArray<float>& OutDepthCm);
	void SaveFrame(const TArray<FColor>& Pixels, const FPendingSample& Sample);
	void SaveDepth(const TArray<float>& DepthCm, const FPendingSample& Sample);
	void AppendPoseLine(const FPendingSample& Sample);
	void FlushPoseLines();
	void WriteMeta();
	void CloseSession();

	static bool SavePngStatic(const TArray<FColor>& Pixels, int32 InWidth, int32 InHeight, const FString& FullPath);
	static bool SaveDepthPngStatic(const TArray<float>& DepthCm, int32 InWidth, int32 InHeight, const FString& FullPath);
	void EnqueueSave(const TArray<FColor>& Pixels, const FString& FullPath);
	void EnqueueDepthSave(const TArray<float>& DepthCm, const FString& FullPath);
	void DrainSaves();
	static FString Num(double Value);

	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> Capture;

	UPROPERTY(Transient)
	TObjectPtr<USceneCaptureComponent2D> DepthCapture;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> DepthTarget;

	UPROPERTY(Transient)
	TObjectPtr<UTextureRenderTarget2D> RenderTarget;

	UPROPERTY(Transient)
	TObjectPtr<UCameraComponent> TargetCamera;

	bool bRecording = false;
	bool bWarmingUp = false;
	float WarmupRequestedSeconds = 0.0f;
	double WarmupStartWall = 0.0;
	double WarmupStartGame = 0.0;
	double WarmupActualWall = 0.0;
	double WarmupActualGame = 0.0;
	int32 WarmupRenderTicks = 0;
	float InitialFov = 90.0f;
	bool bCalibrationConstant = true;
	FVector InitialCameraLocation = FVector::ZeroVector;
	FRotator InitialCameraRotation = FRotator::ZeroRotator;
	FString SessionStartUtc;
	double SampleAccumulator = 0.0;
	int32 FrameIndex = 0;
	int32 SampledFrames = 0;
	int32 DroppedSamples = 0;
	float MaxReadbackMs = 0.0f;
	double LastSampleWall = 0.0;
	double SessionStartWall = 0.0;
	TArray<float> SampleIntervalsMs;
	TArray<FString> PendingPoseLines;
	FString SessionDirectory;
	FString FramesDirectory;
	FString DepthDirectory;
	FString PosesPath;
	FString SessionLabel;
	FPendingSample Pending;

	// PNG encoding costs ~100 ms at 720p, far too much for the game thread at 10 Hz.
	// Frames are encoded on background tasks; the queue is capped so a slow disk
	// drops frames instead of growing memory without bound.
	struct FSaveStats
	{
		int32 Saved = 0;
		int32 Failed = 0;
		int32 DepthSaved = 0;
		int32 DepthFailed = 0;
		float MaxMs = 0.0f;
		double SumMs = 0.0;
	};

	mutable FCriticalSection SaveStatsLock;
	FSaveStats SaveStats;
	std::atomic<int32> PendingSaves{ 0 };
};
