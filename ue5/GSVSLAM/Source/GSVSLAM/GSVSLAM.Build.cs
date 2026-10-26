// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class GSVSLAM : ModuleRules
{
	public GSVSLAM(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore"
		});

		// RenderCore: FRenderTarget / FReadSurfaceDataFlags.
		// RHI: FlushRenderingCommands.
		// ImageWrapper: PNG encoding.
		PrivateDependencyModuleNames.AddRange(new string[] {
			"RenderCore",
			"RHI",
			"ImageWrapper"
		});
	}
}
