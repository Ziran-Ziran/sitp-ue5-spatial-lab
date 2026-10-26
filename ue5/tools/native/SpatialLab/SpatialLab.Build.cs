using UnrealBuildTool;

public class SpatialLab : ModuleRules
{
    public SpatialLab(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "NavigationSystem", "AIModule"
        });
        PrivateDependencyModuleNames.AddRange(new string[] { "Json", "JsonUtilities", "RenderCore", "RHI", "ImageWrapper" });
        if (Target.bBuildEditor)
            PrivateDependencyModuleNames.AddRange(new string[] { "UnrealEd", "PythonScriptPlugin" });
    }
}
