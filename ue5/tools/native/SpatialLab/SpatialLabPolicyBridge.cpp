#include "SpatialLabPolicyBridge.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"
#if WITH_EDITOR
#include "IPythonScriptPlugin.h"
#endif

namespace
{
    FString Json(const TSharedPtr<FJsonObject>& Object)
    {
        FString Text;
        FJsonSerializer::Serialize(Object.ToSharedRef(),TJsonWriterFactory<>::Create(&Text));
        return Text;
    }
    FString Quoted(const FString& Text)
    {
        FString Result;
        auto Writer=TJsonWriterFactory<>::Create(&Result);
        Writer->WriteValue(Text);
        Writer->Close();
        return Result;
    }
    bool Execute(const FString& Command)
    {
#if WITH_EDITOR
        IPythonScriptPlugin* Python=FModuleManager::LoadModulePtr<IPythonScriptPlugin>(TEXT("PythonScriptPlugin"));
        if (!Python || (!Python->IsPythonInitialized() && !Python->ForceEnablePythonAtRuntime())) return false;
        if (!Python->IsPythonInitialized()) return false;
        FPythonCommandEx Request;
        Request.Flags=EPythonCommandFlags::Unattended;
        Request.FileExecutionScope=EPythonFileExecutionScope::Public;
        Request.Command=Command;
        return Python->ExecPythonCommandEx(Request);
#else
        return false;
#endif
    }
}

bool FSpatialLabPolicyBridge::Initialize(const FString& ToolsDirectory,const TSharedPtr<FJsonObject>& Mission,
                                        const FString& ResponsePath,const FString& Attempt)
{
    Path=ResponsePath;
    AttemptId=Attempt;
    if (IFileManager::Get().FileExists(*Path)) return false;
    const FString Command=TEXT("import sys, json\nsys.path.insert(0,")+Quoted(ToolsDirectory)
        +TEXT(")\nimport semantic_search_policy as _sitp_policy\n_sitp_search = _sitp_policy.SearchPolicy(json.loads(")
        +Quoted(Json(Mission))+TEXT("))");
    return Execute(Command);
}

bool FSpatialLabPolicyBridge::Step(const TSharedPtr<FJsonObject>& Frame,const TSharedPtr<FJsonObject>& Feedback,
                                  TSharedPtr<FJsonObject>& Action)
{
    if (IFileManager::Get().FileExists(*Path)) return false;
    const FString Command=TEXT("_sitp_policy.dispatch(_sitp_search,")+Quoted(Json(Frame))+TEXT(",")
        +Quoted(Json(Feedback))+TEXT(",")+Quoted(Path)+TEXT(",")+Quoted(AttemptId)+TEXT(")");
    FString Text;
    TSharedPtr<FJsonObject> Response;
    if (!Execute(Command) || !FFileHelper::LoadFileToString(Text,*Path)
        || !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text),Response) || !Response.IsValid()) return false;
    if (!IFileManager::Get().Delete(*Path,false,true)) return false;
    if (Response->GetStringField(TEXT("attempt_id"))!=AttemptId
        || Response->GetStringField(TEXT("run_id"))!=Frame->GetStringField(TEXT("run_id"))
        || Response->GetIntegerField(TEXT("frame_id"))!=Frame->GetIntegerField(TEXT("frame_id"))) return false;
    Action=Response->GetObjectField(TEXT("action"));
    return Action.IsValid();
}
