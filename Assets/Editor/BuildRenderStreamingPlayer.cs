using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

public static class BuildRenderStreamingPlayer
{
    private const string OutputDirectory = "Build/RenderStreaming";
    private const string OutputFileName = "RLStream.x86_64";
    private static readonly string[] Scenes =
    {
        "Assets/Scenes/KJH/RL.unity"
    };

    public static string OutputPath => Path.Combine(OutputDirectory, OutputFileName);

    [MenuItem("Build/Build Render Streaming Player")]
    public static void BuildFromMenu()
    {
        BuildLinuxPlayer();
    }

    public static void BuildLinuxPlayer()
    {
        Directory.CreateDirectory(OutputDirectory);

        var options = new BuildPlayerOptions
        {
            scenes = Scenes,
            locationPathName = OutputPath,
            target = BuildTarget.StandaloneLinux64,
            options = BuildOptions.None,
        };

        var report = BuildPipeline.BuildPlayer(options);
        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new BuildFailedException(
                $"Render Streaming player build failed: {report.summary.result}");
        }

        Debug.Log($"Render Streaming player built at {Path.GetFullPath(OutputPath)}");
    }
}
