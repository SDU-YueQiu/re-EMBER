# 对 re-EMBER 命令行运行进行性能采样，并把耗时与 ETW 产物写入 build\perf。
param(
    [string]$Lhs,
    [string]$Rhs,
    [ValidateSet("union", "intersection", "difference")]
    [string]$Op = "difference",
    [string]$Out,
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Configuration = "RelWithDebInfo",
    [int]$LeafThreshold = 25,
    [int]$Iterations = 3,
    [int]$TimeoutSeconds = 300,
    [int]$BuildTimeoutSeconds = 900,
    [int]$ReportTimeoutSeconds = 120,
    [switch]$SkipBuild,
    [switch]$ForceBuild,
    [switch]$NoEtw,
    [switch]$NoInputAssumptions,
    [switch]$ResolveSymbols,
    [switch]$ForceStopExisting,
    [switch]$Help
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Show-Help {
    @"
Usage:
  powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1
  powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Lhs <lhs.obj> -Rhs <rhs.obj> -Op difference

Recommended:
  Run from an elevated PowerShell if you want ETW CPU stacks.
  Default build config is RelWithDebInfo for better ETW symbol quality.

Outputs:
  build\perf\run_<timestamp>\profile.log
  build\perf\run_<timestamp>\manifest.json
  build\perf\run_<timestamp>\timings.csv
  build\perf\run_<timestamp>\summary.txt
  build\perf\run_<timestamp>\xperf_profile_detail.txt
  build\perf\run_<timestamp>\xperf_stack_butterfly.txt
  build\perf\run_<timestamp>\xperf_process.txt
  build\perf\run_<timestamp>\xperf_process_cmdline.txt
  build\perf\run_<timestamp>\reember_cpu.etl

Notes:
  - Uses build\ only.
  - If -Lhs/-Rhs are omitted, the script tries common repo assets.
  - The default visual workloads include one known-overlap intersection case and one visual-test default pose difference case.
  - Reuses the executable only when the requested configuration already exists.
  - Re-run with -ForceBuild to rebuild the requested configuration.
  - Use -NoEtw for timing-only runs.
  - OBJ workloads are assumed to satisfy NSI/NNC input assumptions by default.
  - Use -NoInputAssumptions only when profiling intentionally invalid or adversarial OBJ inputs.
  - ETW uses xperf only. If ETW prerequisites are missing, the script fails instead of silently falling back.
  - Use -ResolveSymbols only when you need symbol-server lookup in addition to local PDB resolution.
"@
}

if ($Help) {
    Show-Help
    exit 0
}

$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
Set-Location -LiteralPath $RepoRoot

$BuildRoot = Join-Path $RepoRoot "build"
$RunStamp = Get-Date -Format "yyyyMMdd_HHmmss"
$PerfRoot = Join-Path $RepoRoot ("build\perf\run_{0}" -f $RunStamp)
New-Item -ItemType Directory -Force -Path $PerfRoot | Out-Null

$TranscriptPath = Join-Path $PerfRoot "profile.log"
$TranscriptStarted = $false

function Write-Info {
    param([string]$Message)
    $line = "[profile-re-ember] {0}" -f $Message
    Write-Host $line
}

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Get-CommandPath {
    param([string]$Name)

    $command = Get-Command $Name -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    if ($Name -ieq "xperf.exe") {
        $knownPaths = @()
        if (${env:ProgramFiles(x86)}) {
            $knownPaths += (Join-Path ${env:ProgramFiles(x86)} "Windows Kits\10\Windows Performance Toolkit\xperf.exe")
        }
        if ($env:ProgramFiles) {
            $knownPaths += (Join-Path $env:ProgramFiles "Windows Kits\10\Windows Performance Toolkit\xperf.exe")
        }

        foreach ($path in $knownPaths) {
            if (Test-Path -LiteralPath $path) {
                return $path
            }
        }
    }

    return $null
}

function Count-ObjFaces {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        return $null
    }

    return (Select-String -LiteralPath $Path -Pattern '^\s*f\s+' -ErrorAction Stop).Count
}

function Format-ObjScalar {
    param([double]$Value)

    return $Value.ToString("G17", [System.Globalization.CultureInfo]::InvariantCulture)
}

function Write-TranslatedObjCopy {
    param(
        [string]$SourcePath,
        [string]$DestinationPath,
        [double]$TranslateX,
        [double]$TranslateY,
        [double]$TranslateZ
    )

    if (-not (Test-Path -LiteralPath $SourcePath)) {
        throw ("Cannot translate missing OBJ: {0}" -f $SourcePath)
    }

    Ensure-ParentDirectory $DestinationPath

    $vertexPattern = '^(?<prefix>\s*v\s+)(?<x>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)' +
                     '(?<sep1>\s+)(?<y>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)' +
                     '(?<sep2>\s+)(?<z>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)(?<suffix>\s*(?:#.*)?)$'

    $translatedLines = New-Object System.Collections.Generic.List[string]
    foreach ($line in Get-Content -LiteralPath $SourcePath) {
        $match = [System.Text.RegularExpressions.Regex]::Match($line, $vertexPattern)
        if (-not $match.Success) {
            $translatedLines.Add($line)
            continue
        }

        $x = [double]::Parse($match.Groups["x"].Value, [System.Globalization.CultureInfo]::InvariantCulture) + $TranslateX
        $y = [double]::Parse($match.Groups["y"].Value, [System.Globalization.CultureInfo]::InvariantCulture) + $TranslateY
        $z = [double]::Parse($match.Groups["z"].Value, [System.Globalization.CultureInfo]::InvariantCulture) + $TranslateZ

        $translatedLines.Add(
            ("{0}{1}{2}{3}{4}{5}{6}" -f
                $match.Groups["prefix"].Value,
                (Format-ObjScalar $x),
                $match.Groups["sep1"].Value,
                (Format-ObjScalar $y),
                $match.Groups["sep2"].Value,
                (Format-ObjScalar $z),
                $match.Groups["suffix"].Value))
    }

    Set-Content -LiteralPath $DestinationPath -Value $translatedLines -Encoding UTF8
    return $DestinationPath
}

function Join-CommandLine {
    param([string[]]$Arguments)

    $quoted = foreach ($arg in $Arguments) {
        $value = [string]$arg
        if ($value.Length -eq 0) {
            '""'
        }
        elseif ($value -notmatch '[\s"]') {
            $value
        }
        else {
            '"' + ($value -replace '"', '\"') + '"'
        }
    }

    return ($quoted -join " ")
}

function Invoke-External {
    param(
        [string]$FilePath,
        [string[]]$Arguments,
        [string]$StdoutPath,
        [string]$StderrPath,
        [int]$TimeoutSec,
        [switch]$IgnoreExit
    )

    Write-Info ("EXEC: {0} {1}" -f $FilePath, ($Arguments -join " "))

    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $FilePath
    $startInfo.WorkingDirectory = $RepoRoot
    $startInfo.UseShellExecute = $false
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    $startInfo.Arguments = Join-CommandLine $Arguments

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $startInfo

    $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
    [void]$process.Start()
    $stdoutTask = $process.StandardOutput.ReadToEndAsync()
    $stderrTask = $process.StandardError.ReadToEndAsync()

    if (-not $process.WaitForExit($TimeoutSec * 1000)) {
        try {
            $process.Kill($true)
        }
        catch {
            $process.Kill()
        }
        $stopwatch.Stop()
        throw ("Timed out after {0}s: {1}" -f $TimeoutSec, $FilePath)
    }

    $stopwatch.Stop()
    $stdout = $stdoutTask.GetAwaiter().GetResult()
    $stderr = $stderrTask.GetAwaiter().GetResult()

    if ($StdoutPath) {
        Set-Content -LiteralPath $StdoutPath -Value $stdout -Encoding UTF8
    }
    if ($StderrPath) {
        Set-Content -LiteralPath $StderrPath -Value $stderr -Encoding UTF8
    }

    $result = [pscustomobject]@{
        FilePath = $FilePath
        Arguments = $Arguments
        ExitCode = $process.ExitCode
        ElapsedMs = [math]::Round($stopwatch.Elapsed.TotalMilliseconds, 3)
        StdoutPath = $StdoutPath
        StderrPath = $StderrPath
    }

    if (($process.ExitCode -ne 0) -and (-not $IgnoreExit)) {
        throw ("Command failed with exit code {0}: {1}" -f $process.ExitCode, $FilePath)
    }

    return $result
}

function Resolve-WorkloadPath {
    param([string]$Path)

    return (Resolve-Path -LiteralPath $Path).Path
}

function Resolve-OutputPath {
    param([string]$Path)

    if ([System.IO.Path]::IsPathRooted($Path)) {
        return $Path
    }

    return (Join-Path $RepoRoot $Path)
}

function Ensure-ParentDirectory {
    param([string]$Path)

    $parent = Split-Path -Parent $Path
    if ($parent) {
        New-Item -ItemType Directory -Force -Path $parent | Out-Null
    }
}

function Read-ReEmberTimingMetrics {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        throw ("Missing timing metrics file: {0}" -f $Path)
    }

    $metrics = @{}
    foreach ($line in Get-Content -LiteralPath $Path) {
        if ($line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)=(.+?)\s*$') {
            $metrics[$matches[1]] = [double]::Parse($matches[2], [System.Globalization.CultureInfo]::InvariantCulture)
        }
    }

    foreach ($requiredKey in @("read_ms", "prepare_ms", "solve_ms", "export_ms")) {
        if (-not $metrics.ContainsKey($requiredKey)) {
            throw ("Timing metrics file is missing key '{0}': {1}" -f $requiredKey, $Path)
        }
    }

    return [pscustomobject]@{
        ReadMs = [math]::Round($metrics["read_ms"], 3)
        PrepareMs = [math]::Round($metrics["prepare_ms"], 3)
        SolveMs = [math]::Round($metrics["solve_ms"], 3)
        ExportMs = [math]::Round($metrics["export_ms"], 3)
        InputPolygons = $(if ($metrics.ContainsKey("input_polygons")) { [math]::Round($metrics["input_polygons"], 0) } else { 0 })
        NodeCount = $(if ($metrics.ContainsKey("node_count")) { [math]::Round($metrics["node_count"], 0) } else { 0 })
        InternalNodeCount = $(if ($metrics.ContainsKey("internal_node_count")) { [math]::Round($metrics["internal_node_count"], 0) } else { 0 })
        LeafNodeCount = $(if ($metrics.ContainsKey("leaf_node_count")) { [math]::Round($metrics["leaf_node_count"], 0) } else { 0 })
        DiscardedNodeCount = $(if ($metrics.ContainsKey("discarded_node_count")) { [math]::Round($metrics["discarded_node_count"], 0) } else { 0 })
        MaxDepth = $(if ($metrics.ContainsKey("max_depth")) { [math]::Round($metrics["max_depth"], 0) } else { 0 })
        TotalPolygonCount = $(if ($metrics.ContainsKey("total_polygon_count")) { [math]::Round($metrics["total_polygon_count"], 0) } else { 0 })
        LeafFragmentCount = $(if ($metrics.ContainsKey("leaf_fragment_count")) { [math]::Round($metrics["leaf_fragment_count"], 0) } else { 0 })
        ClassifiedFragmentCount = $(if ($metrics.ContainsKey("classified_fragment_count")) { [math]::Round($metrics["classified_fragment_count"], 0) } else { 0 })
        ResultFragmentCount = $(if ($metrics.ContainsKey("result_fragment_count")) { [math]::Round($metrics["result_fragment_count"], 0) } else { 0 })
        ConstantDiscardCount = $(if ($metrics.ContainsKey("constant_discard_count")) { [math]::Round($metrics["constant_discard_count"], 0) } else { 0 })
        LeafThresholdStopCount = $(if ($metrics.ContainsKey("leaf_threshold_stop_count")) { [math]::Round($metrics["leaf_threshold_stop_count"], 0) } else { 0 })
        AabbNotSplittableStopCount = $(if ($metrics.ContainsKey("aabb_not_splittable_stop_count")) { [math]::Round($metrics["aabb_not_splittable_stop_count"], 0) } else { 0 })
        SplitFailureStopCount = $(if ($metrics.ContainsKey("split_failure_stop_count")) { [math]::Round($metrics["split_failure_stop_count"], 0) } else { 0 })
        WntvAwareSplitCount = $(if ($metrics.ContainsKey("wntv_aware_split_count")) { [math]::Round($metrics["wntv_aware_split_count"], 0) } else { 0 })
        CenterRangeSplitCount = $(if ($metrics.ContainsKey("center_range_split_count")) { [math]::Round($metrics["center_range_split_count"], 0) } else { 0 })
        MidpointSplitCount = $(if ($metrics.ContainsKey("midpoint_split_count")) { [math]::Round($metrics["midpoint_split_count"], 0) } else { 0 })
        ChildReferenceReuseCount = $(if ($metrics.ContainsKey("child_reference_reuse_count")) { [math]::Round($metrics["child_reference_reuse_count"], 0) } else { 0 })
        ChildReferenceTraceCount = $(if ($metrics.ContainsKey("child_reference_trace_count")) { [math]::Round($metrics["child_reference_trace_count"], 0) } else { 0 })
        SingleOperandLeafBspSkipCount = $(if ($metrics.ContainsKey("single_operand_leaf_bsp_skip_count")) { [math]::Round($metrics["single_operand_leaf_bsp_skip_count"], 0) } else { 0 })
        SingleOperandClassificationReuseCount = $(if ($metrics.ContainsKey("single_operand_classification_reuse_count")) { [math]::Round($metrics["single_operand_classification_reuse_count"], 0) } else { 0 })
        LeafBspBuildCount = $(if ($metrics.ContainsKey("leaf_bsp_build_count")) { [math]::Round($metrics["leaf_bsp_build_count"], 0) } else { 0 })
    }
}

function Get-CMakeCacheValue {
    param(
        [string]$CachePath,
        [string]$Key
    )

    if (-not (Test-Path -LiteralPath $CachePath)) {
        return $null
    }

    $pattern = '^{0}(?::[^=]+)?=(.*)$' -f [regex]::Escape($Key)
    $match = Select-String -LiteralPath $CachePath -Pattern $pattern | Select-Object -First 1
    if ($null -eq $match) {
        return $null
    }

    return $match.Matches[0].Groups[1].Value
}

function Test-CMakeMultiConfig {
    param([string]$CachePath)

    $configTypes = Get-CMakeCacheValue $CachePath "CMAKE_CONFIGURATION_TYPES"
    return (-not [string]::IsNullOrWhiteSpace($configTypes))
}

function Get-ReEmberExecutableCandidates {
    param(
        [string]$BuildDir,
        [string]$BuildConfiguration
    )

    $candidates = New-Object System.Collections.Generic.List[string]
    $cachePath = Join-Path $BuildDir "CMakeCache.txt"

    if (Test-CMakeMultiConfig $cachePath) {
        $candidates.Add((Join-Path $BuildDir ("{0}\re-EMBER.exe" -f $BuildConfiguration)))
    }
    else {
        $candidates.Add((Join-Path $BuildDir "re-EMBER.exe"))
        $candidates.Add((Join-Path $BuildDir ("{0}\re-EMBER.exe" -f $BuildConfiguration)))
    }

    return $candidates | Select-Object -Unique
}

function Resolve-ReEmberExecutablePath {
    param(
        [string]$BuildDir,
        [string]$BuildConfiguration
    )

    foreach ($candidate in Get-ReEmberExecutableCandidates $BuildDir $BuildConfiguration) {
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    return $null
}

function Get-BuildCommandArguments {
    param(
        [string]$BuildDir,
        [string]$BuildConfiguration
    )

    $args = New-Object System.Collections.Generic.List[string]
    $args.Add("--build")
    $args.Add($BuildDir)
    if (Test-CMakeMultiConfig (Join-Path $BuildDir "CMakeCache.txt")) {
        $args.Add("--config")
        $args.Add($BuildConfiguration)
    }
    $args.Add("--target")
    $args.Add("re-EMBER")
    return $args
}

function Throw-FriendlyCMakeFailure {
    param(
        [string]$StepName,
        [string]$StdoutPath,
        [string]$StderrPath,
        [System.Management.Automation.ErrorRecord]$ErrorRecord
    )

    $diagnosticText = New-Object System.Collections.Generic.List[string]
    foreach ($path in @($StdoutPath, $StderrPath)) {
        if ($path -and (Test-Path -LiteralPath $path)) {
            $diagnosticText.Add((Get-Content -LiteralPath $path -Raw))
        }
    }

    $combined = $diagnosticText -join "`n"
    if ($combined -match 'generate\.stamp' -or $combined -match 'Cannot restore timestamp') {
        throw ("{0} failed because build\CMakeFiles\generate.stamp could not be updated. The current build tree is locked or not writable. Fix or recreate build\ before rebuilding." -f $StepName)
    }

    throw $ErrorRecord
}

function New-Workload {
    param(
        [string]$Name,
        [string]$LhsPath,
        [string]$RhsPath,
        [string]$Operation,
        [string]$OutPath
    )

    return [pscustomobject]@{
        Name = $Name
        Lhs = Resolve-WorkloadPath $LhsPath
        Rhs = Resolve-WorkloadPath $RhsPath
        Op = $Operation
        Out = Resolve-OutputPath $OutPath
        LhsFaces = Count-ObjFaces $LhsPath
        RhsFaces = Count-ObjFaces $RhsPath
    }
}

function Get-Workloads {
    $items = New-Object System.Collections.Generic.List[object]
    $missingDefaults = New-Object System.Collections.Generic.List[string]

    if (($Lhs -and -not $Rhs) -or ($Rhs -and -not $Lhs)) {
        throw "Specify both -Lhs and -Rhs, or omit both to use default workloads."
    }

    if ($Lhs -and $Rhs) {
        $outputPath = $Out
        if (-not $outputPath) {
            $outputPath = Join-Path $PerfRoot ("result_custom_{0}.obj" -f $Op)
        }
        $items.Add((New-Workload "custom" $Lhs $Rhs $Op $outputPath))
        return $items
    }

    $visualToolSource = "assets\visual_test\tool_box.obj"
    $visualToolOverlapPosePath = Join-Path $PerfRoot "visual_test_overlap_pose_tool.obj"
    $visualToolUiDefaultPosePath = Join-Path $PerfRoot "visual_test_ui_default_pose_tool.obj"
    if (Test-Path -LiteralPath $visualToolSource) {
        [void](Write-TranslatedObjCopy $visualToolSource $visualToolOverlapPosePath 0.5 0.2 0.35)
        [void](Write-TranslatedObjCopy $visualToolSource $visualToolUiDefaultPosePath 0.5 0.5 0.35)
    }

    $defaults = @(
        [pscustomobject]@{
            Name = "icosphere80_toolbox_difference"
            Lhs = "assets\models\test_icosphere_80.obj"
            Rhs = "assets\models\tool_box.obj"
            Op = "difference"
        },
        [pscustomobject]@{
            Name = "visual_block_toolbox_overlap_intersection"
            Lhs = "assets\visual_test\workpiece_block.obj"
            Rhs = $visualToolOverlapPosePath
            Op = "intersection"
        },
        [pscustomobject]@{
            Name = "visual_block_toolbox_visual_test_default_difference"
            Lhs = "assets\visual_test\workpiece_block.obj"
            Rhs = $visualToolUiDefaultPosePath
            Op = "difference"
        }
    )

    foreach ($item in $defaults) {
        if ((Test-Path -LiteralPath $item.Lhs) -and (Test-Path -LiteralPath $item.Rhs)) {
            $outPath = Join-Path $PerfRoot ("result_{0}.obj" -f $item.Name)
            $items.Add((New-Workload $item.Name $item.Lhs $item.Rhs $item.Op $outPath))
        }
        else {
            $missingDefaults.Add($item.Name)
        }
    }

    if ($missingDefaults.Count -gt 0) {
        throw ("Default workloads are incomplete. Missing assets for: {0}. Re-run with explicit -Lhs/-Rhs or restore the expected assets." -f ($missingDefaults -join ", "))
    }

    return $items
}

function Get-ReEmberArguments {
    param(
        [object]$Workload,
        [string]$OutputPath,
        [string]$MetricsPath
    )

    $arguments = @(
        "--lhs", $Workload.Lhs,
        "--rhs", $Workload.Rhs,
        "--op", $Workload.Op,
        "--out", $OutputPath,
        "--leaf-threshold", [string]$LeafThreshold,
        "--timings-out", $MetricsPath
    )

    if (-not $NoInputAssumptions) {
        $arguments += @(
            "--assume-lhs-nsi",
            "--assume-lhs-nnc",
            "--assume-rhs-nsi",
            "--assume-rhs-nnc"
        )
    }

    return $arguments
}

function Invoke-ReEmberWorkload {
    param(
        [string]$ExePath,
        [object]$Workload,
        [string]$Tag
    )

    $stdoutPath = Join-Path $PerfRoot ("{0}_{1}.stdout.txt" -f $Tag, $Workload.Name)
    $stderrPath = Join-Path $PerfRoot ("{0}_{1}.stderr.txt" -f $Tag, $Workload.Name)
    $metricsPath = Join-Path $PerfRoot ("{0}_{1}.metrics.txt" -f $Tag, $Workload.Name)
    $outPath = $Workload.Out
    if ($Tag -notlike "timing*") {
        $outPath = Join-Path $PerfRoot ("result_{0}_{1}.obj" -f $Tag, $Workload.Name)
    }

    Ensure-ParentDirectory $outPath
    $result = Invoke-External $ExePath (Get-ReEmberArguments $Workload $outPath $metricsPath) $stdoutPath $stderrPath $TimeoutSeconds
    $metrics = Read-ReEmberTimingMetrics $metricsPath
    return [pscustomobject]@{
        FilePath = $result.FilePath
        Arguments = $result.Arguments
        ExitCode = $result.ExitCode
        ElapsedMs = $result.ElapsedMs
        StdoutPath = $result.StdoutPath
        StderrPath = $result.StderrPath
        MetricsPath = $metricsPath
        ReadMs = $metrics.ReadMs
        PrepareMs = $metrics.PrepareMs
        SolveMs = $metrics.SolveMs
        ExportMs = $metrics.ExportMs
        InputPolygons = $metrics.InputPolygons
        NodeCount = $metrics.NodeCount
        InternalNodeCount = $metrics.InternalNodeCount
        LeafNodeCount = $metrics.LeafNodeCount
        DiscardedNodeCount = $metrics.DiscardedNodeCount
        MaxDepth = $metrics.MaxDepth
        TotalPolygonCount = $metrics.TotalPolygonCount
        LeafFragmentCount = $metrics.LeafFragmentCount
        ClassifiedFragmentCount = $metrics.ClassifiedFragmentCount
        ResultFragmentCount = $metrics.ResultFragmentCount
        ConstantDiscardCount = $metrics.ConstantDiscardCount
        LeafThresholdStopCount = $metrics.LeafThresholdStopCount
        AabbNotSplittableStopCount = $metrics.AabbNotSplittableStopCount
        SplitFailureStopCount = $metrics.SplitFailureStopCount
        WntvAwareSplitCount = $metrics.WntvAwareSplitCount
        CenterRangeSplitCount = $metrics.CenterRangeSplitCount
        MidpointSplitCount = $metrics.MidpointSplitCount
        ChildReferenceReuseCount = $metrics.ChildReferenceReuseCount
        ChildReferenceTraceCount = $metrics.ChildReferenceTraceCount
        SingleOperandLeafBspSkipCount = $metrics.SingleOperandLeafBspSkipCount
        SingleOperandClassificationReuseCount = $metrics.SingleOperandClassificationReuseCount
        LeafBspBuildCount = $metrics.LeafBspBuildCount
    }
}

function Export-XperfReports {
    param(
        [string]$XperfPath,
        [string]$EtlPath,
        [string]$ExePath,
        [int]$ReportTimeoutSec
    )

    $exeDir = Split-Path -Parent $ExePath
    $symbolCache = Join-Path $PerfRoot "symcache"
    $profileOut = Join-Path $PerfRoot "xperf_profile_detail.txt"
    $profileErr = Join-Path $PerfRoot "xperf_profile_detail.err.txt"
    $stackOut = Join-Path $PerfRoot "xperf_stack_butterfly.txt"
    $stackErr = Join-Path $PerfRoot "xperf_stack_butterfly.err.txt"
    $processOut = Join-Path $PerfRoot "xperf_process.txt"
    $processErr = Join-Path $PerfRoot "xperf_process.err.txt"
    $processCmdlineOut = Join-Path $PerfRoot "xperf_process_cmdline.txt"
    $processCmdlineErr = Join-Path $PerfRoot "xperf_process_cmdline.err.txt"

    $previousSymbolPath = $env:_NT_SYMBOL_PATH
    $previousSymCachePath = $env:_NT_SYMCACHE_PATH

    try {
        $symbolPathEntries = @($exeDir)
        if ($ResolveSymbols) {
            New-Item -ItemType Directory -Force -Path $symbolCache | Out-Null
            $symbolPathEntries += ("srv*{0}*https://msdl.microsoft.com/download/symbols" -f $symbolCache)
            $env:_NT_SYMCACHE_PATH = $symbolCache
        }
        else {
            Remove-Item Env:_NT_SYMCACHE_PATH -ErrorAction SilentlyContinue
        }
        $env:_NT_SYMBOL_PATH = ($symbolPathEntries -join ";")

        $reports = @(
            [pscustomobject]@{
                Name = "xperf profile"
                Args = @("-i", $EtlPath, "-symbols", "-o", $profileOut, "-a", "profile", "-detail")
                Stdout = $null
                Stderr = $profileErr
            },
            [pscustomobject]@{
                Name = "xperf stack"
                Args = @("-i", $EtlPath, "-symbols", "-o", $stackOut, "-a", "stack", "-process", "re-EMBER.exe", "-butterfly", "1")
                Stdout = $null
                Stderr = $stackErr
            },
            [pscustomobject]@{
                Name = "xperf process"
                Args = @("-i", $EtlPath, "-o", $processOut, "-a", "process")
                Stdout = $null
                Stderr = $processErr
            },
            [pscustomobject]@{
                Name = "xperf process cmdline"
                Args = @("-i", $EtlPath, "-o", $processCmdlineOut, "-a", "process", "-withcmdline")
                Stdout = $null
                Stderr = $processCmdlineErr
            }
        )

        foreach ($report in $reports) {
            try {
                [void](Invoke-External $XperfPath $report.Args $report.Stdout $report.Stderr $ReportTimeoutSec)
            }
            catch {
                throw ("Failed to export {0}: {1}" -f $report.Name, $_.Exception.Message)
            }
        }
    }
    finally {
        if ($null -ne $previousSymbolPath) {
            $env:_NT_SYMBOL_PATH = $previousSymbolPath
        }
        else {
            Remove-Item Env:_NT_SYMBOL_PATH -ErrorAction SilentlyContinue
        }

        if ($null -ne $previousSymCachePath) {
            $env:_NT_SYMCACHE_PATH = $previousSymCachePath
        }
        else {
            Remove-Item Env:_NT_SYMCACHE_PATH -ErrorAction SilentlyContinue
        }
    }

    return [pscustomobject]@{
        EtlPath = $EtlPath
        ProfilePath = $profileOut
        StackPath = $stackOut
        ProcessPath = $processOut
        ProcessCmdlinePath = $processCmdlineOut
    }
}

function Invoke-EtwProfile {
    param(
        [string]$ExePath,
        [object[]]$Workloads,
        [string]$XperfPath,
        [bool]$IsAdmin
    )

    if ($NoEtw) {
        Write-Info "Skipping ETW because -NoEtw was specified."
        return $null
    }

    if (-not $IsAdmin) {
        throw "ETW profiling requires an elevated PowerShell session. Re-run as Administrator or use -NoEtw."
    }

    if (-not $XperfPath) {
        throw "xperf.exe was not found. Install Windows Performance Toolkit or use -NoEtw."
    }

    $etlPath = Join-Path $PerfRoot "reember_cpu.etl"
    $rawEtlPath = Join-Path $PerfRoot "reember_cpu_raw.etl"
    $started = $false

    try {
        if ($ForceStopExisting) {
            Write-Info "Force-stopping any existing xperf kernel logger."
            [void](Invoke-External $XperfPath @("-stop") $null (Join-Path $PerfRoot "xperf_force_stop.err.txt") 60 -IgnoreExit)
        }

        [void](Invoke-External $XperfPath @(
            "-on", "PROC_THREAD+LOADER+PROFILE",
            "-stackwalk", "Profile",
            "-BufferSize", "1024",
            "-MaxFile", "512",
            "-FileMode", "Circular",
            "-f", $rawEtlPath
        ) (Join-Path $PerfRoot "xperf_start.stdout.txt") (Join-Path $PerfRoot "xperf_start.stderr.txt") 60)
        $started = $true

        foreach ($workload in $Workloads) {
            [void](Invoke-ReEmberWorkload $ExePath $workload "etw")
        }

        [void](Invoke-External $XperfPath @("-d", $etlPath) (Join-Path $PerfRoot "xperf_stop.stdout.txt") (Join-Path $PerfRoot "xperf_stop.stderr.txt") 180)
        $started = $false

        if (-not (Test-Path -LiteralPath $etlPath)) {
            throw "xperf stop completed but reember_cpu.etl was not produced."
        }

        return Export-XperfReports $XperfPath $etlPath $ExePath $ReportTimeoutSeconds
    }
    finally {
        if ($started) {
            try {
                [void](Invoke-External $XperfPath @("-stop") $null (Join-Path $PerfRoot "xperf_stop_recovery.err.txt") 60 -IgnoreExit)
            }
            catch {
                Write-Warning ("Failed to stop xperf cleanly: {0}" -f $_.Exception.Message)
            }
        }
    }
}

function Assert-EtwPrerequisites {
    param(
        [string]$XperfPath,
        [bool]$IsAdmin
    )

    if ($NoEtw) {
        return
    }

    if (-not $IsAdmin) {
        throw "ETW profiling requires an elevated PowerShell session. Re-run as Administrator or use -NoEtw."
    }

    if (-not $XperfPath) {
        throw "xperf.exe was not found. Install Windows Performance Toolkit or use -NoEtw."
    }
}

try {
    Start-Transcript -Path $TranscriptPath -Force | Out-Null
    $TranscriptStarted = $true

    $isAdmin = Test-IsAdministrator
    $cmakePath = Get-CommandPath "cmake.exe"
    $xperfPath = Get-CommandPath "xperf.exe"

    Write-Info ("Repo root: {0}" -f $RepoRoot)
    Write-Info ("Build dir: {0}" -f $BuildRoot)
    Write-Info ("Output dir: {0}" -f $PerfRoot)
    Write-Info ("Configuration: {0}" -f $Configuration)
    Write-Info ("Administrator: {0}" -f $isAdmin)
    Write-Info ("cmake: {0}" -f $(if ($cmakePath) { $cmakePath } else { "<missing>" }))
    Write-Info ("xperf: {0}" -f $(if ($xperfPath) { $xperfPath } else { "<missing>" }))
    Assert-EtwPrerequisites $xperfPath $isAdmin

    $cachePath = Join-Path $BuildRoot "CMakeCache.txt"
    $multiConfig = Test-CMakeMultiConfig $cachePath
    $generator = Get-CMakeCacheValue $cachePath "CMAKE_GENERATOR"
    $configuredBuildType = Get-CMakeCacheValue $cachePath "CMAKE_BUILD_TYPE"
    $exePath = Resolve-ReEmberExecutablePath $BuildRoot $Configuration

    if ($generator) {
        Write-Info ("CMake generator: {0}" -f $generator)
    }
    if ((-not $multiConfig) -and $configuredBuildType) {
        Write-Info ("Single-config build type: {0}" -f $configuredBuildType)
    }

    if (($configuredBuildType) -and (-not $multiConfig) -and ($configuredBuildType -ine $Configuration) -and $SkipBuild) {
        throw ("build\ is configured as {0}, but -Configuration requested {1} with -SkipBuild." -f $configuredBuildType, $Configuration)
    }

    if (($null -eq $exePath) -and $SkipBuild) {
        $expectedLocations = Get-ReEmberExecutableCandidates $BuildRoot $Configuration
        throw ("Missing executable and -SkipBuild was specified. Checked: {0}" -f ($expectedLocations -join ", "))
    }

    if (-not $SkipBuild) {
        if (-not $cmakePath) {
            throw "cmake.exe was not found in PATH."
        }

        $needsConfigure = -not (Test-Path -LiteralPath $cachePath)
        $needsBuild = $ForceBuild -or (-not $exePath)

        if ((-not $multiConfig) -and $configuredBuildType -and ($configuredBuildType -ine $Configuration)) {
            $needsConfigure = $true
            $needsBuild = $true
        }

        if ($needsConfigure) {
            $configureStdoutPath = Join-Path $PerfRoot "cmake_configure.stdout.txt"
            $configureStderrPath = Join-Path $PerfRoot "cmake_configure.stderr.txt"
            try {
                [void](Invoke-External $cmakePath @("-S", ".", "-B", "build", "-DCMAKE_BUILD_TYPE=$Configuration") $configureStdoutPath $configureStderrPath $BuildTimeoutSeconds)
            }
            catch {
                Throw-FriendlyCMakeFailure "CMake configure" $configureStdoutPath $configureStderrPath $_
            }
            $cachePath = Join-Path $BuildRoot "CMakeCache.txt"
            $multiConfig = Test-CMakeMultiConfig $cachePath
            $generator = Get-CMakeCacheValue $cachePath "CMAKE_GENERATOR"
            $configuredBuildType = Get-CMakeCacheValue $cachePath "CMAKE_BUILD_TYPE"
            if ($generator) {
                Write-Info ("Configured generator: {0}" -f $generator)
            }
        }

        if ($needsBuild) {
            $buildStdoutPath = Join-Path $PerfRoot "cmake_build.stdout.txt"
            $buildStderrPath = Join-Path $PerfRoot "cmake_build.stderr.txt"
            try {
                [void](Invoke-External $cmakePath (Get-BuildCommandArguments $BuildRoot $Configuration) $buildStdoutPath $buildStderrPath $BuildTimeoutSeconds)
            }
            catch {
                Throw-FriendlyCMakeFailure "CMake build" $buildStdoutPath $buildStderrPath $_
            }
        }
        else {
            Write-Info ("Reusing existing executable for requested configuration: {0}" -f $Configuration)
        }
    }

    $exePath = Resolve-ReEmberExecutablePath $BuildRoot $Configuration
    if ([string]::IsNullOrWhiteSpace($exePath) -or (-not (Test-Path -LiteralPath $exePath))) {
        $expectedLocations = Get-ReEmberExecutableCandidates $BuildRoot $Configuration
        throw ("Missing executable after build. Checked: {0}" -f ($expectedLocations -join ", "))
    }

    $workloads = @(Get-Workloads)
    $manifest = [pscustomobject]@{
        repoRoot = [string]$RepoRoot
        buildDir = [string]$BuildRoot
        outputDir = [string]$PerfRoot
        timestamp = $RunStamp
        administrator = $isAdmin
        configuration = $Configuration
        executable = $exePath
        leafThreshold = $LeafThreshold
        iterations = $Iterations
        timeoutSeconds = $TimeoutSeconds
        etwEnabled = (-not $NoEtw)
        inputAssumptionsEnabled = (-not $NoInputAssumptions)
        workloads = $workloads
    }
    $manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $PerfRoot "manifest.json") -Encoding UTF8

    $timingRows = New-Object System.Collections.Generic.List[object]
    foreach ($workload in $workloads) {
        Write-Info ("Timing workload {0}: lhs_faces={1} rhs_faces={2} op={3}" -f $workload.Name, $workload.LhsFaces, $workload.RhsFaces, $workload.Op)
        for ($i = 1; $i -le $Iterations; ++$i) {
            $result = Invoke-ReEmberWorkload $exePath $workload ("timing_{0}" -f $i)
        $timingRows.Add([pscustomobject]@{
            workload = $workload.Name
            iteration = $i
            elapsed_ms = $result.ElapsedMs
            read_ms = $result.ReadMs
            prepare_ms = $result.PrepareMs
            solve_ms = $result.SolveMs
            export_ms = $result.ExportMs
            input_polygons = $result.InputPolygons
            node_count = $result.NodeCount
            internal_node_count = $result.InternalNodeCount
            leaf_node_count = $result.LeafNodeCount
            discarded_node_count = $result.DiscardedNodeCount
            max_depth = $result.MaxDepth
            total_polygon_count = $result.TotalPolygonCount
            leaf_fragment_count = $result.LeafFragmentCount
            classified_fragment_count = $result.ClassifiedFragmentCount
            result_fragment_count = $result.ResultFragmentCount
            constant_discard_count = $result.ConstantDiscardCount
            leaf_threshold_stop_count = $result.LeafThresholdStopCount
            aabb_not_splittable_stop_count = $result.AabbNotSplittableStopCount
            split_failure_stop_count = $result.SplitFailureStopCount
            wntv_aware_split_count = $result.WntvAwareSplitCount
            center_range_split_count = $result.CenterRangeSplitCount
            midpoint_split_count = $result.MidpointSplitCount
            child_reference_reuse_count = $result.ChildReferenceReuseCount
            child_reference_trace_count = $result.ChildReferenceTraceCount
            single_operand_leaf_bsp_skip_count = $result.SingleOperandLeafBspSkipCount
            single_operand_classification_reuse_count = $result.SingleOperandClassificationReuseCount
            leaf_bsp_build_count = $result.LeafBspBuildCount
            exit_code = $result.ExitCode
            lhs_faces = $workload.LhsFaces
            rhs_faces = $workload.RhsFaces
            op = $workload.Op
                output = $(if (Test-Path -LiteralPath $workload.Out) { $workload.Out } else { $null })
                stdout = $result.StdoutPath
                stderr = $result.StderrPath
                metrics = $result.MetricsPath
            })
        }
    }

    $timingCsvPath = Join-Path $PerfRoot "timings.csv"
    $timingRows | Export-Csv -LiteralPath $timingCsvPath -NoTypeInformation -Encoding UTF8

    $etwArtifacts = Invoke-EtwProfile $exePath $workloads $xperfPath $isAdmin

    $summaryLines = New-Object System.Collections.Generic.List[string]
    $summaryLines.Add(("executable={0}" -f $exePath))
    $summaryLines.Add(("configuration={0}" -f $Configuration))
    $summaryLines.Add(("iterations={0}" -f $Iterations))
    $summaryLines.Add(("timings_csv={0}" -f $timingCsvPath))
    foreach ($group in ($timingRows | Group-Object workload)) {
        $elapsedValues = @($group.Group | ForEach-Object { [double]$_.elapsed_ms })
        $readValues = @($group.Group | ForEach-Object { [double]$_.read_ms })
        $prepareValues = @($group.Group | ForEach-Object { [double]$_.prepare_ms })
        $solveValues = @($group.Group | ForEach-Object { [double]$_.solve_ms })
        $exportValues = @($group.Group | ForEach-Object { [double]$_.export_ms })
        $nodeCountValues = @($group.Group | ForEach-Object { [double]$_.node_count })
        $leafNodeValues = @($group.Group | ForEach-Object { [double]$_.leaf_node_count })
        $discardedNodeValues = @($group.Group | ForEach-Object { [double]$_.discarded_node_count })
        $maxDepthValues = @($group.Group | ForEach-Object { [double]$_.max_depth })
        $summaryLines.Add(("workload={0}" -f $group.Name))
        $summaryLines.Add(("  min_ms={0}" -f ([math]::Round(($elapsedValues | Measure-Object -Minimum).Minimum, 3))))
        $summaryLines.Add(("  max_ms={0}" -f ([math]::Round(($elapsedValues | Measure-Object -Maximum).Maximum, 3))))
        $summaryLines.Add(("  avg_ms={0}" -f ([math]::Round(($elapsedValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_read_ms={0}" -f ([math]::Round(($readValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_prepare_ms={0}" -f ([math]::Round(($prepareValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_solve_ms={0}" -f ([math]::Round(($solveValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_export_ms={0}" -f ([math]::Round(($exportValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_node_count={0}" -f ([math]::Round(($nodeCountValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_leaf_node_count={0}" -f ([math]::Round(($leafNodeValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_discarded_node_count={0}" -f ([math]::Round(($discardedNodeValues | Measure-Object -Average).Average, 3))))
        $summaryLines.Add(("  avg_max_depth={0}" -f ([math]::Round(($maxDepthValues | Measure-Object -Average).Average, 3))))
    }
    if ($etwArtifacts) {
        $summaryLines.Add(("etl={0}" -f $etwArtifacts.EtlPath))
        $summaryLines.Add(("xperf_profile_detail={0}" -f $etwArtifacts.ProfilePath))
        $summaryLines.Add(("xperf_stack_butterfly={0}" -f $etwArtifacts.StackPath))
        $summaryLines.Add(("xperf_process={0}" -f $etwArtifacts.ProcessPath))
        $summaryLines.Add(("xperf_process_cmdline={0}" -f $etwArtifacts.ProcessCmdlinePath))
    }
    Set-Content -LiteralPath (Join-Path $PerfRoot "summary.txt") -Value $summaryLines -Encoding UTF8

    Write-Info "Done. Send profile.log, timings.csv, summary.txt, and xperf_*.txt back for analysis. Keep reember_cpu.etl for WPA."
}
finally {
    if ($TranscriptStarted) {
        Stop-Transcript | Out-Null
    }
}
