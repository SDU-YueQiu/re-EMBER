# Capture re-EMBER CLI workloads with Tracy and write reports under build\perf.
param(
    [string]$Lhs,
    [string]$Rhs,
    [ValidateSet("union", "intersection", "difference")][string]$Op = "difference",
    [string]$Out,
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")][string]$Configuration = "RelWithDebInfo",
    [int]$LeafThreshold = 25,
    [int]$Iterations = 3,
    [int]$TimeoutSeconds = 300,
    [int]$BuildTimeoutSeconds = 900,
    [int]$ReportTimeoutSeconds = 120,
    [int]$TracyPort = 8086,
    [int]$TracyAttachWaitMilliseconds = 1500,
    [string]$TracyToolRoot,
    [switch]$SkipBuild,
    [switch]$ForceBuild,
    [switch]$NoTracy,
    [switch]$NoInputAssumptions,
    [switch]$Help
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$TracyPortWasExplicit = $PSBoundParameters.ContainsKey("TracyPort")

function Show-Help {
    @"
Usage:
  powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1
  powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Lhs <lhs.obj> -Rhs <rhs.obj> -Op difference

Outputs:
  build\perf\run_<timestamp>\profile.log
  build\perf\run_<timestamp>\manifest.json
  build\perf\run_<timestamp>\timings.csv
  build\perf\run_<timestamp>\tracy_traces\*.tracy
  build\perf\run_<timestamp>\tracy_zones.csv
  build\perf\run_<timestamp>\report.md
  build\perf\run_<timestamp>\summary.txt

Notes:
  - Uses build\ only.
  - Tracy profiling is the default path and requires REEMBER_ENABLE_TRACY=ON.
  - When -SkipBuild is not specified, the script configures build\ with -DREEMBER_ENABLE_TRACY=ON.
  - Install the required tools with: vcpkg install tracy[cli-tools]:x64-windows
  - The script auto-selects a bindable Tracy port when the default port is reserved on the local machine.
  - The script sets REEMBER_TRACY_WAIT_MS before timed work so tracy-capture can attach without polluting read/prepare/solve/export timings.
  - Use -NoTracy only for timing-only runs without zone capture.
  - OBJ workloads are assumed to satisfy NSI/NNC input assumptions by default.
  - Use -NoInputAssumptions only when profiling intentionally invalid or adversarial OBJ inputs.
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
$TraceRoot = Join-Path $PerfRoot "tracy_traces"
New-Item -ItemType Directory -Force -Path $PerfRoot | Out-Null
New-Item -ItemType Directory -Force -Path $TraceRoot | Out-Null

$TranscriptPath = Join-Path $PerfRoot "profile.log"
$TranscriptStarted = $false

function Write-Info {
    param([string]$Message)
    Write-Host ("[profile-re-ember] {0}" -f $Message)
}

function Ensure-ParentDirectory {
    param([string]$Path)

    $parent = Split-Path -Parent $Path
    if ($parent) {
        New-Item -ItemType Directory -Force -Path $parent | Out-Null
    }
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
        [hashtable]$EnvironmentVariables,
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
    if ($EnvironmentVariables) {
        foreach ($entry in $EnvironmentVariables.GetEnumerator()) {
            $startInfo.Environment[$entry.Key] = [string]$entry.Value
        }
    }

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

function Test-TcpLoopbackBindable {
    param([int]$Port)

    $listener = $null
    try {
        $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, $Port)
        $listener.Start()
        return $true
    }
    catch {
        return $false
    }
    finally {
        if ($listener) {
            $listener.Stop()
        }
    }
}

function Get-EphemeralTcpPort {
    $listener = [System.Net.Sockets.TcpListener]::new([System.Net.IPAddress]::Loopback, 0)
    try {
        $listener.Start()
        return $listener.LocalEndpoint.Port
    }
    finally {
        $listener.Stop()
    }
}

function Resolve-TracyPort {
    param(
        [int]$PreferredPort,
        [bool]$WasExplicit
    )

    if ($WasExplicit) {
        if (-not (Test-TcpLoopbackBindable $PreferredPort)) {
            throw ("Requested -TracyPort {0} is not bindable on 127.0.0.1. Choose another port." -f $PreferredPort)
        }
        return $PreferredPort
    }

    for ($candidate = $PreferredPort; $candidate -lt ($PreferredPort + 200); ++$candidate) {
        if (Test-TcpLoopbackBindable $candidate) {
            if ($candidate -ne $PreferredPort) {
                Write-Info ("Default Tracy port {0} is unavailable; using {1} instead." -f $PreferredPort, $candidate)
            }
            return $candidate
        }
    }

    $fallbackPort = Get-EphemeralTcpPort
    Write-Info ("Default Tracy port window near {0} is unavailable; using ephemeral port {1}." -f $PreferredPort, $fallbackPort)
    return $fallbackPort
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

    return $metrics
}

function Get-Metric {
    param(
        [hashtable]$Metrics,
        [string]$Key,
        [double]$Default = 0
    )

    if ($Metrics.ContainsKey($Key)) {
        return $Metrics[$Key]
    }

    return $Default
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

function Test-TracyEnabledInBuild {
    param([string]$BuildDir)

    $cachePath = Join-Path $BuildDir "CMakeCache.txt"
    $value = Get-CMakeCacheValue $cachePath "REEMBER_ENABLE_TRACY"
    return ($value -eq "ON" -or $value -eq "TRUE" -or $value -eq "1")
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

function Invoke-CMakeConfigure {
    $args = @("-S", ".", "-B", $BuildRoot)
    if (-not $NoTracy) {
        $args += "-DREEMBER_ENABLE_TRACY=ON"
    }

    Invoke-External "cmake" $args (Join-Path $PerfRoot "cmake_configure.stdout.txt") (Join-Path $PerfRoot "cmake_configure.stderr.txt") $BuildTimeoutSeconds | Out-Null
}

function Invoke-CMakeBuild {
    $args = New-Object System.Collections.Generic.List[string]
    $args.Add("--build")
    $args.Add($BuildRoot)
    if (Test-CMakeMultiConfig (Join-Path $BuildRoot "CMakeCache.txt")) {
        $args.Add("--config")
        $args.Add($Configuration)
    }
    $args.Add("--target")
    $args.Add("re-EMBER")

    Invoke-External "cmake" $args.ToArray() (Join-Path $PerfRoot "cmake_build.stdout.txt") (Join-Path $PerfRoot "cmake_build.stderr.txt") $BuildTimeoutSeconds | Out-Null
}

function Resolve-ToolPath {
    param(
        [string]$ToolName,
        [string]$ExplicitRoot
    )

    $candidates = New-Object System.Collections.Generic.List[string]
    if ($ExplicitRoot) {
        $candidates.Add((Join-Path $ExplicitRoot $ToolName))
        $candidates.Add((Join-Path $ExplicitRoot ("tools\tracy\{0}" -f $ToolName)))
    }
    if ($env:VCPKG_ROOT) {
        $candidates.Add((Join-Path $env:VCPKG_ROOT ("installed\x64-windows\tools\tracy\{0}" -f $ToolName)))
        $candidates.Add((Join-Path $env:VCPKG_ROOT ("installed\x64-windows\bin\{0}" -f $ToolName)))
    }

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    $command = Get-Command $ToolName -ErrorAction SilentlyContinue
    if ($command) {
        return $command.Source
    }

    return $null
}

function Assert-TracyTools {
    param(
        [string]$CapturePath,
        [string]$CsvExportPath
    )

    if (-not $CapturePath -or -not $CsvExportPath) {
        throw "Missing Tracy CLI tools. Install them with: vcpkg install tracy[cli-tools]:x64-windows, or pass -TracyToolRoot."
    }
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
    $environment = $null
    if (-not $NoTracy) {
        $environment = @{
            REEMBER_TRACY_WAIT_MS = [string]$TracyAttachWaitMilliseconds
            TRACY_PORT = [string]$script:ResolvedTracyPort
            TRACY_ONLY_LOCALHOST = "1"
            TRACY_ONLY_IPV4 = "1"
            TRACY_NO_BROADCAST = "1"
        }
    }
    $result = Invoke-External $ExePath (Get-ReEmberArguments $Workload $outPath $metricsPath) $stdoutPath $stderrPath $TimeoutSeconds $environment
    $metrics = Read-ReEmberTimingMetrics $metricsPath

    return [pscustomobject]@{
        FilePath = $result.FilePath
        Arguments = $result.Arguments
        ExitCode = $result.ExitCode
        ElapsedMs = $result.ElapsedMs
        StdoutPath = $result.StdoutPath
        StderrPath = $result.StderrPath
        MetricsPath = $metricsPath
        Metrics = $metrics
    }
}

function Start-TracyCaptureProcess {
    param(
        [string]$CapturePath,
        [string]$TracePath,
        [int]$CaptureSeconds,
        [string]$StdoutPath,
        [string]$StderrPath
    )

    Ensure-ParentDirectory $TracePath
    $arguments = @(
        "-a", "127.0.0.1",
        "-p", [string]$script:ResolvedTracyPort,
        "-o", $TracePath,
        "-f",
        "-s", [string]$CaptureSeconds
    )

    Write-Info ("EXEC: {0} {1}" -f $CapturePath, ($arguments -join " "))

    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $CapturePath
    $startInfo.WorkingDirectory = $RepoRoot
    $startInfo.UseShellExecute = $false
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    $startInfo.Arguments = Join-CommandLine $arguments

    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $startInfo
    [void]$process.Start()

    return [pscustomobject]@{
        Process = $process
        StdoutTask = $process.StandardOutput.ReadToEndAsync()
        StderrTask = $process.StandardError.ReadToEndAsync()
        StdoutPath = $StdoutPath
        StderrPath = $StderrPath
        TracePath = $TracePath
        CaptureSeconds = $CaptureSeconds
    }
}

function Write-TracyCaptureLogs {
    param([object]$Capture)

    $stdout = $Capture.StdoutTask.GetAwaiter().GetResult()
    $stderr = $Capture.StderrTask.GetAwaiter().GetResult()
    Set-Content -LiteralPath $Capture.StdoutPath -Value $stdout -Encoding UTF8
    Set-Content -LiteralPath $Capture.StderrPath -Value $stderr -Encoding UTF8
}

function Stop-TracyCaptureProcess {
    param([object]$Capture)

    if (-not $Capture) {
        return
    }

    $process = $Capture.Process
    $waitSeconds = [math]::Max($ReportTimeoutSeconds, $Capture.CaptureSeconds + 5)
    if (-not $process.WaitForExit($waitSeconds * 1000)) {
        try {
            $process.Kill($true)
        }
        catch {
            $process.Kill()
        }
        $process.WaitForExit()
        Write-TracyCaptureLogs $Capture
        throw ("Timed out waiting for tracy-capture to finish after {0}s. Check {1} and {2}." -f $waitSeconds, $Capture.StdoutPath, $Capture.StderrPath)
    }

    Write-TracyCaptureLogs $Capture

    if ($process.ExitCode -ne 0) {
        throw ("tracy-capture failed with exit code {0}. Check {1}." -f $process.ExitCode, $Capture.StderrPath)
    }

    if (-not (Test-Path -LiteralPath $Capture.TracePath)) {
        throw ("tracy-capture completed but did not create trace: {0}" -f $Capture.TracePath)
    }
}

function Invoke-TracyCapturedWorkload {
    param(
        [string]$ExePath,
        [object]$Workload,
        [string]$Tag,
        [string]$CapturePath
    )

    $tracePath = Join-Path $TraceRoot ("{0}_{1}.tracy" -f $Tag, $Workload.Name)
    $captureStdout = Join-Path $PerfRoot ("{0}_{1}.tracy-capture.stdout.txt" -f $Tag, $Workload.Name)
    $captureStderr = Join-Path $PerfRoot ("{0}_{1}.tracy-capture.stderr.txt" -f $Tag, $Workload.Name)
    $captureSeconds = [math]::Max(30, [math]::Ceiling(($TimeoutSeconds + ($TracyAttachWaitMilliseconds / 1000.0) + 10)))
    $capture = Start-TracyCaptureProcess $CapturePath $tracePath $captureSeconds $captureStdout $captureStderr
    Start-Sleep -Milliseconds 300

    try {
        $result = Invoke-ReEmberWorkload $ExePath $Workload $Tag
        Stop-TracyCaptureProcess $capture
        $result | Add-Member -NotePropertyName TracePath -NotePropertyValue $tracePath
        return $result
    }
    catch {
        if ($capture -and -not $capture.Process.HasExited) {
            try {
                $capture.Process.Kill($true)
            }
            catch {
                $capture.Process.Kill()
            }
        }
        throw
    }
}

function New-TimingRow {
    param(
        [object]$Workload,
        [int]$Iteration,
        [object]$Result
    )

    $metrics = $Result.Metrics
    return [pscustomobject]@{
        workload = $Workload.Name
        iteration = $Iteration
        elapsed_ms = $Result.ElapsedMs
        read_ms = [math]::Round((Get-Metric $metrics "read_ms"), 3)
        prepare_ms = [math]::Round((Get-Metric $metrics "prepare_ms"), 3)
        solve_ms = [math]::Round((Get-Metric $metrics "solve_ms"), 3)
        export_ms = [math]::Round((Get-Metric $metrics "export_ms"), 3)
        lhs_input_faces = [math]::Round((Get-Metric $metrics "lhs_input_faces"), 0)
        rhs_input_faces = [math]::Round((Get-Metric $metrics "rhs_input_faces"), 0)
        shared_scale = [math]::Round((Get-Metric $metrics "shared_scale"), 0)
        lhs_polygons = [math]::Round((Get-Metric $metrics "lhs_polygons"), 0)
        rhs_polygons = [math]::Round((Get-Metric $metrics "rhs_polygons"), 0)
        exported_faces = [math]::Round((Get-Metric $metrics "exported_faces"), 0)
        input_polygons = [math]::Round((Get-Metric $metrics "input_polygons"), 0)
        node_count = [math]::Round((Get-Metric $metrics "node_count"), 0)
        internal_node_count = [math]::Round((Get-Metric $metrics "internal_node_count"), 0)
        leaf_node_count = [math]::Round((Get-Metric $metrics "leaf_node_count"), 0)
        discarded_node_count = [math]::Round((Get-Metric $metrics "discarded_node_count"), 0)
        max_depth = [math]::Round((Get-Metric $metrics "max_depth"), 0)
        total_polygon_count = [math]::Round((Get-Metric $metrics "total_polygon_count"), 0)
        leaf_fragment_count = [math]::Round((Get-Metric $metrics "leaf_fragment_count"), 0)
        classified_fragment_count = [math]::Round((Get-Metric $metrics "classified_fragment_count"), 0)
        result_fragment_count = [math]::Round((Get-Metric $metrics "result_fragment_count"), 0)
        constant_discard_count = [math]::Round((Get-Metric $metrics "constant_discard_count"), 0)
        leaf_threshold_stop_count = [math]::Round((Get-Metric $metrics "leaf_threshold_stop_count"), 0)
        aabb_not_splittable_stop_count = [math]::Round((Get-Metric $metrics "aabb_not_splittable_stop_count"), 0)
        split_failure_stop_count = [math]::Round((Get-Metric $metrics "split_failure_stop_count"), 0)
        wntv_aware_split_count = [math]::Round((Get-Metric $metrics "wntv_aware_split_count"), 0)
        center_range_split_count = [math]::Round((Get-Metric $metrics "center_range_split_count"), 0)
        midpoint_split_count = [math]::Round((Get-Metric $metrics "midpoint_split_count"), 0)
        child_reference_reuse_count = [math]::Round((Get-Metric $metrics "child_reference_reuse_count"), 0)
        child_reference_trace_count = [math]::Round((Get-Metric $metrics "child_reference_trace_count"), 0)
        child_reference_candidate_count = [math]::Round((Get-Metric $metrics "child_reference_candidate_count"), 0)
        child_reference_candidate_tried_count = [math]::Round((Get-Metric $metrics "child_reference_candidate_tried_count"), 0)
        single_operand_leaf_bsp_skip_count = [math]::Round((Get-Metric $metrics "single_operand_leaf_bsp_skip_count"), 0)
        single_operand_classification_reuse_count = [math]::Round((Get-Metric $metrics "single_operand_classification_reuse_count"), 0)
        leaf_bsp_build_count = [math]::Round((Get-Metric $metrics "leaf_bsp_build_count"), 0)
        leaf_classification_point_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_point_candidate_count"), 0)
        leaf_classification_trace_attempt_count = [math]::Round((Get-Metric $metrics "leaf_classification_trace_attempt_count"), 0)
        leaf_classification_fast_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_fast_candidate_count"), 0)
        leaf_classification_fallback_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_fallback_candidate_count"), 0)
        leaf_classification_normal_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_normal_candidate_count"), 0)
        leaf_classification_interior_bridge_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_interior_bridge_candidate_count"), 0)
        exit_code = $Result.ExitCode
        lhs_faces = $Workload.LhsFaces
        rhs_faces = $Workload.RhsFaces
        op = $Workload.Op
        output = $(if (Test-Path -LiteralPath $Workload.Out) { $Workload.Out } else { $null })
        stdout = $Result.StdoutPath
        stderr = $Result.StderrPath
        metrics = $Result.MetricsPath
        tracy_trace = $(if ($Result.PSObject.Properties.Name -contains "TracePath") { $Result.TracePath } else { $null })
    }
}

function Export-TracyZones {
    param(
        [string]$CsvExportPath,
        [object[]]$TraceRecords
    )

    $zoneRows = New-Object System.Collections.Generic.List[object]
    foreach ($traceRecord in $TraceRecords) {
        $tracePath = $traceRecord.TracePath
        $rawCsvPath = Join-Path $PerfRoot ("{0}_{1}.tracy-zones.raw.csv" -f $traceRecord.Tag, $traceRecord.Workload)
        $stderrPath = Join-Path $PerfRoot ("{0}_{1}.tracy-csvexport.stderr.txt" -f $traceRecord.Tag, $traceRecord.Workload)
        Invoke-External $CsvExportPath @($tracePath) $rawCsvPath $stderrPath $ReportTimeoutSeconds | Out-Null

        foreach ($row in Import-Csv -LiteralPath $rawCsvPath) {
            $zoneRows.Add([pscustomobject]@{
                workload = $traceRecord.Workload
                iteration = $traceRecord.Iteration
                trace = $tracePath
                name = $row.name
                src_file = $row.src_file
                src_line = $row.src_line
                total_ns = [double]$row.total_ns
                total_perc = [double]$row.total_perc
                counts = [double]$row.counts
                mean_ns = [double]$row.mean_ns
                min_ns = [double]$row.min_ns
                max_ns = [double]$row.max_ns
                std_ns = [double]$row.std_ns
            })
        }
    }

    $zonesCsvPath = Join-Path $PerfRoot "tracy_zones.csv"
    $zoneRows | Export-Csv -LiteralPath $zonesCsvPath -NoTypeInformation -Encoding UTF8
    return [pscustomobject]@{
        Path = $zonesCsvPath
        Rows = $zoneRows.ToArray()
    }
}

function Get-PropertyDouble {
    param(
        [object]$Row,
        [string]$Property
    )

    return [double]$Row.PSObject.Properties[$Property].Value
}

function Measure-AverageProperty {
    param(
        [object[]]$Rows,
        [string]$Property
    )

    if ($Rows.Count -eq 0) {
        return 0
    }

    return [math]::Round((($Rows | ForEach-Object { Get-PropertyDouble $_ $Property }) | Measure-Object -Average).Average, 3)
}

function Measure-SumProperty {
    param(
        [object[]]$Rows,
        [string]$Property
    )

    if ($Rows.Count -eq 0) {
        return 0
    }

    return [math]::Round((($Rows | ForEach-Object { Get-PropertyDouble $_ $Property }) | Measure-Object -Sum).Sum, 3)
}

function Get-ZoneAggregates {
    param([object[]]$ZoneRows)

    $aggregates = New-Object System.Collections.Generic.List[object]
    foreach ($group in ($ZoneRows | Group-Object name)) {
        $count = Measure-SumProperty $group.Group "counts"
        $totalNs = Measure-SumProperty $group.Group "total_ns"
        $maxNs = (($group.Group | ForEach-Object { Get-PropertyDouble $_ "max_ns" }) | Measure-Object -Maximum).Maximum
        $meanNs = $(if ($count -gt 0) { $totalNs / $count } else { 0 })
        $aggregates.Add([pscustomobject]@{
            name = $group.Name
            counts = [math]::Round($count, 0)
            total_ms = [math]::Round($totalNs / 1000000.0, 3)
            mean_us = [math]::Round($meanNs / 1000.0, 3)
            max_ms = [math]::Round($maxNs / 1000000.0, 3)
        })
    }

    return @($aggregates | Sort-Object total_ms -Descending)
}

function Find-ZoneAggregate {
    param(
        [object[]]$Aggregates,
        [string]$Name
    )

    return $Aggregates | Where-Object { $_.name -eq $Name } | Select-Object -First 1
}

function Add-MarkdownTable {
    param(
        [System.Collections.Generic.List[string]]$Lines,
        [string[]]$Headers,
        [object[]]$Rows
    )

    $Lines.Add(("| {0} |" -f ($Headers -join " | ")))
    $Lines.Add(("| {0} |" -f (($Headers | ForEach-Object { "---" }) -join " | ")))
    foreach ($row in $Rows) {
        $values = foreach ($header in $Headers) {
            [string]$row.PSObject.Properties[$header].Value
        }
        $Lines.Add(("| {0} |" -f ($values -join " | ")))
    }
}

function Write-Reports {
    param(
        [object[]]$TimingRows,
        [object[]]$ZoneRows,
        [string]$TimingCsvPath,
        [string]$ZonesCsvPath
    )

    $zoneAggregates = @(Get-ZoneAggregates $ZoneRows)
    $reportLines = New-Object System.Collections.Generic.List[string]
    $reportLines.Add("# re-EMBER Tracy Performance Report")
    $reportLines.Add("")
    $reportLines.Add(("run={0}" -f $RunStamp))
    $reportLines.Add(("configuration={0}" -f $Configuration))
    $reportLines.Add(("iterations={0}" -f $Iterations))
    $reportLines.Add(("timings_csv={0}" -f $TimingCsvPath))
    if ($ZonesCsvPath) {
        $reportLines.Add(("tracy_zones_csv={0}" -f $ZonesCsvPath))
    }
    $reportLines.Add("")

    $workloadRows = New-Object System.Collections.Generic.List[object]
    foreach ($group in ($TimingRows | Group-Object workload)) {
        $rows = @($group.Group)
        $workloadRows.Add([pscustomobject]@{
            workload = $group.Name
            avg_elapsed_ms = Measure-AverageProperty $rows "elapsed_ms"
            avg_solve_ms = Measure-AverageProperty $rows "solve_ms"
            avg_nodes = Measure-AverageProperty $rows "node_count"
            avg_leaves = Measure-AverageProperty $rows "leaf_node_count"
            avg_leaf_traces = Measure-AverageProperty $rows "leaf_classification_trace_attempt_count"
            avg_child_ref_tried = Measure-AverageProperty $rows "child_reference_candidate_tried_count"
            avg_results = Measure-AverageProperty $rows "result_fragment_count"
        })
    }

    $reportLines.Add("## Workload Scale")
    Add-MarkdownTable -Lines $reportLines -Headers @(
        "workload",
        "avg_elapsed_ms",
        "avg_solve_ms",
        "avg_nodes",
        "avg_leaves",
        "avg_leaf_traces",
        "avg_child_ref_tried",
        "avg_results"
    ) -Rows ($workloadRows.ToArray())
    $reportLines.Add("")

    if ($ZoneRows.Count -gt 0) {
        $keyZoneNames = @(
            "re-EMBER::read_obj",
            "re-EMBER::prepare_polygons",
            "re-EMBER::prepare_problem",
            "re-EMBER::solve_bool_problem",
            "BoolProblem::solve",
            "SubdivisionSolver::solve",
            "SubdivisionSolver::solveRecursive",
            "SubdivisionSolver::createChildrenFromSplit",
            "SubdivisionSolver::makeChildReference",
            "SubdivisionSolver::solveLeafArrangement",
            "SubdivisionSolver::classifyLeafFragmentsAndCollectResults",
            "LeafClassification::traceCandidate",
            "tracePathWNVAllowSubdivisionClipCrossingTrusted",
            "tracePathWNVToSurfacePointTrusted",
            "BSPTree::insertTrusted",
            "BSPTree::addSegmentRecursive",
            "enumerateAABBPathCandidates",
            "enumerateLeafClassificationFastPathCandidatesFromPoints",
            "enumerateLeafClassificationFallbackPathCandidatesFromPoints",
            "enumerateLeafClassificationNormalApproachCandidatesFromPoints",
            "enumerateLeafClassificationInteriorBridgeCandidatesFromPoints"
        )

        $keyRows = New-Object System.Collections.Generic.List[object]
        foreach ($zoneName in $keyZoneNames) {
            $zone = Find-ZoneAggregate $zoneAggregates $zoneName
            if ($zone) {
                $keyRows.Add($zone)
            }
        }

        $reportLines.Add("## Key Zone Counts")
        Add-MarkdownTable -Lines $reportLines -Headers @("name", "counts", "total_ms", "mean_us", "max_ms") -Rows ($keyRows.ToArray())
        $reportLines.Add("")

        $reportLines.Add("## Top Hot Zones")
        Add-MarkdownTable -Lines $reportLines -Headers @("name", "counts", "total_ms", "mean_us", "max_ms") -Rows @($zoneAggregates | Select-Object -First 20)
        $reportLines.Add("")

        $leafTraceMetric = Measure-SumProperty $TimingRows "leaf_classification_trace_attempt_count"
        $leafTraceZone = Find-ZoneAggregate $zoneAggregates "LeafClassification::traceCandidate"
        $childTraceMetric = Measure-SumProperty $TimingRows "child_reference_candidate_tried_count"
        $childTraceZone = Find-ZoneAggregate $zoneAggregates "tracePathWNVAllowSubdivisionClipCrossingTrusted"
        $reportLines.Add("## Cross Checks")
        $reportLines.Add(("- leaf trace attempts: metrics={0}, zone={1}" -f $leafTraceMetric, $(if ($leafTraceZone) { $leafTraceZone.counts } else { 0 })))
        $reportLines.Add(("- child reference trace attempts: metrics={0}, zone={1}" -f $childTraceMetric, $(if ($childTraceZone) { $childTraceZone.counts } else { 0 })))
    }
    else {
        $reportLines.Add("## Tracy Zones")
        $reportLines.Add("No Tracy zone data was captured because -NoTracy was used.")
    }

    $reportPath = Join-Path $PerfRoot "report.md"
    Set-Content -LiteralPath $reportPath -Value $reportLines -Encoding UTF8

    $summaryLines = New-Object System.Collections.Generic.List[string]
    $summaryLines.Add(("executable={0}" -f $script:ExePath))
    $summaryLines.Add(("configuration={0}" -f $Configuration))
    $summaryLines.Add(("iterations={0}" -f $Iterations))
    $summaryLines.Add(("timings_csv={0}" -f $TimingCsvPath))
    if ($ZonesCsvPath) {
        $summaryLines.Add(("tracy_zones_csv={0}" -f $ZonesCsvPath))
    }
    $summaryLines.Add(("report={0}" -f $reportPath))
    foreach ($row in $workloadRows) {
        $summaryLines.Add(("workload={0}" -f $row.workload))
        $summaryLines.Add(("  avg_elapsed_ms={0}" -f $row.avg_elapsed_ms))
        $summaryLines.Add(("  avg_solve_ms={0}" -f $row.avg_solve_ms))
        $summaryLines.Add(("  avg_nodes={0}" -f $row.avg_nodes))
        $summaryLines.Add(("  avg_leaf_traces={0}" -f $row.avg_leaf_traces))
    }
    Set-Content -LiteralPath (Join-Path $PerfRoot "summary.txt") -Value $summaryLines -Encoding UTF8

    return $reportPath
}

try {
    Start-Transcript -Path $TranscriptPath -Force | Out-Null
    $TranscriptStarted = $true

    if (-not (Test-Path -LiteralPath $BuildRoot)) {
        New-Item -ItemType Directory -Force -Path $BuildRoot | Out-Null
    }

    if (-not $SkipBuild) {
        Invoke-CMakeConfigure
    }

    if ((-not $NoTracy) -and (-not (Test-TracyEnabledInBuild $BuildRoot))) {
        throw "The build tree is not configured with REEMBER_ENABLE_TRACY=ON. Re-run without -SkipBuild or configure build\ manually."
    }

    $script:ResolvedTracyPort = $null
    if (-not $NoTracy) {
        $script:ResolvedTracyPort = Resolve-TracyPort -PreferredPort $TracyPort -WasExplicit $TracyPortWasExplicit
        Write-Info ("Using Tracy port {0}" -f $script:ResolvedTracyPort)
    }

    if ((-not $SkipBuild) -or $ForceBuild) {
        Invoke-CMakeBuild
    }

    $script:ExePath = Resolve-ReEmberExecutablePath $BuildRoot $Configuration
    if (-not $script:ExePath) {
        $expectedLocations = Get-ReEmberExecutableCandidates $BuildRoot $Configuration
        throw ("Missing executable after build. Checked: {0}" -f ($expectedLocations -join ", "))
    }

    $tracyCapturePath = $null
    $tracyCsvExportPath = $null
    if (-not $NoTracy) {
        $tracyCapturePath = Resolve-ToolPath "tracy-capture.exe" $TracyToolRoot
        $tracyCsvExportPath = Resolve-ToolPath "tracy-csvexport.exe" $TracyToolRoot
        Assert-TracyTools $tracyCapturePath $tracyCsvExportPath
    }

    $workloads = @(Get-Workloads)
    $manifest = [pscustomobject]@{
        repoRoot = [string]$RepoRoot
        buildDir = [string]$BuildRoot
        outputDir = [string]$PerfRoot
        timestamp = $RunStamp
        configuration = $Configuration
        executable = $script:ExePath
        leafThreshold = $LeafThreshold
        iterations = $Iterations
        timeoutSeconds = $TimeoutSeconds
        tracyEnabled = (-not $NoTracy)
        tracyPort = $script:ResolvedTracyPort
        tracyCapture = $tracyCapturePath
        tracyCsvExport = $tracyCsvExportPath
        inputAssumptionsEnabled = (-not $NoInputAssumptions)
        workloads = $workloads
    }
    $manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $PerfRoot "manifest.json") -Encoding UTF8

    $timingRows = New-Object System.Collections.Generic.List[object]
    $traceRecords = New-Object System.Collections.Generic.List[object]
    foreach ($workload in $workloads) {
        Write-Info ("Profiling workload {0}: lhs_faces={1} rhs_faces={2} op={3}" -f $workload.Name, $workload.LhsFaces, $workload.RhsFaces, $workload.Op)
        for ($i = 1; $i -le $Iterations; ++$i) {
            $tag = "timing_{0}" -f $i
            if ($NoTracy) {
                $result = Invoke-ReEmberWorkload $script:ExePath $workload $tag
            }
            else {
                $result = Invoke-TracyCapturedWorkload $script:ExePath $workload $tag $tracyCapturePath
                $traceRecords.Add([pscustomobject]@{
                    Workload = $workload.Name
                    Iteration = $i
                    Tag = $tag
                    TracePath = $result.TracePath
                })
            }

            $timingRows.Add((New-TimingRow $workload $i $result))
        }
    }

    $timingCsvPath = Join-Path $PerfRoot "timings.csv"
    $timingRows | Export-Csv -LiteralPath $timingCsvPath -NoTypeInformation -Encoding UTF8

    $zoneRows = @()
    $zonesCsvPath = $null
    if (-not $NoTracy) {
        $zoneResult = Export-TracyZones -CsvExportPath $tracyCsvExportPath -TraceRecords ($traceRecords.ToArray())
        $zonesCsvPath = $zoneResult.Path
        $zoneRows = @($zoneResult.Rows)
    }

    $reportPath = Write-Reports -TimingRows ($timingRows.ToArray()) -ZoneRows @($zoneRows) -TimingCsvPath $timingCsvPath -ZonesCsvPath $zonesCsvPath
    Write-Info ("Done. See report: {0}" -f $reportPath)
}
finally {
    if ($TranscriptStarted) {
        Stop-Transcript | Out-Null
    }
}
