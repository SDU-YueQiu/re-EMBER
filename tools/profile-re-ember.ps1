# Capture re-EMBER CLI workloads with Tracy and write reports under build\perf.
param(
    [string]$Lhs,
    [string]$Rhs,
    [ValidateSet("union", "intersection", "difference")][string]$Op = "difference",
    [string]$Out,
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")][string]$Configuration = "RelWithDebInfo",
    [int]$LeafThreshold = 25,
    [int]$Iterations = 5,
    [int]$TimeoutSeconds = 300,
    [int]$BuildTimeoutSeconds = 900,
    [int]$ReportTimeoutSeconds = 120,
    [int]$TracyPort = 8086,
    [int]$TracyAttachWaitMilliseconds = 1500,
    [string]$TracyToolRoot,
    [string[]]$UnwrapZoneFilter = @(),
    [switch]$EnableMathTracy,
    [switch]$SkipBuild,
    [switch]$ForceBuild,
    [switch]$NoTracy,
    [switch]$NoInputAssumptions,
    [switch]$Help
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"
$TracyPortWasExplicit = $PSBoundParameters.ContainsKey("TracyPort")
$UnwrapZoneFilter = @(
    foreach ($value in $UnwrapZoneFilter) {
        foreach ($token in ([string]$value -split '[,;]')) {
            $trimmed = $token.Trim()
            if (-not [string]::IsNullOrWhiteSpace($trimmed)) {
                $trimmed
            }
        }
    }
)

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
  build\perf\run_<timestamp>\tracy_zones_self.csv
  build\perf\run_<timestamp>\tracy_unwrap\*.csv
  build\perf\run_<timestamp>\report.md
  build\perf\run_<timestamp>\summary.txt

Notes:
  - Uses build\ only; profiling builds live under build\profile_* and do not reuse the ordinary build\ tree.
  - Tracy profiling is the default path and uses build\profile_tracy\ with REEMBER_ENABLE_TRACY=ON.
  - When -SkipBuild is not specified, the script configures the mode-matched profiling build tree automatically.
  - Use -EnableMathTracy to additionally enable low-level math256 Tracy zones; the default is OFF to avoid steady-state overhead.
  - -EnableMathTracy uses build\profile_tracy_math\ and enables REEMBER_ENABLE_TRACY_MATH automatically.
  - Install the required tools with: vcpkg install tracy[cli-tools]:x64-windows
  - The script auto-selects a bindable Tracy port when the default port is reserved on the local machine.
  - The script sets REEMBER_TRACY_WAIT_MS before timed work so tracy-capture can attach without polluting read/prepare/solve/export timings.
  - tracy_zones.csv keeps inclusive time; tracy_zones_self.csv uses tracy-csvexport -e for self time.
  - Use -UnwrapZoneFilter <zone-name> to export per-event CSVs for specific hotspots only.
  - Use -NoTracy only for timing-only runs without zone capture; that mode uses build\profile_notracy\ automatically.
  - OBJ workloads are assumed to satisfy NSI/NNC input assumptions by default.
  - Use -NoInputAssumptions only when profiling intentionally invalid or adversarial OBJ inputs.
"@
}

if ($Help) {
    Show-Help
    exit 0
}

if ($EnableMathTracy -and $NoTracy) {
    throw "-EnableMathTracy requires Tracy capture. Remove -NoTracy or omit -EnableMathTracy."
}

$RepoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
Set-Location -LiteralPath $RepoRoot

function Get-ProfilingBuildFlavor {
    if ($NoTracy) {
        return "notracy"
    }

    if ($EnableMathTracy) {
        return "tracy_math"
    }

    return "tracy"
}

$BuildFlavor = Get-ProfilingBuildFlavor
$BuildRoot = Join-Path $RepoRoot ("build\profile_{0}" -f $BuildFlavor)
$RunStamp = Get-Date -Format "yyyyMMdd_HHmmss"
$PerfRoot = Join-Path $RepoRoot ("build\perf\run_{0}" -f $RunStamp)
$TraceRoot = Join-Path $PerfRoot "tracy_traces"
$UnwrapRoot = Join-Path $PerfRoot "tracy_unwrap"
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

function Get-ObjBounds {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) {
        throw ("Cannot read bounds from missing OBJ: {0}" -f $Path)
    }

    $vertexPattern = '^\s*v\s+(?<x>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s+' +
                     '(?<y>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s+' +
                     '(?<z>[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)'
    $minX = [double]::PositiveInfinity
    $minY = [double]::PositiveInfinity
    $minZ = [double]::PositiveInfinity
    $maxX = [double]::NegativeInfinity
    $maxY = [double]::NegativeInfinity
    $maxZ = [double]::NegativeInfinity
    $vertexCount = 0

    foreach ($line in Get-Content -LiteralPath $Path) {
        $match = [System.Text.RegularExpressions.Regex]::Match($line, $vertexPattern)
        if (-not $match.Success) {
            continue
        }

        $x = [double]::Parse($match.Groups["x"].Value, [System.Globalization.CultureInfo]::InvariantCulture)
        $y = [double]::Parse($match.Groups["y"].Value, [System.Globalization.CultureInfo]::InvariantCulture)
        $z = [double]::Parse($match.Groups["z"].Value, [System.Globalization.CultureInfo]::InvariantCulture)
        $minX = [Math]::Min($minX, $x)
        $minY = [Math]::Min($minY, $y)
        $minZ = [Math]::Min($minZ, $z)
        $maxX = [Math]::Max($maxX, $x)
        $maxY = [Math]::Max($maxY, $y)
        $maxZ = [Math]::Max($maxZ, $z)
        ++$vertexCount
    }

    if ($vertexCount -eq 0) {
        throw ("OBJ contains no vertices: {0}" -f $Path)
    }

    return [pscustomobject]@{
        MinX = $minX
        MinY = $minY
        MinZ = $minZ
        MaxX = $maxX
        MaxY = $maxY
        MaxZ = $maxZ
        SizeX = $maxX - $minX
        SizeY = $maxY - $minY
        SizeZ = $maxZ - $minZ
        CenterX = ($minX + $maxX) / 2.0
        CenterY = ($minY + $maxY) / 2.0
        CenterZ = ($minZ + $maxZ) / 2.0
    }
}

function Get-VisualTranslationRange {
    param(
        [object]$WorkpieceBounds,
        [object]$ToolBounds
    )

    $dominantExtent = (
        @(
            [double]$WorkpieceBounds.SizeX,
            [double]$WorkpieceBounds.SizeY,
            [double]$WorkpieceBounds.SizeZ,
            [double]$ToolBounds.SizeX,
            [double]$ToolBounds.SizeY,
            [double]$ToolBounds.SizeZ,
            1.0
        ) | Measure-Object -Maximum
    ).Maximum

    return 2.0 * [double]$dominantExtent
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

function Write-CenterAlignedObjCopy {
    param(
        [string]$SourcePath,
        [string]$ReferencePath,
        [string]$DestinationPath,
        [double]$OffsetX,
        [double]$OffsetY,
        [double]$OffsetZ
    )

    $sourceBounds = Get-ObjBounds $SourcePath
    $referenceBounds = Get-ObjBounds $ReferencePath

    return Write-TranslatedObjCopy `
        $SourcePath `
        $DestinationPath `
        ($referenceBounds.CenterX - $sourceBounds.CenterX + $OffsetX) `
        ($referenceBounds.CenterY - $sourceBounds.CenterY + $OffsetY) `
        ($referenceBounds.CenterZ - $sourceBounds.CenterZ + $OffsetZ)
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

function Test-TracyMathEnabledInBuild {
    param([string]$BuildDir)

    $cachePath = Join-Path $BuildDir "CMakeCache.txt"
    $value = Get-CMakeCacheValue $cachePath "REEMBER_ENABLE_TRACY_MATH"
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
        if ($EnableMathTracy) {
            $args += "-DREEMBER_ENABLE_TRACY_MATH=ON"
        }
        else {
            $args += "-DREEMBER_ENABLE_TRACY_MATH=OFF"
        }
    }
    else {
        $args += "-DREEMBER_ENABLE_TRACY=OFF"
        $args += "-DREEMBER_ENABLE_TRACY_MATH=OFF"
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

    $visualWorkpieceSource = "assets\visual_test\workpiece_block.obj"
    $visualToolSource = "assets\visual_test\tool_box.obj"
    $visualToolOverlapPosePath = Join-Path $PerfRoot "visual_test_overlap_pose_tool.obj"
    $visualToolUiDefaultPosePath = Join-Path $PerfRoot "visual_test_ui_default_pose_tool.obj"
    if ((Test-Path -LiteralPath $visualWorkpieceSource) -and (Test-Path -LiteralPath $visualToolSource)) {
        $visualWorkpieceBounds = Get-ObjBounds $visualWorkpieceSource
        $visualToolBounds = Get-ObjBounds $visualToolSource
        $visualTranslationRange = Get-VisualTranslationRange $visualWorkpieceBounds $visualToolBounds
        $overlapOffsetY = -0.25 * $visualTranslationRange

        [void](Write-CenterAlignedObjCopy $visualToolSource $visualWorkpieceSource $visualToolOverlapPosePath 0.0 $overlapOffsetY 0.0)
        [void](Write-CenterAlignedObjCopy $visualToolSource $visualWorkpieceSource $visualToolUiDefaultPosePath 0.0 0.0 0.0)
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
        invalid_or_empty_discard_count = [math]::Round((Get-Metric $metrics "invalid_or_empty_discard_count"), 0)
        child_constant_discard_count = [math]::Round((Get-Metric $metrics "child_constant_discard_count"), 0)
        leaf_threshold_stop_count = [math]::Round((Get-Metric $metrics "leaf_threshold_stop_count"), 0)
        aabb_not_splittable_stop_count = [math]::Round((Get-Metric $metrics "aabb_not_splittable_stop_count"), 0)
        split_failure_stop_count = [math]::Round((Get-Metric $metrics "split_failure_stop_count"), 0)
        wntv_aware_split_count = [math]::Round((Get-Metric $metrics "wntv_aware_split_count"), 0)
        center_range_split_count = [math]::Round((Get-Metric $metrics "center_range_split_count"), 0)
        midpoint_split_count = [math]::Round((Get-Metric $metrics "midpoint_split_count"), 0)
        child_reference_reuse_count = [math]::Round((Get-Metric $metrics "child_reference_reuse_count"), 0)
        child_reference_trace_count = [math]::Round((Get-Metric $metrics "child_reference_trace_count"), 0)
        child_reference_candidate_count = [math]::Round((Get-Metric $metrics "child_reference_candidate_count"), 0)
        child_reference_fast_candidate_count = [math]::Round((Get-Metric $metrics "child_reference_fast_candidate_count"), 0)
        child_reference_exhaustive_candidate_count = [math]::Round((Get-Metric $metrics "child_reference_exhaustive_candidate_count"), 0)
        child_reference_candidate_tried_count = [math]::Round((Get-Metric $metrics "child_reference_candidate_tried_count"), 0)
        child_reference_fast_candidate_tried_count = [math]::Round((Get-Metric $metrics "child_reference_fast_candidate_tried_count"), 0)
        child_reference_exhaustive_candidate_tried_count = [math]::Round((Get-Metric $metrics "child_reference_exhaustive_candidate_tried_count"), 0)
        single_operand_assumption_stop_count = [math]::Round((Get-Metric $metrics "single_operand_assumption_stop_count"), 0)
        single_operand_assumption_fallback_count = [math]::Round((Get-Metric $metrics "single_operand_assumption_fallback_count"), 0)
        single_operand_leaf_bsp_skip_count = [math]::Round((Get-Metric $metrics "single_operand_leaf_bsp_skip_count"), 0)
        single_operand_classification_reuse_count = [math]::Round((Get-Metric $metrics "single_operand_classification_reuse_count"), 0)
        leaf_bsp_build_count = [math]::Round((Get-Metric $metrics "leaf_bsp_build_count"), 0)
        leaf_classification_point_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_point_candidate_count"), 0)
        leaf_classification_primary_point_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_primary_point_candidate_count"), 0)
        leaf_classification_expanded_point_candidate_count = [math]::Round((Get-Metric $metrics "leaf_classification_expanded_point_candidate_count"), 0)
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

function Convert-ToSafePathToken {
    param([string]$Value)

    if ([string]::IsNullOrWhiteSpace($Value)) {
        return "zone"
    }

    $safe = $Value -replace '[^A-Za-z0-9._-]+', '_'
    $safe = $safe.Trim('_')
    if ([string]::IsNullOrWhiteSpace($safe)) {
        return "zone"
    }

    return $safe
}

function Export-TracyZoneTable {
    param(
        [string]$CsvExportPath,
        [object[]]$TraceRecords,
        [string[]]$ExtraArguments,
        [string]$RawSuffix,
        [string]$OutputCsvName
    )

    $zoneRows = New-Object System.Collections.Generic.List[object]
    foreach ($traceRecord in $TraceRecords) {
        $tracePath = $traceRecord.TracePath
        $rawCsvPath = Join-Path $PerfRoot ("{0}_{1}.{2}.raw.csv" -f $traceRecord.Tag, $traceRecord.Workload, $RawSuffix)
        $stderrPath = Join-Path $PerfRoot ("{0}_{1}.{2}.stderr.txt" -f $traceRecord.Tag, $traceRecord.Workload, $RawSuffix)
        $arguments = @()
        if ($ExtraArguments) {
            $arguments += $ExtraArguments
        }
        $arguments += @($tracePath)
        Invoke-External $CsvExportPath $arguments $rawCsvPath $stderrPath $ReportTimeoutSeconds | Out-Null

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

    $zonesCsvPath = Join-Path $PerfRoot $OutputCsvName
    $zoneRows | Export-Csv -LiteralPath $zonesCsvPath -NoTypeInformation -Encoding UTF8
    return [pscustomobject]@{
        Path = $zonesCsvPath
        Rows = $zoneRows.ToArray()
    }
}

function Export-TracyZones {
    param(
        [string]$CsvExportPath,
        [object[]]$TraceRecords
    )

    return Export-TracyZoneTable -CsvExportPath $CsvExportPath -TraceRecords $TraceRecords -ExtraArguments @() -RawSuffix "tracy-zones" -OutputCsvName "tracy_zones.csv"
}

function Export-TracySelfZones {
    param(
        [string]$CsvExportPath,
        [object[]]$TraceRecords
    )

    return Export-TracyZoneTable -CsvExportPath $CsvExportPath -TraceRecords $TraceRecords -ExtraArguments @("-e") -RawSuffix "tracy-zones-self" -OutputCsvName "tracy_zones_self.csv"
}

function Export-TracyUnwrapRows {
    param(
        [string]$CsvExportPath,
        [object[]]$TraceRecords,
        [string[]]$ZoneFilters
    )

    $unwrapExports = New-Object System.Collections.Generic.List[object]
    if (-not $ZoneFilters -or $ZoneFilters.Count -eq 0) {
        return $unwrapExports.ToArray()
    }

    New-Item -ItemType Directory -Force -Path $UnwrapRoot | Out-Null
    foreach ($zoneFilter in $ZoneFilters) {
        $safeFilter = Convert-ToSafePathToken $zoneFilter
        foreach ($traceRecord in $TraceRecords) {
            $tracePath = $traceRecord.TracePath
            $rawCsvPath = Join-Path $UnwrapRoot ("{0}_{1}.{2}.unwrap.csv" -f $traceRecord.Tag, $traceRecord.Workload, $safeFilter)
            $stderrPath = Join-Path $PerfRoot ("{0}_{1}.{2}.tracy-unwrap.stderr.txt" -f $traceRecord.Tag, $traceRecord.Workload, $safeFilter)
            Invoke-External $CsvExportPath @("-u", "-f", $zoneFilter, $tracePath) $rawCsvPath $stderrPath $ReportTimeoutSeconds | Out-Null

            $eventCount = 0
            if (Test-Path -LiteralPath $rawCsvPath) {
                $rows = @(Import-Csv -LiteralPath $rawCsvPath)
                $eventCount = $rows.Count
            }

            $unwrapExports.Add([pscustomobject]@{
                workload = $traceRecord.Workload
                iteration = $traceRecord.Iteration
                zone_filter = $zoneFilter
                path = $rawCsvPath
                trace = $tracePath
                event_count = $eventCount
            })
        }
    }

    return $unwrapExports.ToArray()
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

function Measure-SumProperties {
    param(
        [object[]]$Rows,
        [string[]]$Properties
    )

    $sum = 0
    foreach ($property in $Properties) {
        $sum += (Measure-SumProperty $Rows $property)
    }

    return [math]::Round($sum, 3)
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

function Get-ZoneAggregateRollup {
    param(
        [object[]]$Aggregates,
        [string[]]$Names
    )

    $matched = @()
    foreach ($name in $Names) {
        $match = Find-ZoneAggregate $Aggregates $name
        if ($match) {
            $matched += $match
        }
    }

    $count = 0
    $totalMs = 0
    $maxMs = 0
    foreach ($row in $matched) {
        $count += [double]$row.counts
        $totalMs += [double]$row.total_ms
        if ([double]$row.max_ms -gt $maxMs) {
            $maxMs = [double]$row.max_ms
        }
    }

    return [pscustomobject]@{
        zone_name = ($Names -join " + ")
        zone_count = [math]::Round($count, 0)
        zone_total_ms = [math]::Round($totalMs, 3)
        zone_max_ms = [math]::Round($maxMs, 3)
    }
}

function Get-WorkloadZoneAggregates {
    param([object[]]$ZoneRows)

    $aggregates = New-Object System.Collections.Generic.List[object]
    foreach ($workloadGroup in ($ZoneRows | Group-Object workload)) {
        foreach ($zoneGroup in ($workloadGroup.Group | Group-Object name)) {
            $count = Measure-SumProperty $zoneGroup.Group "counts"
            $totalNs = Measure-SumProperty $zoneGroup.Group "total_ns"
            $maxNs = (($zoneGroup.Group | ForEach-Object { Get-PropertyDouble $_ "max_ns" }) | Measure-Object -Maximum).Maximum
            $meanNs = $(if ($count -gt 0) { $totalNs / $count } else { 0 })
            $aggregates.Add([pscustomobject]@{
                workload = $workloadGroup.Name
                name = $zoneGroup.Name
                counts = [math]::Round($count, 0)
                total_ms = [math]::Round($totalNs / 1000000.0, 3)
                mean_us = [math]::Round($meanNs / 1000.0, 3)
                max_ms = [math]::Round($maxNs / 1000000.0, 3)
            })
        }
    }

    return @($aggregates | Sort-Object workload, @{ Expression = "total_ms"; Descending = $true })
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

function New-StrategySummaryRow {
    param(
        [object[]]$TimingRows,
        [object[]]$InclusiveAggregates,
        [object[]]$SelfAggregates,
        [string]$Item,
        [string]$MetricProperty,
        [string[]]$ZoneNames
    )

    $metricCount = [math]::Round((Measure-SumProperty $TimingRows $MetricProperty), 0)
    $inclusiveRollup = Get-ZoneAggregateRollup $InclusiveAggregates $ZoneNames
    $selfRollup = Get-ZoneAggregateRollup $SelfAggregates $ZoneNames
    return [pscustomobject]@{
        item = $Item
        metric_count = $metricCount
        zone_count = $inclusiveRollup.zone_count
        status = $(if ($metricCount -eq $inclusiveRollup.zone_count) { "ok" } else { "mismatch" })
        zone_name = $inclusiveRollup.zone_name
        inclusive_ms = $inclusiveRollup.zone_total_ms
        self_ms = $selfRollup.zone_total_ms
    }
}

function New-StrategySummaryRowFromMetricValue {
    param(
        [double]$MetricCount,
        [object[]]$InclusiveAggregates,
        [object[]]$SelfAggregates,
        [string]$Item,
        [string[]]$ZoneNames
    )

    $roundedMetricCount = [math]::Round($MetricCount, 0)
    $inclusiveRollup = Get-ZoneAggregateRollup $InclusiveAggregates $ZoneNames
    $selfRollup = Get-ZoneAggregateRollup $SelfAggregates $ZoneNames
    return [pscustomobject]@{
        item = $Item
        metric_count = $roundedMetricCount
        zone_count = $inclusiveRollup.zone_count
        status = $(if ($roundedMetricCount -eq $inclusiveRollup.zone_count) { "ok" } else { "mismatch" })
        zone_name = $inclusiveRollup.zone_name
        inclusive_ms = $inclusiveRollup.zone_total_ms
        self_ms = $selfRollup.zone_total_ms
    }
}

function Write-Reports {
    param(
        [object[]]$TimingRows,
        [object[]]$InclusiveZoneRows,
        [object[]]$SelfZoneRows,
        [object[]]$UnwrapExports,
        [string]$TimingCsvPath,
        [string]$InclusiveZonesCsvPath,
        [string]$SelfZonesCsvPath
    )

    $inclusiveAggregates = @(Get-ZoneAggregates $InclusiveZoneRows)
    $selfAggregates = @(Get-ZoneAggregates $SelfZoneRows)
    $selfWorkloadAggregates = @(Get-WorkloadZoneAggregates $SelfZoneRows)
    $reportLines = New-Object System.Collections.Generic.List[string]
    $reportLines.Add("# re-EMBER Tracy Performance Report")
    $reportLines.Add("")
    $reportLines.Add(("run={0}" -f $RunStamp))
    $reportLines.Add(("configuration={0}" -f $Configuration))
    $reportLines.Add(("iterations={0}" -f $Iterations))
    $reportLines.Add(("timings_csv={0}" -f $TimingCsvPath))
    if ($InclusiveZonesCsvPath) {
        $reportLines.Add(("tracy_zones_csv={0}" -f $InclusiveZonesCsvPath))
    }
    if ($SelfZonesCsvPath) {
        $reportLines.Add(("tracy_zones_self_csv={0}" -f $SelfZonesCsvPath))
    }
    if ($UnwrapExports.Count -gt 0) {
        $reportLines.Add(("tracy_unwrap_root={0}" -f $UnwrapRoot))
    }
    $reportLines.Add("")

    $workloadRows = New-Object System.Collections.Generic.List[object]
    foreach ($group in ($TimingRows | Group-Object workload)) {
        $rows = @($group.Group)
        $workloadRows.Add([pscustomobject]@{
            workload = $group.Name
            avg_read_ms = Measure-AverageProperty $rows "read_ms"
            avg_prepare_ms = Measure-AverageProperty $rows "prepare_ms"
            avg_elapsed_ms = Measure-AverageProperty $rows "elapsed_ms"
            avg_solve_ms = Measure-AverageProperty $rows "solve_ms"
            avg_export_ms = Measure-AverageProperty $rows "export_ms"
            avg_nodes = Measure-AverageProperty $rows "node_count"
            avg_leaves = Measure-AverageProperty $rows "leaf_node_count"
            avg_leaf_points = Measure-AverageProperty $rows "leaf_classification_point_candidate_count"
            avg_leaf_traces = Measure-AverageProperty $rows "leaf_classification_trace_attempt_count"
            avg_child_ref_candidates = Measure-AverageProperty $rows "child_reference_candidate_count"
            avg_results = Measure-AverageProperty $rows "result_fragment_count"
        })
    }

    $reportLines.Add("## Workload Scale")
    Add-MarkdownTable -Lines $reportLines -Headers @(
        "workload",
        "avg_read_ms",
        "avg_prepare_ms",
        "avg_elapsed_ms",
        "avg_solve_ms",
        "avg_export_ms",
        "avg_nodes",
        "avg_leaves",
        "avg_leaf_points",
        "avg_leaf_traces",
        "avg_child_ref_candidates",
        "avg_results"
    ) -Rows ($workloadRows.ToArray())
    $reportLines.Add("")

    $reportLines.Add("## Pipeline Inclusive Time")
    if ($InclusiveZoneRows.Count -gt 0) {
        $pipelineZoneNames = @(
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
            "SubdivisionSolver::classifyLeafFragmentsAndCollectResults"
        )

        $pipelineRows = New-Object System.Collections.Generic.List[object]
        foreach ($zoneName in $pipelineZoneNames) {
            $zone = Find-ZoneAggregate $inclusiveAggregates $zoneName
            if ($zone) {
                $pipelineRows.Add($zone)
            }
        }

        Add-MarkdownTable -Lines $reportLines -Headers @("name", "counts", "total_ms", "mean_us", "max_ms") -Rows ($pipelineRows.ToArray())
    }
    else {
        $reportLines.Add("No Tracy inclusive zone data was captured because -NoTracy was used.")
    }
    $reportLines.Add("")

    $reportLines.Add("## Strategy/Optimization Summary")
    if ($InclusiveZoneRows.Count -gt 0) {
        $strategyRows = New-Object System.Collections.Generic.List[object]
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "invalid_or_empty_discard" "invalid_or_empty_discard_count" @("SubdivisionSolver::discardInvalidOrEmptyNode")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "constant_discard" "constant_discard_count" @("SubdivisionSolver::discardConstantIndicatorNode", "SubdivisionSolver::discardChildConstantIndicator")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_constant_discard" "child_constant_discard_count" @("SubdivisionSolver::discardChildConstantIndicator")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_threshold_stop" "leaf_threshold_stop_count" @("SubdivisionSolver::stopByLeafThreshold")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "aabb_not_splittable_stop" "aabb_not_splittable_stop_count" @("SubdivisionSolver::stopByAabbNotSplittable")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "split_failure_stop" "split_failure_stop_count" @("SubdivisionSolver::stopBySplitFailure")))
        $splitDecisionMetric = Measure-SumProperties $TimingRows @("wntv_aware_split_count", "center_range_split_count", "midpoint_split_count", "split_failure_stop_count")
        $strategyRows.Add((New-StrategySummaryRowFromMetricValue $splitDecisionMetric $inclusiveAggregates $selfAggregates "split_decision_attempt" @("chooseSubdivisionSplit")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "wntv_aware_split" "wntv_aware_split_count" @("SubdivisionSolver::splitStrategyWntvAware")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "center_range_split" "center_range_split_count" @("SubdivisionSolver::splitStrategyCenterRange")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "midpoint_split" "midpoint_split_count" @("SubdivisionSolver::splitStrategyMidpoint")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "single_operand_assumption_stop" "single_operand_assumption_stop_count" @("SubdivisionSolver::singleOperandAssumptionStop")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "single_operand_assumption_fallback" "single_operand_assumption_fallback_count" @("SubdivisionSolver::singleOperandAssumptionFallback")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "single_operand_leaf_bsp_skip" "single_operand_leaf_bsp_skip_count" @("SubdivisionSolver::skipLeafBspBySingleOperandAssumption")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "single_operand_classification_reuse" "single_operand_classification_reuse_count" @("LeafClassification::reuseSingleOperandClassification")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_bsp_build" "leaf_bsp_build_count" @("buildLeafArrangement")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_reuse" "child_reference_reuse_count" @("SubdivisionSolver::reuseChildReference")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_candidate" "child_reference_candidate_count" @("SubdivisionSolver::childReferenceFastCandidate", "SubdivisionSolver::childReferenceExhaustiveCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_fast_candidate" "child_reference_fast_candidate_count" @("SubdivisionSolver::childReferenceFastCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_exhaustive_candidate" "child_reference_exhaustive_candidate_count" @("SubdivisionSolver::childReferenceExhaustiveCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_candidate_tried" "child_reference_candidate_tried_count" @("SubdivisionSolver::childReferenceFastCandidateTrace", "SubdivisionSolver::childReferenceExhaustiveCandidateTrace")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_fast_candidate_tried" "child_reference_fast_candidate_tried_count" @("SubdivisionSolver::childReferenceFastCandidateTrace")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_exhaustive_candidate_tried" "child_reference_exhaustive_candidate_tried_count" @("SubdivisionSolver::childReferenceExhaustiveCandidateTrace")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "child_reference_trace_success" "child_reference_trace_count" @("SubdivisionSolver::childReferenceTraceSuccess")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_point_candidate" "leaf_classification_point_candidate_count" @("LeafClassification::primaryPointCandidate", "LeafClassification::expandedPointCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_primary_point_candidate" "leaf_classification_primary_point_candidate_count" @("LeafClassification::primaryPointCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_expanded_point_candidate" "leaf_classification_expanded_point_candidate_count" @("LeafClassification::expandedPointCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_trace_attempt" "leaf_classification_trace_attempt_count" @("LeafClassification::traceCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_fast_candidate" "leaf_classification_fast_candidate_count" @("LeafClassification::fastCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_fallback_candidate" "leaf_classification_fallback_candidate_count" @("LeafClassification::fallbackCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_normal_candidate" "leaf_classification_normal_candidate_count" @("LeafClassification::normalCandidate")))
        $strategyRows.Add((New-StrategySummaryRow $TimingRows $inclusiveAggregates $selfAggregates "leaf_classification_interior_bridge_candidate" "leaf_classification_interior_bridge_candidate_count" @("LeafClassification::interiorBridgeCandidate")))

        Add-MarkdownTable -Lines $reportLines -Headers @("item", "metric_count", "zone_count", "status", "inclusive_ms", "self_ms", "zone_name") -Rows ($strategyRows.ToArray())

        $mismatchRows = @($strategyRows | Where-Object { $_.status -ne "ok" })
        if ($mismatchRows.Count -gt 0) {
            $reportLines.Add("")
            $reportLines.Add("Mismatch rows:")
            foreach ($row in $mismatchRows) {
                $reportLines.Add(("- {0}: metrics={1}, zone={2}" -f $row.item, $row.metric_count, $row.zone_count))
            }
        }
    }
    else {
        $reportLines.Add("No Tracy zone data was captured because -NoTracy was used; strategy timing cross-checks are unavailable.")
    }
    $reportLines.Add("")

    $reportLines.Add("## Per-Workload Self Hot Zones")
    if ($SelfZoneRows.Count -gt 0) {
        foreach ($workloadGroup in ($selfWorkloadAggregates | Group-Object workload)) {
            $reportLines.Add(("### {0}" -f $workloadGroup.Name))
            Add-MarkdownTable -Lines $reportLines -Headers @("name", "counts", "total_ms", "mean_us", "max_ms") -Rows @($workloadGroup.Group | Sort-Object total_ms -Descending | Select-Object -First 12)
            $reportLines.Add("")
        }
    }
    else {
        $reportLines.Add("No Tracy self-time zone data was captured because -NoTracy was used.")
        $reportLines.Add("")
    }

    if ($UnwrapExports.Count -gt 0) {
        $reportLines.Add("### Unwrap Exports")
        Add-MarkdownTable -Lines $reportLines -Headers @("workload", "iteration", "zone_filter", "event_count", "path") -Rows @($UnwrapExports | Sort-Object workload, iteration, zone_filter)
        $reportLines.Add("")
    }

    $reportPath = Join-Path $PerfRoot "report.md"
    Set-Content -LiteralPath $reportPath -Value $reportLines -Encoding UTF8

    $summaryLines = New-Object System.Collections.Generic.List[string]
    $summaryLines.Add(("executable={0}" -f $script:ExePath))
    $summaryLines.Add(("configuration={0}" -f $Configuration))
    $summaryLines.Add(("iterations={0}" -f $Iterations))
    $summaryLines.Add(("timings_csv={0}" -f $TimingCsvPath))
    if ($InclusiveZonesCsvPath) {
        $summaryLines.Add(("tracy_zones_csv={0}" -f $InclusiveZonesCsvPath))
    }
    if ($SelfZonesCsvPath) {
        $summaryLines.Add(("tracy_zones_self_csv={0}" -f $SelfZonesCsvPath))
    }
    if ($UnwrapExports.Count -gt 0) {
        $summaryLines.Add(("tracy_unwrap_root={0}" -f $UnwrapRoot))
    }
    $summaryLines.Add(("report={0}" -f $reportPath))
    foreach ($row in $workloadRows) {
        $summaryLines.Add(("workload={0}" -f $row.workload))
        $summaryLines.Add(("  avg_read_ms={0}" -f $row.avg_read_ms))
        $summaryLines.Add(("  avg_prepare_ms={0}" -f $row.avg_prepare_ms))
        $summaryLines.Add(("  avg_elapsed_ms={0}" -f $row.avg_elapsed_ms))
        $summaryLines.Add(("  avg_solve_ms={0}" -f $row.avg_solve_ms))
        $summaryLines.Add(("  avg_export_ms={0}" -f $row.avg_export_ms))
        $summaryLines.Add(("  avg_nodes={0}" -f $row.avg_nodes))
        $summaryLines.Add(("  avg_leaf_points={0}" -f $row.avg_leaf_points))
        $summaryLines.Add(("  avg_leaf_traces={0}" -f $row.avg_leaf_traces))
    }
    foreach ($unwrapExport in ($UnwrapExports | Sort-Object workload, iteration, zone_filter)) {
        $summaryLines.Add(("unwrap={0} workload={1} iteration={2} events={3} path={4}" -f $unwrapExport.zone_filter, $unwrapExport.workload, $unwrapExport.iteration, $unwrapExport.event_count, $unwrapExport.path))
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
    Write-Info ("Using profiling build tree: {0}" -f $BuildRoot)

    if (-not $SkipBuild) {
        Invoke-CMakeConfigure
    }

    $script:TracyEnabledInBuild = Test-TracyEnabledInBuild $BuildRoot
    $script:MathTracyEnabledInBuild = Test-TracyMathEnabledInBuild $BuildRoot

    if ($NoTracy -and $script:TracyEnabledInBuild) {
        throw ("The selected profiling build tree '{0}' is configured with REEMBER_ENABLE_TRACY=ON, but -NoTracy was requested. Re-run without -SkipBuild so the script can configure build\\profile_notracy\\ automatically." -f $BuildRoot)
    }
    if ((-not $NoTracy) -and (-not $script:TracyEnabledInBuild)) {
        throw ("The profiling build tree '{0}' is not configured with REEMBER_ENABLE_TRACY=ON. Re-run without -SkipBuild so the script can configure it automatically." -f $BuildRoot)
    }
    if ($EnableMathTracy -and (-not $script:MathTracyEnabledInBuild)) {
        throw ("The profiling build tree '{0}' is not configured with REEMBER_ENABLE_TRACY_MATH=ON. Re-run without -SkipBuild so the script can configure it automatically." -f $BuildRoot)
    }
    $script:MathTracyEnabledInBuild = (-not $NoTracy) -and $script:MathTracyEnabledInBuild
    if ($SkipBuild -and (-not $EnableMathTracy) -and $script:MathTracyEnabledInBuild) {
        throw ("The selected profiling build tree '{0}' already has REEMBER_ENABLE_TRACY_MATH=ON, but -EnableMathTracy was not requested. Re-run without -SkipBuild so the script can reconfigure build\\profile_tracy\\ automatically." -f $BuildRoot)
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
        buildFlavor = [string]$BuildFlavor
        outputDir = [string]$PerfRoot
        timestamp = $RunStamp
        configuration = $Configuration
        executable = $script:ExePath
        leafThreshold = $LeafThreshold
        iterations = $Iterations
        timeoutSeconds = $TimeoutSeconds
        tracyEnabled = (-not $NoTracy)
        mathTracyEnabled = $script:MathTracyEnabledInBuild
        tracyPort = $script:ResolvedTracyPort
        tracyCapture = $tracyCapturePath
        tracyCsvExport = $tracyCsvExportPath
        unwrapZoneFilter = $UnwrapZoneFilter
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

    $inclusiveZoneRows = @()
    $selfZoneRows = @()
    $unwrapExports = @()
    $inclusiveZonesCsvPath = $null
    $selfZonesCsvPath = $null
    if (-not $NoTracy) {
        $zoneResult = Export-TracyZones -CsvExportPath $tracyCsvExportPath -TraceRecords ($traceRecords.ToArray())
        $inclusiveZonesCsvPath = $zoneResult.Path
        $inclusiveZoneRows = @($zoneResult.Rows)

        $selfZoneResult = Export-TracySelfZones -CsvExportPath $tracyCsvExportPath -TraceRecords ($traceRecords.ToArray())
        $selfZonesCsvPath = $selfZoneResult.Path
        $selfZoneRows = @($selfZoneResult.Rows)

        if ($UnwrapZoneFilter.Count -gt 0) {
            $unwrapExports = @(Export-TracyUnwrapRows -CsvExportPath $tracyCsvExportPath -TraceRecords ($traceRecords.ToArray()) -ZoneFilters $UnwrapZoneFilter)
        }
    }

    $reportPath = Write-Reports `
        -TimingRows ($timingRows.ToArray()) `
        -InclusiveZoneRows @($inclusiveZoneRows) `
        -SelfZoneRows @($selfZoneRows) `
        -UnwrapExports @($unwrapExports) `
        -TimingCsvPath $timingCsvPath `
        -InclusiveZonesCsvPath $inclusiveZonesCsvPath `
        -SelfZonesCsvPath $selfZonesCsvPath
    Write-Info ("Done. See report: {0}" -f $reportPath)
}
finally {
    if ($TranscriptStarted) {
        Stop-Transcript | Out-Null
    }
}
