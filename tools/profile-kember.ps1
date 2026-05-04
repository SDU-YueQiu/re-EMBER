# Profiles kEmber CLI runs and writes timing/ETW artifacts under build\perf.
param(
    [string]$Lhs,
    [string]$Rhs,
    [ValidateSet("union", "intersection", "difference")]
    [string]$Op = "difference",
    [string]$Out,
    [int]$LeafThreshold = 25,
    [int]$Iterations = 3,
    [int]$TimeoutSeconds = 300,
    [int]$BuildTimeoutSeconds = 900,
    [int]$ReportTimeoutSeconds = 120,
    [switch]$SkipBuild,
    [switch]$NoEtw,
    [switch]$ResolveSymbols,
    [switch]$ForceStopExisting,
    [switch]$Help
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Show-Help {
    @"
Usage:
  powershell -ExecutionPolicy Bypass -File .\tools\profile-kember.ps1
  powershell -ExecutionPolicy Bypass -File .\tools\profile-kember.ps1 -Lhs <lhs.obj> -Rhs <rhs.obj> -Op difference

Recommended:
  Run from an elevated PowerShell if you want ETW CPU stacks.

Outputs:
  build\perf\run_<timestamp>\profile.log
  build\perf\run_<timestamp>\timings.csv
  build\perf\run_<timestamp>\xperf_profile_detail.txt
  build\perf\run_<timestamp>\xperf_stack_butterfly.txt
  build\perf\run_<timestamp>\kember_cpu.etl

Notes:
  - Uses build\ only.
  - If -Lhs/-Rhs are omitted, the script tries common repo assets.
  - Use -NoEtw for timing-only runs.
  - Xperf text reports are bounded by -ReportTimeoutSeconds.
  - Use -ResolveSymbols only when you need symbol-server resolution.
  - Use -ForceStopExisting only if a stale ETW kernel logger blocks xperf.
"@
}

if ($Help) {
    Show-Help
    exit 0
}

$RepoRoot = Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")
Set-Location -LiteralPath $RepoRoot

$RunStamp = Get-Date -Format "yyyyMMdd_HHmmss"
$PerfRoot = Join-Path $RepoRoot ("build\perf\run_{0}" -f $RunStamp)
New-Item -ItemType Directory -Force -Path $PerfRoot | Out-Null

$TranscriptPath = Join-Path $PerfRoot "profile.log"
$TranscriptStarted = $false
$CreateZipAtEnd = $false
$TextZip = $null

function Write-Info {
    param([string]$Message)
    $line = "[profile-kember] {0}" -f $Message
    Write-Host $line
}

function Test-IsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Get-CommandPath {
    param([string]$Name)
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if ($null -eq $cmd) {
        $knownPaths = @()
        if ($Name -ieq "wpr.exe") {
            $knownPaths += (Join-Path $env:SystemRoot "System32\wpr.exe")
        }
        elseif ($Name -ieq "xperf.exe") {
            if (${env:ProgramFiles(x86)}) {
                $knownPaths += (Join-Path ${env:ProgramFiles(x86)} "Windows Kits\10\Windows Performance Toolkit\xperf.exe")
            }
            if ($env:ProgramFiles) {
                $knownPaths += (Join-Path $env:ProgramFiles "Windows Kits\10\Windows Performance Toolkit\xperf.exe")
            }
        }

        foreach ($path in $knownPaths) {
            if (Test-Path -LiteralPath $path) {
                return $path
            }
        }

        return $null
    }
    return $cmd.Source
}

function Count-ObjFaces {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        return $null
    }
    return (Select-String -LiteralPath $Path -Pattern '^\s*f\s+' -ErrorAction Stop).Count
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
        Out = $OutPath
        LhsFaces = Count-ObjFaces $LhsPath
        RhsFaces = Count-ObjFaces $RhsPath
    }
}

function Get-Workloads {
    $items = New-Object System.Collections.Generic.List[object]

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

    $defaults = @(
        [pscustomobject]@{
            Name = "icosphere80_toolbox_difference"
            Lhs = "assets\models\test_icosphere_80.obj"
            Rhs = "assets\models\tool_box.obj"
            Op = "difference"
        },
        [pscustomobject]@{
            Name = "visual_block_toolbox_intersection"
            Lhs = "assets\visual_test\workpiece_block.obj"
            Rhs = "assets\visual_test\tool_box.obj"
            Op = "intersection"
        }
    )

    foreach ($item in $defaults) {
        if ((Test-Path -LiteralPath $item.Lhs) -and (Test-Path -LiteralPath $item.Rhs)) {
            $outPath = Join-Path $PerfRoot ("result_{0}.obj" -f $item.Name)
            $items.Add((New-Workload $item.Name $item.Lhs $item.Rhs $item.Op $outPath))
        }
        else {
            Write-Info ("Skipping missing default workload: {0}" -f $item.Name)
        }
    }

    if ($items.Count -eq 0) {
        throw "No default workload assets were found. Re-run with -Lhs <lhs.obj> -Rhs <rhs.obj> -Op <operation>."
    }

    return $items
}

function Invoke-KemberWorkload {
    param(
        [string]$ExePath,
        [object]$Workload,
        [string]$Tag
    )

    $stdoutPath = Join-Path $PerfRoot ("{0}_{1}.stdout.txt" -f $Tag, $Workload.Name)
    $stderrPath = Join-Path $PerfRoot ("{0}_{1}.stderr.txt" -f $Tag, $Workload.Name)
    $outPath = $Workload.Out
    if ($Tag -ne "timing") {
        $outPath = Join-Path $PerfRoot ("result_{0}_{1}.obj" -f $Tag, $Workload.Name)
    }

    $args = @(
        "--lhs", $Workload.Lhs,
        "--rhs", $Workload.Rhs,
        "--op", $Workload.Op,
        "--out", $outPath,
        "--leaf-threshold", [string]$LeafThreshold
    )

    return Invoke-External $ExePath $args $stdoutPath $stderrPath $TimeoutSeconds
}

function Export-XperfReports {
    param(
        [string]$XperfPath,
        [string]$EtlPath,
        [string]$SymbolCache,
        [string]$ReleaseDir,
        [bool]$UseSymbols,
        [int]$ReportTimeoutSec
    )

    if (-not $XperfPath) {
        Write-Info "xperf.exe is unavailable; keeping ETL only."
        return
    }

    if ($UseSymbols) {
        New-Item -ItemType Directory -Force -Path $SymbolCache | Out-Null
        $env:_NT_SYMBOL_PATH = ("{0};srv*{1}*https://msdl.microsoft.com/download/symbols" -f $ReleaseDir, $SymbolCache)
        $env:_NT_SYMCACHE_PATH = $SymbolCache
    }
    else {
        Write-Info "Skipping xperf symbol resolution. Re-run with -ResolveSymbols for symbol-server lookup."
    }

    $profileOut = Join-Path $PerfRoot "xperf_profile_detail.txt"
    $profileErr = Join-Path $PerfRoot "xperf_profile_detail.err.txt"
    $stackOut = Join-Path $PerfRoot "xperf_stack_butterfly.txt"
    $stackErr = Join-Path $PerfRoot "xperf_stack_butterfly.err.txt"
    $processOut = Join-Path $PerfRoot "xperf_process.txt"
    $processErr = Join-Path $PerfRoot "xperf_process.err.txt"
    $processCmdlineOut = Join-Path $PerfRoot "xperf_process_cmdline.txt"
    $processCmdlineErr = Join-Path $PerfRoot "xperf_process_cmdline.err.txt"

    $symbolArg = @()
    if ($UseSymbols) {
        $symbolArg = @("-symbols")
    }

    $reports = @(
        [pscustomobject]@{
            Name = "xperf profile"
            Args = @("-i", $EtlPath) + $symbolArg + @("-o", $profileOut, "-a", "profile", "-detail")
            Stderr = $profileErr
        },
        [pscustomobject]@{
            Name = "xperf stack"
            Args = @("-i", $EtlPath) + $symbolArg + @("-o", $stackOut, "-a", "stack", "-process", "kEmber.exe", "-butterfly", "1")
            Stderr = $stackErr
        },
        [pscustomobject]@{
            Name = "xperf process"
            Args = @("-i", $EtlPath, "-o", $processOut, "-a", "process")
            Stderr = $processErr
        },
        [pscustomobject]@{
            Name = "xperf process cmdline"
            Args = @("-i", $EtlPath, "-o", $processCmdlineOut, "-a", "process", "-withcmdline")
            Stderr = $processCmdlineErr
        }
    )

    foreach ($report in $reports) {
        try {
            [void](Invoke-External $XperfPath $report.Args $null $report.Stderr $ReportTimeoutSec -IgnoreExit)
        }
        catch {
            Write-Warning ("Skipping {0}: {1}" -f $report.Name, $_.Exception.Message)
        }
    }
}

function Invoke-EtwProfile {
    param(
        [string]$ExePath,
        [object[]]$Workloads,
        [string]$XperfPath,
        [string]$WprPath,
        [bool]$IsAdmin
    )

    if ($NoEtw) {
        Write-Info "Skipping ETW because -NoEtw was specified."
        return
    }

    if (-not $IsAdmin) {
        Write-Info "Skipping ETW because this PowerShell is not elevated. Re-run as Administrator for CPU stacks."
        return
    }

    if ((-not $XperfPath) -and (-not $WprPath)) {
        Write-Info "Skipping ETW because neither xperf.exe nor wpr.exe was found."
        return
    }

    $etlPath = Join-Path $PerfRoot "kember_cpu.etl"
    $rawEtlPath = Join-Path $PerfRoot "kember_cpu_raw.etl"
    $releaseDir = Split-Path -Parent $ExePath
    $symbolCache = Join-Path $PerfRoot "symcache"

    $mode = $null
    $started = $false
    $stopped = $false

    try {
        if ($ForceStopExisting -and $XperfPath) {
            Write-Info "Force-stopping any existing xperf kernel logger."
            [void](Invoke-External $XperfPath @("-stop") $null (Join-Path $PerfRoot "xperf_force_stop.err.txt") 60 -IgnoreExit)
        }

        if ($XperfPath) {
            $startOut = Join-Path $PerfRoot "xperf_start.stdout.txt"
            $startErr = Join-Path $PerfRoot "xperf_start.stderr.txt"
            $startArgs = @(
                "-on", "PROC_THREAD+LOADER+PROFILE",
                "-stackwalk", "Profile",
                "-BufferSize", "1024",
                "-MaxFile", "512",
                "-FileMode", "Circular",
                "-f", $rawEtlPath
            )
            $startResult = Invoke-External $XperfPath $startArgs $startOut $startErr 60 -IgnoreExit
            if ($startResult.ExitCode -eq 0) {
                $mode = "xperf"
                $started = $true
            }
            else {
                Write-Info "xperf start failed; trying WPR if available."
            }
        }

        if ((-not $started) -and $WprPath) {
            $startOut = Join-Path $PerfRoot "wpr_start.stdout.txt"
            $startErr = Join-Path $PerfRoot "wpr_start.stderr.txt"
            $startResult = Invoke-External $WprPath @("-start", "CPU", "-filemode") $startOut $startErr 60 -IgnoreExit
            if ($startResult.ExitCode -eq 0) {
                $mode = "wpr"
                $started = $true
            }
            else {
                Write-Info "WPR start also failed; ETW trace was not collected."
                return
            }
        }

        foreach ($workload in $Workloads) {
            [void](Invoke-KemberWorkload $ExePath $workload "etw")
        }
    }
    finally {
        if ($started -and (-not $stopped)) {
            if ($mode -eq "xperf") {
                [void](Invoke-External $XperfPath @("-d", $etlPath) (Join-Path $PerfRoot "xperf_stop.stdout.txt") (Join-Path $PerfRoot "xperf_stop.stderr.txt") 180 -IgnoreExit)
                $stopped = $true
            }
            elseif ($mode -eq "wpr") {
                [void](Invoke-External $WprPath @("-stop", $etlPath) (Join-Path $PerfRoot "wpr_stop.stdout.txt") (Join-Path $PerfRoot "wpr_stop.stderr.txt") 180 -IgnoreExit)
                $stopped = $true
            }
        }
    }

    if (Test-Path -LiteralPath $etlPath) {
        Export-XperfReports $XperfPath $etlPath $symbolCache $releaseDir $ResolveSymbols.IsPresent $ReportTimeoutSeconds
    }
    else {
        Write-Info "ETW stop did not produce kember_cpu.etl."
    }
}

try {
    Start-Transcript -Path $TranscriptPath -Force | Out-Null
    $TranscriptStarted = $true

    $isAdmin = Test-IsAdministrator
    $cmakePath = Get-CommandPath "cmake.exe"
    $xperfPath = Get-CommandPath "xperf.exe"
    $wprPath = Get-CommandPath "wpr.exe"

    Write-Info ("Repo root: {0}" -f $RepoRoot)
    Write-Info ("Output dir: {0}" -f $PerfRoot)
    Write-Info ("Administrator: {0}" -f $isAdmin)
    Write-Info ("cmake: {0}" -f $(if ($cmakePath) { $cmakePath } else { "<missing>" }))
    Write-Info ("xperf: {0}" -f $(if ($xperfPath) { $xperfPath } else { "<missing>" }))
    Write-Info ("wpr: {0}" -f $(if ($wprPath) { $wprPath } else { "<missing>" }))

    $exePath = Join-Path $RepoRoot "build\Release\kEmber.exe"
    if ((-not (Test-Path -LiteralPath $exePath)) -and $SkipBuild) {
        throw ("Missing executable and -SkipBuild was specified: {0}" -f $exePath)
    }

    if (-not $SkipBuild) {
        if (-not $cmakePath) {
            throw "cmake.exe was not found in PATH."
        }
        if (-not (Test-Path -LiteralPath (Join-Path $RepoRoot "build\CMakeCache.txt"))) {
            [void](Invoke-External $cmakePath @("-S", ".", "-B", "build") (Join-Path $PerfRoot "cmake_configure.stdout.txt") (Join-Path $PerfRoot "cmake_configure.stderr.txt") $BuildTimeoutSeconds)
        }
        [void](Invoke-External $cmakePath @("--build", "build", "--config", "Release", "--target", "kEmber") (Join-Path $PerfRoot "cmake_build.stdout.txt") (Join-Path $PerfRoot "cmake_build.stderr.txt") $BuildTimeoutSeconds)
    }

    if (-not (Test-Path -LiteralPath $exePath)) {
        throw ("Missing executable after build: {0}" -f $exePath)
    }

    $workloads = @(Get-Workloads)
    $manifest = [pscustomobject]@{
        repoRoot = [string]$RepoRoot
        outputDir = [string]$PerfRoot
        timestamp = $RunStamp
        administrator = $isAdmin
        executable = $exePath
        leafThreshold = $LeafThreshold
        iterations = $Iterations
        timeoutSeconds = $TimeoutSeconds
        workloads = $workloads
    }
    $manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $PerfRoot "manifest.json") -Encoding UTF8

    $timingRows = New-Object System.Collections.Generic.List[object]
    foreach ($workload in $workloads) {
        Write-Info ("Timing workload {0}: lhs_faces={1} rhs_faces={2} op={3}" -f $workload.Name, $workload.LhsFaces, $workload.RhsFaces, $workload.Op)
        for ($i = 1; $i -le $Iterations; ++$i) {
            $result = Invoke-KemberWorkload $exePath $workload ("timing_{0}" -f $i)
            $timingRows.Add([pscustomobject]@{
                workload = $workload.Name
                iteration = $i
                elapsed_ms = $result.ElapsedMs
                exit_code = $result.ExitCode
                lhs_faces = $workload.LhsFaces
                rhs_faces = $workload.RhsFaces
                op = $workload.Op
                stdout = $result.StdoutPath
                stderr = $result.StderrPath
            })
        }
    }

    $timingRows | Export-Csv -LiteralPath (Join-Path $PerfRoot "timings.csv") -NoTypeInformation -Encoding UTF8

    Invoke-EtwProfile $exePath $workloads $xperfPath $wprPath $isAdmin

    $TextZip = Join-Path (Split-Path -Parent $PerfRoot) ("{0}_text.zip" -f (Split-Path -Leaf $PerfRoot))
    $CreateZipAtEnd = $true

    Write-Info "Done. Send profile.log, timings.csv, and xperf_*.txt back for analysis. Keep kember_cpu.etl for WPA if stack text is missing."
}
finally {
    if ($TranscriptStarted) {
        Stop-Transcript | Out-Null
    }
    if ($CreateZipAtEnd -and $TextZip) {
        try {
            $textFiles = Get-ChildItem -LiteralPath $PerfRoot -File | Where-Object { $_.Extension -ne ".etl" }
            if ($textFiles.Count -gt 0) {
                Compress-Archive -Path $textFiles.FullName -DestinationPath $TextZip -Force
                Write-Host ("[profile-kember] Text result zip: {0}" -f $TextZip)
            }
        }
        catch {
            Write-Warning ("Could not create text zip: {0}" -f $_.Exception.Message)
        }
    }
}
