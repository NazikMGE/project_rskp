<#
.SYNOPSIS
  Builds the host tests of the firmware modules, runs them, collects the
  branch-coverage report and runs the control example.

.DESCRIPTION
  Steps:
    1. configure and build firmware/test/host with CMake + Ninja (GCC,
       coverage instrumentation of the modules under test);
    2. run the unit tests and save their output;
    3. run gcov on the instrumented modules (coverage of the unit tests
       only - the control example is started after the report is taken);
    4. run the control example and save its CSV files and journal.

  Everything is written to project/results. The build directory is kept
  outside the repository, in an ASCII-only temporary path: the instrumented
  executables write their .gcda files by the absolute path recorded at
  compile time, which must not contain non-ASCII characters.

.PARAMETER BuildDir
  Build directory. Default: <temp>\stability_tracker_host_build.

.PARAMETER Clean
  Remove the build directory first.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File tools\run_host_tests.ps1 -Clean
#>
param(
  [string]$BuildDir = (Join-Path ([IO.Path]::GetTempPath()) "stability_tracker_host_build"),
  [switch]$Clean
)

[Console]::OutputEncoding = [Text.Encoding]::UTF8

$project = Split-Path -Parent $PSScriptRoot
$fw = Join-Path $project "firmware"
$results = Join-Path $project "results"
$coverageDir = Join-Path $results "coverage"
$exampleDir = Join-Path $results "control_example"

function Save-Lines([string]$Path, $Lines) {
  # UTF-8 without BOM, LF-independent: one string per line.
  [IO.File]::WriteAllLines($Path, [string[]]@($Lines))
}

function Assert-Ok([string]$What) {
  if ($LASTEXITCODE -ne 0) {
    Write-Host "FAILED: $What (exit code $LASTEXITCODE)"
    exit 1
  }
}

foreach ($dir in @($results, $coverageDir, $exampleDir)) {
  New-Item -ItemType Directory -Force -Path $dir | Out-Null
}
if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
  Remove-Item -Recurse -Force -LiteralPath $BuildDir
}

# ---- 1. build ---------------------------------------------------------
Write-Host "== configure and build"
cmake -S (Join-Path $fw "test\host") -B $BuildDir -G Ninja -DHOST_COVERAGE=ON | Out-Null
Assert-Ok "cmake configure"
$buildLog = cmake --build $BuildDir
Assert-Ok "cmake build"
Save-Lines (Join-Path $results "build_log.txt") $buildLog

# Stale counters from an earlier run would inflate the coverage.
Get-ChildItem -Path $BuildDir -Recurse -Filter *.gcda | Remove-Item -Force

# ---- 2. unit tests ----------------------------------------------------
$failed = 0
foreach ($test in @("shot_detect_test", "stability_test", "imu_profile_test")) {
  Write-Host "== $test"
  $out = & (Join-Path $BuildDir "$test.exe")
  $code = $LASTEXITCODE
  $out | ForEach-Object { Write-Host $_ }
  Save-Lines (Join-Path $results "$test.txt") (@($out) + "exit code: $code")
  if ($code -ne 0) { $failed++ }
}

# ---- 3. coverage of the unit tests ------------------------------------
Write-Host "== coverage (gcov)"
$summary = @()
$uncovered = @()
Push-Location -LiteralPath $fw   # sources are recorded relative to this directory
foreach ($gcda in (Get-ChildItem -Path $BuildDir -Recurse -Filter *.gcda | Sort-Object Name)) {
  $module = $gcda.Name -replace '\.gcda$', ''
  $listing = gcov -b -c -t $gcda.FullName
  Save-Lines (Join-Path $coverageDir "$module.gcov") $listing
  $summary += gcov -b -c -n $gcda.FullName | Where-Object { $_ -match "^(File|Lines|Branches|Taken|Calls)" } | Select-Object -First 5
  $summary += ""

  # Branch outcomes that no test produced, with the source line they belong to.
  $uncovered += "== $module"
  $source = ""
  foreach ($line in $listing) {
    if ($line -match '^\s*(-|#####|\d+\*?):\s*(\d+):(.*)$') { $source = "line $($Matches[2]):$($Matches[3])" }
    if ($line -match '^branch\s+\d+\s+(never executed|taken 0)') { $uncovered += "$line  <=  $source" }
  }
  $uncovered += ""
}
Pop-Location
$summary | ForEach-Object { Write-Host $_ }
Save-Lines (Join-Path $results "coverage_summary.txt") $summary
Save-Lines (Join-Path $results "coverage_uncovered.txt") $uncovered

# ---- 4. control example -----------------------------------------------
Write-Host "== control example"
Push-Location -LiteralPath $exampleDir
$out = & (Join-Path $BuildDir "control_example.exe") "."
$code = $LASTEXITCODE
Pop-Location
$out | ForEach-Object { Write-Host $_ }
Save-Lines (Join-Path $exampleDir "control_example_output.txt") (@($out) + "exit code: $code")
if ($code -ne 0) { $failed++ }

# ---- environment ------------------------------------------------------
$environment = @(
  "date: $(Get-Date -Format 'yyyy-MM-dd')",
  "os: $([Environment]::OSVersion.VersionString)",
  "gcc: $((gcc --version | Select-Object -First 1))",
  "gcov: $((gcov --version | Select-Object -First 1))",
  "cmake: $((cmake --version | Select-Object -First 1))",
  "ninja: $(ninja --version)"
)
Save-Lines (Join-Path $results "environment.txt") $environment

if ($failed -ne 0) {
  Write-Host "RESULT: $failed step(s) failed"
  exit 1
}
Write-Host "RESULT: all host tests and the control example passed"
exit 0
