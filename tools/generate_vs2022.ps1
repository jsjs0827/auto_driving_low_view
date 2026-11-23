param(
    [string]$OrtRoot = '',
    [switch]$Build,
    [ValidateSet('Debug','Release')][string]$Configuration = 'Debug',
    [switch]$Test,
    [string]$Target = '',
    [switch]$Fresh
)
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$cmakeCommand = Get-Command cmake.exe -ErrorAction SilentlyContinue
if ($cmakeCommand) { $cmake = $cmakeCommand.Source }
else {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (!(Test-Path -LiteralPath $vswhere)) { throw 'Install Visual Studio 2022 Desktop development with C++ and CMake.' }
    $vsInstall = & $vswhere -latest -version '[17.0,18.0)' -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    $cmake = Join-Path $vsInstall 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
}
if (!(Test-Path -LiteralPath $cmake)) { throw 'CMake 3.21 or newer is required.' }
function Invoke-CleanEnvironmentTool([string]$Exe, [string[]]$ToolArgs) {
    $start = New-Object Diagnostics.ProcessStartInfo
    $start.FileName = $Exe
    $start.WorkingDirectory = $projectRoot
    $start.UseShellExecute = $false
    # Normalize inherited key casing, including PATH/Path from some launchers.
    $start.EnvironmentVariables.Clear()
    foreach ($entry in [Environment]::GetEnvironmentVariables().GetEnumerator()) {
        $start.EnvironmentVariables[$entry.Key.ToUpperInvariant()] = [string]$entry.Value
    }
    if ($start.PSObject.Properties.Name -contains 'ArgumentList') {
        foreach ($argument in $ToolArgs) { $start.ArgumentList.Add($argument) }
    } else {
        $quoted = foreach ($argument in $ToolArgs) {
            '"' + [regex]::Replace([regex]::Replace($argument, '(\\*)"', '$1$1\"'), '(\\+)$', '$1$1') + '"'
        }
        $start.Arguments = $quoted -join ' '
    }
    $process = [Diagnostics.Process]::Start($start)
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) { throw "Tool failed with exit code $($process.ExitCode): $Exe" }
}
$configure = @('--preset','windows-vs2022')
if ($Fresh) { $configure = @('--fresh') + $configure }
if ($OrtRoot) { $configure += ('-DORT_ROOT=' + [IO.Path]::GetFullPath($OrtRoot)) }
Invoke-CleanEnvironmentTool $cmake $configure
if ($Target -and $Test) { throw '-Test requires building all test targets; omit -Target.' }
if ($Build -or $Test) {
    $buildArguments = @('--build', (Join-Path $projectRoot 'build/vs2022'), '--config', $Configuration, '--parallel', '4')
    if ($Target) { $buildArguments += @('--target', $Target) }
    Invoke-CleanEnvironmentTool $cmake $buildArguments
}
if ($Test) { Invoke-CleanEnvironmentTool (Join-Path (Split-Path $cmake) 'ctest.exe') @('--test-dir', (Join-Path $projectRoot 'build/vs2022'), '-C', $Configuration, '--output-on-failure') }
Write-Host ('Solution: ' + (Join-Path $projectRoot 'build/vs2022/perception_stereo_cpp_v8.sln'))
