# Run from iquip, with lolenc, lolenc_repo and artiq-proxy beside it.
# Example: .\start_lolenc_stack.ps1 -CheckOnly
[CmdletBinding()]
param(
    [string]$Root,
    [ValidatePattern('^[A-Za-z0-9_.-]+$')]
    [string]$EnvironmentName = 'lolenc-stack',
    [string]$CondaExe,
    [ValidateRange(1, 65535)]
    [int]$ProxyPort = 8000,
    [ValidateRange(1, 3600)]
    [int]$StartupTimeoutSeconds = 120,
    [switch]$CheckOnly,
    [ValidateSet('Launch', 'Master', 'Proxy', 'GUI')]
    [string]$Session = 'Launch',
    [string]$EnvironmentPrefix
)

$ErrorActionPreference = 'Stop'
$launcherScript = $PSCommandPath
if ([string]::IsNullOrWhiteSpace($Root)) {
    $Root = Split-Path -Parent $PSScriptRoot
}
$Root = [IO.Path]::GetFullPath($Root)
$iquipPath = Join-Path $Root 'iquip'
$repositoryPath = Join-Path $Root 'lolenc_repo'
$proxyPath = Join-Path $Root 'artiq-proxy'
$lolencPath = Join-Path $Root 'lolenc'
$runtimePath = Join-Path $iquipPath '.lolenc-stack'
# Keep the master config beside the experiment configs for the GUI config editor.
$masterConfigPath = Join-Path $repositoryPath 'configuration.lolenc-stack.json'
$proxyConfigPath = Join-Path $runtimePath 'proxy.json'
$guiConfigPath = Join-Path $runtimePath 'gui.json'

function Find-CondaExecutable {
    if ($CondaExe) {
        if (-not (Test-Path -LiteralPath $CondaExe -PathType Leaf)) {
            throw "Conda executable not found: $CondaExe"
        }
        return (Resolve-Path -LiteralPath $CondaExe).Path
    }
    $command = Get-Command conda.exe -ErrorAction SilentlyContinue
    $candidates = @(
        $env:CONDA_EXE
        $(if ($command) { $command.Source })
        $(if ($env:USERPROFILE) {
            Join-Path $env:USERPROFILE 'anaconda3\Scripts\conda.exe'
            Join-Path $env:USERPROFILE 'miniconda3\Scripts\conda.exe'
        })
        $(if ($env:LOCALAPPDATA) {
            Join-Path $env:LOCALAPPDATA 'anaconda3\Scripts\conda.exe'
            Join-Path $env:LOCALAPPDATA 'miniconda3\Scripts\conda.exe'
        })
        $(if ($env:ProgramData) {
            Join-Path $env:ProgramData 'anaconda3\Scripts\conda.exe'
            Join-Path $env:ProgramData 'miniconda3\Scripts\conda.exe'
        })
    )
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Leaf)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw 'Conda was not found. Pass -CondaExe "D:\Miniconda3\Scripts\conda.exe".'
}

function Get-EnvironmentPrefix([string]$Executable) {
    $infoText = & $Executable env list --json
    if ($LASTEXITCODE -ne 0) { throw 'Could not list Conda environments.' }
    $info = ($infoText -join "`n") | ConvertFrom-Json
    if ($EnvironmentName -eq 'base') {
        return Split-Path -Parent (Split-Path -Parent $Executable)
    }
    foreach ($prefix in $info.envs) {
        if ((Split-Path -Leaf $prefix) -eq $EnvironmentName) { return $prefix }
    }
    throw "Conda environment '$EnvironmentName' was not found. Install the stack in that environment first."
}

function Assert-Path([string]$Path, [string]$Kind = 'Leaf') {
    if (-not (Test-Path -LiteralPath $Path -PathType $Kind)) {
        throw "Required $Kind not found: $Path"
    }
}

function Read-Config([string]$Path) {
    Assert-Path $Path
    return Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json
}

function Set-ConfigValue($Config, [string]$Name, $Value) {
    $Config | Add-Member -MemberType NoteProperty -Name $Name -Value $Value -Force
}

function Resolve-ConfigPath([string]$Value, [string]$Directory) {
    if ([string]::IsNullOrWhiteSpace($Value)) { throw 'An empty configuration path was found.' }
    if (-not [IO.Path]::IsPathRooted($Value)) { $Value = Join-Path $Directory $Value }
    return [IO.Path]::GetFullPath($Value)
}

function Rebase-ConfigPath([string]$Value, [string]$Directory, $Mappings) {
    $absolute = Resolve-ConfigPath $Value $Directory
    foreach ($mapping in $Mappings) {
        $oldPrefix = $mapping.Old.TrimEnd('\', '/')
        if ($absolute.Equals($oldPrefix, [StringComparison]::OrdinalIgnoreCase)) {
            return $mapping.New
        }
        if ($absolute.StartsWith($oldPrefix + '\', [StringComparison]::OrdinalIgnoreCase)) {
            return Join-Path $mapping.New $absolute.Substring($oldPrefix.Length + 1)
        }
    }
    return $absolute
}

function Get-StackConfiguration {
    $master = Read-Config (Join-Path $repositoryPath 'configuration.json')
    $proxy = Read-Config (Join-Path $proxyPath 'config.json')
    $gui = Read-Config (Join-Path $iquipPath 'config.json')
    $mappings = @(
        @{ Old = (Resolve-ConfigPath $proxy.master_path $proxyPath); New = $repositoryPath }
        @{ Old = (Resolve-ConfigPath $master.common_path $repositoryPath); New = $lolencPath }
    )
    foreach ($name in @('device_config', 'device_db', 'log_path')) {
        $value = Rebase-ConfigPath $master.$name $repositoryPath $mappings
        Set-ConfigValue $master $name ($value.Replace('\', '/'))
    }
    Set-ConfigValue $master 'common_path' ($lolencPath.Replace('\', '/'))
    if (-not $master.master_port) { Set-ConfigValue $master 'master_port' '8082' }
    Assert-Path $master.device_config
    Assert-Path $master.device_db
    foreach ($name in @('xilinx_include_path', 'bsp_src_path', 'bsp_include_path', 'bsp_lib_path')) {
        Assert-Path (Resolve-ConfigPath $master.$name $lolencPath) 'Container'
    }
    foreach ($name in @('startup_path', 'linker_path')) {
        Assert-Path (Resolve-ConfigPath $master.$name $lolencPath)
    }
    if ($master.compile_driver -eq 'False') {
        Assert-Path (Join-Path (Resolve-ConfigPath $master.bsp_lib_path $lolencPath) 'lolenc_lib.a')
    }
    foreach ($name in @('result_path', 'repository_path')) {
        if ($proxy.$name -and [IO.Path]::IsPathRooted($proxy.$name)) {
            $value = Rebase-ConfigPath $proxy.$name $proxyPath $mappings
            Set-ConfigValue $proxy $name ($value.Replace('\', '/'))
        }
    }
    Set-ConfigValue $proxy 'master_path' ($repositoryPath.Replace('\', '/'))
    Set-ConfigValue $proxy 'device_db_path' $master.device_db
    Set-ConfigValue $proxy 'core_addr' $master.ip
    Set-ConfigValue $proxy 'master_addr' '::1'
    Set-ConfigValue $proxy 'notify_port' 3250
    Set-ConfigValue $proxy 'control_system' 'lolenc'
    Set-ConfigValue $gui.constant 'proxy_ip' '127.0.0.3'
    Set-ConfigValue $gui.constant 'proxy_port' $ProxyPort
    return [pscustomobject]@{ Master = $master; Proxy = $proxy; GUI = $gui }
}

function Write-Config($Config, [string]$Path) {
    # ASCII JSON also works with the master's readers that use the Windows code page.
    $json = $Config | ConvertTo-Json -Depth 64
    $json = [regex]::Replace($json, '[^\x00-\x7f]', {
        param($match)
        '\u{0:x4}' -f [int][char]$match.Value
    })
    [IO.File]::WriteAllText($Path, $json, (New-Object Text.UTF8Encoding($false)))
}

function Get-ListeningPorts {
    return @([Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().GetActiveTcpListeners() |
        Select-Object -ExpandProperty Port -Unique)
}

function Wait-Master {
    $deadline = [DateTime]::UtcNow.AddSeconds($StartupTimeoutSeconds)
    do {
        $ports = Get-ListeningPorts
        if (($ports -contains 3250) -and ($ports -contains 3251)) { return }
        Start-Sleep -Milliseconds 500
    } while ([DateTime]::UtcNow -lt $deadline)
    throw "Master did not open ports 3250/3251 within $StartupTimeoutSeconds seconds. Check the Master window."
}

function Wait-Proxy {
    $deadline = [DateTime]::UtcNow.AddSeconds($StartupTimeoutSeconds)
    do {
        try {
            $response = Invoke-WebRequest -Uri "http://127.0.0.3:$ProxyPort/openapi.json" `
                -UseBasicParsing -TimeoutSec 2 -Proxy $null
            if ($response.StatusCode -eq 200) { return }
        } catch { }
        Start-Sleep -Milliseconds 500
    } while ([DateTime]::UtcNow -lt $deadline)
    throw "Proxy was not ready within $StartupTimeoutSeconds seconds. Check its window and the board's Moninj connection on port 8081."
}

function ConvertTo-PowerShellLiteral([string]$Value) {
    return "'" + $Value.Replace("'", "''") + "'"
}

function Start-StackSession([string]$Name) {
    # Encode the command so spaces, apostrophes and non-ASCII paths survive process creation.
    $command = '& ' + (ConvertTo-PowerShellLiteral $launcherScript) +
        ' -Session ' + (ConvertTo-PowerShellLiteral $Name) +
        ' -Root ' + (ConvertTo-PowerShellLiteral $Root) +
        ' -EnvironmentName ' + (ConvertTo-PowerShellLiteral $EnvironmentName) +
        ' -EnvironmentPrefix ' + (ConvertTo-PowerShellLiteral $EnvironmentPrefix) +
        ' -CondaExe ' + (ConvertTo-PowerShellLiteral $CondaExe) +
        ' -ProxyPort ' + $ProxyPort
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    $powershellExe = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
    # These visible interactive windows are the three sessions requested by the user.
    Start-Process -FilePath $powershellExe -WindowStyle Normal -ArgumentList @(
        '-NoProfile', '-NoExit', '-ExecutionPolicy', 'Bypass', '-EncodedCommand', $encoded
    ) | Out-Null
}

if ($Session -ne 'Launch') {
    $Host.UI.RawUI.WindowTitle = "[$EnvironmentName] $Session"
    $hook = & $CondaExe shell.powershell hook
    if ($LASTEXITCODE -ne 0) { throw 'Could not initialize Conda in this session.' }
    Invoke-Expression ($hook -join "`n")
    conda activate $EnvironmentPrefix
    if ($env:CONDA_PREFIX -ne $EnvironmentPrefix) {
        throw "Could not activate '$EnvironmentName'."
    }
    switch ($Session) {
        'Master' {
            Set-Location -LiteralPath $repositoryPath
            lolenc_master --configuration $masterConfigPath --bind 127.0.0.3 --repository $repositoryPath
        }
        'Proxy' {
            Set-Location -LiteralPath $proxyPath
            $env:CONFIG_PATH = $proxyConfigPath
            python -m uvicorn main:app --host 0.0.0.0 --port $ProxyPort --loop asyncio
        }
        'GUI' {
            Set-Location -LiteralPath $iquipPath
            qiwis -c $guiConfigPath
        }
    }
    if ($LASTEXITCODE -ne 0) { throw "$Session exited with code $LASTEXITCODE. See the output above." }
    return
}

foreach ($directory in @($iquipPath, $repositoryPath, $proxyPath, $lolencPath)) {
    Assert-Path $directory 'Container'
}
Assert-Path (Join-Path $proxyPath 'main.py')
$CondaExe = Find-CondaExecutable
$EnvironmentPrefix = Get-EnvironmentPrefix $CondaExe
foreach ($relative in @('python.exe', 'Scripts\lolenc_master.exe', 'Scripts\qiwis.exe')) {
    Assert-Path (Join-Path $EnvironmentPrefix $relative)
}
$pythonExe = Join-Path $EnvironmentPrefix 'python.exe'
& $pythonExe -c "import fastapi, uvicorn, pydantic_settings, artiq.coredevice.comm_moninj; import lolenc.frontend.lolenc_master; import qiwis"
if ($LASTEXITCODE -ne 0) { throw "Required packages could not be imported in '$EnvironmentName'." }
if (-not (Get-Command aarch64-none-elf-g++ -ErrorAction SilentlyContinue)) {
    throw 'aarch64-none-elf-g++ was not found. Add the Arm GNU Toolchain bin directory to PATH.'
}
$launchPlan = Get-StackConfiguration
if ($launchPlan.Master.compile_driver -eq 'True' -and -not (Get-Command ar -ErrorAction SilentlyContinue)) {
    throw 'Driver rebuilding requires GNU ar. Add its bin directory to PATH.'
}
Write-Host "Root: $Root"
Write-Host "Conda: $CondaExe"
Write-Host "Environment: $EnvironmentName ($EnvironmentPrefix)"
Write-Host "Board: $($launchPlan.Master.ip)"
Write-Host "Master config: $masterConfigPath"
Write-Host "Proxy URL: http://127.0.0.3:$ProxyPort"
if ($CheckOnly) {
    Write-Host 'Preflight passed. No sessions or runtime configuration files were created.' -ForegroundColor Green
    return
}

$listeningPorts = Get-ListeningPorts
foreach ($port in @(3250, 3251, 1066, 1067, $ProxyPort)) {
    if ($listeningPorts -contains $port) {
        throw "Port $port is already in use. Close the existing service before launching the stack."
    }
}
[IO.Directory]::CreateDirectory($runtimePath) | Out-Null
[IO.Directory]::CreateDirectory($launchPlan.Master.log_path) | Out-Null
Write-Config $launchPlan.Master $masterConfigPath
Write-Config $launchPlan.Proxy $proxyConfigPath
Write-Config $launchPlan.GUI $guiConfigPath

Write-Host 'Starting Master...'
Start-StackSession 'Master'
Wait-Master
Write-Host 'Starting Proxy...'
Start-StackSession 'Proxy'
Wait-Proxy
Write-Host 'Starting iquip GUI...'
Start-StackSession 'GUI'
Write-Host 'Three sessions started. Use Ctrl+C in each server window to stop it.' -ForegroundColor Green
