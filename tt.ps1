[CmdletBinding(PositionalBinding = $false)]
param(
    [string]$Container = $(if ($env:TT_CONTAINER) { $env:TT_CONTAINER } else { "iic-osic-tools_xserver" }),
    [string]$HostBase = $(if ($env:TT_HOST_BASE) { $env:TT_HOST_BASE } else { (Split-Path -Parent $PSScriptRoot) }),
    [string]$ContainerBase = $(if ($env:TT_CONTAINER_BASE) { $env:TT_CONTAINER_BASE } else { "/foss/designs" }),
    [switch]$Interactive,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Cmd
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$runner = Join-Path $PSScriptRoot "tools\dev\run.ps1"
if (-not (Test-Path $runner)) {
    throw "runner not found at $runner"
}

& pwsh -NoProfile -ExecutionPolicy Bypass -File $runner `
    -Runner docker `
    -Container $Container `
    -HostBase $HostBase `
    -ContainerBase $ContainerBase `
    -Interactive:$Interactive `
    @Cmd

exit $LASTEXITCODE
