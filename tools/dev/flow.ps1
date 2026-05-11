[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Args
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$scriptPath = Join-Path $PSScriptRoot "flow.py"
if (-not (Test-Path $scriptPath)) {
    throw "flow.py not found at $scriptPath"
}

function Find-Python {
    $candidates = @(
        "python3",
        "python",
        "C:\msys64\ucrt64\bin\python.exe",
        "C:\msys64\ucrt64\bin\python3.exe",
        "C:\msys64\mingw64\bin\python.exe",
        "C:\msys64\mingw64\bin\python3.exe",
        "$env:USERPROFILE\miniconda3\python.exe",
        "$env:USERPROFILE\Miniconda3\python.exe",
        "$env:USERPROFILE\miniforge3\python.exe",
        "$env:USERPROFILE\Miniforge3\python.exe",
        "$env:USERPROFILE\mambaforge\python.exe",
        "$env:USERPROFILE\Mambaforge\python.exe"
    )

    foreach ($candidate in $candidates) {
        if ([string]::IsNullOrWhiteSpace($candidate)) {
            continue
        }
        if ($candidate -in @("python", "python3")) {
            $cmd = Get-Command $candidate -ErrorAction SilentlyContinue
            if ($cmd) {
                return $cmd.Source
            }
            continue
        }
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    return $null
}

$python = Find-Python
if (-not $python) {
    throw "python3/python not found in PATH or common MSYS2/Conda locations"
}

& $python $scriptPath @Args
exit $LASTEXITCODE
