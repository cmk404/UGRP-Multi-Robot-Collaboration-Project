param(
    [ValidateSet('rule', 'llm_peer_comm')]
    [string]$Condition = 'llm_peer_comm',
    [string]$Model = 'gemini-3.8-flash',
    [switch]$CheckOnly
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$PythonExe = Join-Path $ProjectRoot '.venv-sim\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw 'Missing .venv-sim. Install Python and requirements-sim.txt first.'
}
$env:PYTHONUTF8 = '1'
Set-Location -LiteralPath $ProjectRoot
$RunArgs = @('-m', 'scripts.run_three_robot', '--condition', $Condition, '--model', $Model)
if ($CheckOnly) { $RunArgs += '--check-only' }
& $PythonExe @RunArgs
exit $LASTEXITCODE
