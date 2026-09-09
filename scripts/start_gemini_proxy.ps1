param(
    [string]$BindAddress = '127.0.0.1',
    [int]$Port = 8391
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$PythonExe = Join-Path $ProjectRoot '.venv-sim\Scripts\python.exe'

if (-not (Test-Path -LiteralPath $PythonExe)) {
    throw "Simulation Python was not found: $PythonExe"
}

$SecureKey = Read-Host 'Gemini API key (input is hidden)' -AsSecureString
$KeyPointer = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($SecureKey)
try {
    $env:GEMINI_API_KEY = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($KeyPointer)
} finally {
    [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($KeyPointer)
}

if ([string]::IsNullOrWhiteSpace($env:GEMINI_API_KEY)) {
    throw 'Gemini API key cannot be empty.'
}

$env:GEMINI_PROXY_URL = "http://${BindAddress}:$Port/v1/chat/completions"
$env:PYTHONPATH = $ProjectRoot

Write-Host "Gemini proxy URL: $env:GEMINI_PROXY_URL" -ForegroundColor Green
Write-Host 'Keep this window open. Press Ctrl+C to stop the proxy.'

try {
    Set-Location -LiteralPath $ProjectRoot
    & $PythonExe -m scripts.serve_gemini_proxy --host $BindAddress --port $Port
    if ($LASTEXITCODE -ne 0) {
        throw "Gemini proxy exited with code $LASTEXITCODE"
    }
} finally {
    Remove-Item Env:GEMINI_API_KEY -ErrorAction SilentlyContinue
}
