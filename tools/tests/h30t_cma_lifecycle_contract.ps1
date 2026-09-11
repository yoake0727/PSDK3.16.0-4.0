param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
)

$ErrorActionPreference = 'Stop'
$pipeline = Get-Content -Raw (Join-Path $Root 'src/application/h30t/h30t_rgb_stream_pipeline.cpp')
$session = Get-Content -Raw (Join-Path $Root 'src/application/h30t/h30t_liveview_session.cpp')

function Assert-Contract([bool]$Condition, [string]$Message) {
    if (-not $Condition) { throw "FAIL: $Message" }
    Write-Host "PASS: $Message"
}

Assert-Contract ($pipeline -match 'DjiLiveview_UnregEncoderCallback\(\)') 'encoder callback is unregistered on stop'
Assert-Contract ($pipeline -match 'active = NULL') 'active pipeline pointer is cleared'
Assert-Contract ($pipeline -match 'impl_->running\.load\(\) \|\| impl_->worker\.joinable\(\)') 'duplicate pipeline start is rejected'
Assert-Contract ($pipeline -match 'if \(!active->opened\)') 'publisher state is checked before write'
Assert-Contract ($session -match 'DjiLiveview_StopImageStream[\s\S]*infrared->Stop\(\)[\s\S]*DjiLiveview_Deinit[\s\S]*DjiCameraManager_DeInit') 'stop order releases stream pipeline liveview camera manager'
Assert-Contract (($session.IndexOf('returnCode', [System.StringComparison]::Ordinal) -lt 0)) 'start failure path has no undefined returnCode'

Write-Host 'H30T CMA lifecycle contract: PASS'
