$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path

function Read-Source([string]$path) {
    Get-Content -Raw -Encoding UTF8 (Join-Path $root $path)
}
function Assert-Match([string]$text, [string]$pattern, [string]$message) {
    if ($text -notmatch $pattern) { throw $message }
}

$serviceHeader = Read-Source 'src/application/yolo/h30t_yolo_service.hpp'
$serviceSource = Read-Source 'src/application/yolo/h30t_yolo_service.cpp'
$sessionSource = Read-Source 'src/application/h30t/h30t_liveview_session.cpp'
$controllerHeader = Read-Source 'src/application/h30t/h30t_stream_controller.hpp'
$managerSource = Read-Source 'src/application/system_manager.cpp'

Assert-Match $serviceHeader 'YoloAnnotatedFrameCallback' 'Missing annotated frame callback type.'
Assert-Match $serviceSource 'cv::rectangle' 'Detection boxes are not drawn.'
Assert-Match $serviceSource 'cv::putText' 'Detection labels are not drawn.'
Assert-Match $controllerHeader 'PushProcessedRgb' 'Missing processed RGB encoding entry.'
Assert-Match $managerSource 'PushProcessedRgb' 'Annotated frames are not connected to RTSP encoding.'
Assert-Match $sessionSource 'if \(rgb_frame_callback\)' 'Missing YOLO/raw fallback dispatch.'

Write-Host 'H30T YOLO overlay static contract passed.'
