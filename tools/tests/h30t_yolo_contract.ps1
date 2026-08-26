$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path

function Assert-Contains([string]$Path, [string]$Pattern, [string]$Message) {
    $content = Get-Content -Raw -Encoding UTF8 (Join-Path $root $Path)
    if ($content -notmatch $Pattern) { throw $Message }
}

$required = @(
    'src/application/yolo/yolov8_detector.hpp',
    'src/application/yolo/yolov8_detector.cpp',
    'src/application/yolo/h30t_yolo_service.hpp',
    'src/application/yolo/h30t_yolo_service.cpp'
)
foreach ($file in $required) {
    if (-not (Test-Path (Join-Path $root $file))) { throw "Missing YOLO source: $file" }
}

Assert-Contains 'src/CMakeLists.txt' 'MODULE_YOLO_SRC' 'YOLO sources are not explicitly included by CMake.'
Assert-Contains 'src/application/h30t/h30t_liveview_session.cpp' 'rgb_frame_callback' 'Liveview does not forward RGB frames.'
Assert-Contains 'src/application/system_manager.cpp' 'psdk/h30t/detections' 'Detection MQTT topic is missing.'
Assert-Contains 'src/application/system_manager.cpp' 'yolo_->Start' 'YOLO worker is not started.'
Assert-Contains 'src/application/system_manager.cpp' 'yolo_->Stop' 'YOLO worker is not stopped.'

$yoloText = ($required | ForEach-Object { Get-Content -Raw -Encoding UTF8 (Join-Path $root $_) }) -join "`n"
if ($yoloText -match 'runtime_config\.hpp|h30t_stream_pipeline\.hpp|RtspStreamer') {
    throw 'YOLO sources still reference an obsolete path or class.'
}

Write-Host 'H30T YOLO static contract passed.'
