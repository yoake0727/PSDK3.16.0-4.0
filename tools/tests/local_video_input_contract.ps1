$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$source = Join-Path $root 'src\application\local\video_file_source.cpp'
$header = Join-Path $root 'src\application\local\video_file_source.hpp'
$system = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\system_manager.cpp')
$systemHeader = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\system_manager.hpp')
$yaml = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\app_json\app_config.yaml')
foreach ($file in @($source, $header)) { if (-not (Test-Path -LiteralPath $file)) { throw "Missing local video input file: $file" } }
$code = Get-Content -Raw -Encoding UTF8 $source
foreach ($required in @('cv::VideoCapture', 'video.input_mode', 'video.file_path', 'video.camera_source', 'SubmitRgbFrame', 'MonotonicMs')) {
    if (($code + $system) -notmatch [regex]::Escape($required)) { throw "Local video mode is missing: $required" }
}
foreach ($required in @('DjiLiveview_Init', 'DjiLiveview_DeInit', 'H30tRgbStreamPipeline', 'cv::cvtColor', 'video_input_')) {
    if (($system + $systemHeader) -notmatch [regex]::Escape($required)) { throw "SystemManager local video integration is missing: $required" }
}
foreach ($key in @('video.input_mode:', 'video.file_path:', 'video.camera_source:', 'video.loop:')) {
    if ($yaml -notmatch [regex]::Escape($key)) { throw "Missing video YAML key: $key" }
}
Write-Output 'Local video input integration contract passed'
