$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$header = Join-Path $root 'src\application\local\minio_uploader.hpp'
$source = Join-Path $root 'src\application\local\minio_uploader.cpp'
foreach ($file in @($header, $source)) {
    if (-not (Test-Path -LiteralPath $file)) { throw "Missing MinIO uploader file: $file" }
}

$h = Get-Content -Raw -Encoding UTF8 $header
$cpp = Get-Content -Raw -Encoding UTF8 $source
$system = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\system_manager.cpp')
$yoloH = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\yolo\h30t_yolo_service.hpp')
$yoloCpp = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\yolo\h30t_yolo_service.cpp')
$yaml = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\app_json\app_config.yaml')

foreach ($required in @('miniocpp/client.h', 'StaticProvider', 'PutObjectArgs', 'PutObject', 'image/jpeg', 'minio.access_key', 'minio.secret_key')) {
    if ($cpp -notmatch [regex]::Escape($required)) { throw "Uploader is missing required behavior: $required" }
}
if ($cpp -match 'AWS4-HMAC-SHA256|x-amz-content-sha256|minioadmin|minioadmin123') { throw 'Legacy signing or hard-coded credentials remain' }
foreach ($key in @('minio.addr:', 'minio.bucket:', 'minio.visible_folder:', 'minio.infrared_folder:', 'minio.jpeg_quality:')) {
    if ($yaml -notmatch [regex]::Escape($key)) { throw "Missing YAML setting: $key" }
}
foreach ($key in @('minio.access_key:', 'minio.secret_key:')) {
    if ($yaml -notmatch [regex]::Escape($key)) { throw "Missing YAML credential setting: $key" }
}
if ($h -notmatch 'MinioUploadResult' -or $h -notmatch 'Enqueue') { throw 'Uploader result/queue interface is missing' }
if ($yoloH -notmatch 'std::vector<uint8_t>' -or $yoloCpp -notmatch 'imencode') { throw 'YOLO result callback does not carry JPEG bytes' }
if ($system -notmatch 'fileUrl' -or $system -notmatch 'objectName') { throw 'Upload feedback metadata is missing' }
if ($system -notmatch 'result\.success') { throw 'Location publishing is not gated by upload success' }
Write-Output 'MinIO upload integration contract passed'
