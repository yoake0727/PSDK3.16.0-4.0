$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$header = Join-Path $root 'src\application\local\positioning.hpp'
$source = Join-Path $root 'src\application\local\positioning.cpp'
if (-not (Test-Path $header) -or -not (Test-Path $source)) {
    throw '定位 C++ 接口尚未实现'
}
$h = Get-Content -Raw -Encoding UTF8 $header
$s = Get-Content -Raw -Encoding UTF8 $source
foreach ($name in @('computeGimbalPosition', 'pixelToGpsWithYaw')) {
    if ($h -notmatch $name -or $s -notmatch $name) { throw "缺少函数: $name" }
}
foreach ($token in @('6378137.0', 'focalLengthMm / pixelSizeMm', 'deltaNorth', 'deltaEast')) {
    if ($s -notmatch [regex]::Escape($token)) { throw "缺少定位公式关键项: $token" }
}
Write-Output '定位 C++ 契约测试通过'
