$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$files = @('src\application\local\localizer.hpp','src\application\local\localizer.cpp','src\application\system_manager.cpp','src\application\mqtt\telemetry_pos_publisher.hpp') | ForEach-Object { Join-Path $root $_ }
foreach ($f in $files) { if (-not (Test-Path $f)) { throw "Missing integration file: $f" } }
$s = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\system_manager.cpp')
if ($s -notmatch 'psdk/h30t/locations' -or $s -notmatch 'processDetectionJson') { throw 'SystemManager does not publish locations' }
$h = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\mqtt\telemetry_pos_publisher.hpp')
if ($h -notmatch 'setLocalizer') { throw 'TelemetryPosPublisher is not connected to Localizer' }
$yaml = Join-Path $root 'src\app_json\app_config.yaml'
if (-not (Test-Path $yaml)) { throw 'Missing localizer YAML in app_json' }
$cpp = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\local\localizer.cpp')
foreach ($literal in @('24.0', '0.012', '0.562439', '-0.504344', '0.250245')) {
    if ($cpp -match [regex]::Escape($literal)) { throw "Hard-coded positioning value remains in localizer.cpp: $literal" }
}
Write-Output 'Localizer integration contract passed'
