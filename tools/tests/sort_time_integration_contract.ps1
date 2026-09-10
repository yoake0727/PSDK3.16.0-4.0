$root = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$tracker = Join-Path $root 'src\application\local\sort_tracker.cpp'
if (-not (Test-Path $tracker)) { throw 'SORT tracker is not implemented' }
$t = Get-Content -Raw -Encoding UTF8 $tracker
foreach ($token in @('KalmanFilter','Hungarian','hit_streak','time_since_update')) { if ($t -notmatch $token) { throw "Missing SORT token: $token" } }
$types = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\h30t\h30t_types.hpp')
if ($types -notmatch 'captureTimeMs' -or $types -notmatch 'frameId') { throw 'Frame metadata is not propagated' }
$loc = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'src\application\local\localizer.cpp')
if ($loc -notmatch 'findNearestTelemetry' -or $loc -notmatch 'tracker_.update') { throw 'Localizer lacks tracking or telemetry matching' }
Write-Output 'SORT and time matching contract passed'
