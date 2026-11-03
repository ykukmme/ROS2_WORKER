$ErrorActionPreference = 'Stop'
docker exec ros-humble-novnc touch /tmp/worker_sorting_bridge/auto-worker.stop /tmp/worker_sorting_bridge/auto-vision.stop /tmp/worker_sorting_bridge/zones.stop
if ($LASTEXITCODE -ne 0) { throw 'Automatic sorting shutdown request failed.' }
Write-Output 'Automatic sorting, vision and zone service shutdown requested.'
