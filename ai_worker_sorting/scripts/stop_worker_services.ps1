$ErrorActionPreference = 'Stop'
docker exec ros-humble-novnc touch /tmp/worker_sorting_bridge/worker.stop /tmp/worker_sorting_bridge/vision.stop
if ($LASTEXITCODE -ne 0) { throw 'Worker service stop request failed.' }
Write-Output 'Worker and vision service shutdown requested. Webcam pipeline is unaffected.'
