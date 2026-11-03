$ErrorActionPreference = 'Stop'
$sortingRoot = Split-Path -Parent $PSScriptRoot
$runtimePath = Join-Path $sortingRoot '.runtime'
New-Item -ItemType Directory -Force -Path $runtimePath | Out-Null
New-Item -ItemType File -Force -Path (Join-Path $runtimePath 'bridge.stop') | Out-Null
docker exec ros-humble-novnc touch /tmp/worker_sorting_bridge/ros.stop
if ($LASTEXITCODE -ne 0) { throw 'Webcam stop requested; ROS2 container stop request failed.' }
Write-Output 'Webcam and ROS2 stop requested.'
