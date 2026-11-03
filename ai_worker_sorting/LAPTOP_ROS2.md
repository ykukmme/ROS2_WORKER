# 노트북 웹캠 → Docker ROS2 → YOLO

현재 실제 웹캠으로 연결·검증했고, 인식 창과 ROS2 노드를 실행 중이다.
실제 로봇 제어와는 연결하지 않는다.

```text
Windows webcam_bridge (카메라 0, 640×360)
  → 인증된 JPEG 연결 (127.0.0.1:8765)
  → Docker HttpCamera → /laptop_camera/image_raw
  → Docker YoloDetector → /laptop_camera/detections
                       → /laptop_camera/image_annotated
  → Windows 인식 결과 창
```

웹캠은 Windows 프로그램 하나만 연다. YOLO 추론은 Docker의 CPU 환경에서 실행한다.
Docker는 `host.docker.internal:8765`로 Windows의 loopback 서버에 접근한다.
추가 포트 공개나 방화벽 설정 변경은 하지 않았다. 토큰은 URL이나 로그에 기록하지 않는다.

## ROS2 인터페이스

| 이름 | 타입 | 내용 |
|---|---|---|
| `/laptop_camera/image_raw` | `sensor_msgs/msg/Image` | bgr8 원본 영상 |
| `/laptop_camera/detections` | `vision_msgs/msg/Detection2DArray` | 클래스명·신뢰도·픽셀 박스 |
| `/laptop_camera/image_annotated` | `sensor_msgs/msg/Image` | YOLO 박스 표시 영상 |

- `ROS_DOMAIN_ID=77`로 기존 작업과 분리했다.
- 이미지 QoS는 Reliable / Keep last 1이다. 처리 지연 시 오래된 프레임을 쌓지 않는다.
- 프레임 이름은 `laptop_camera_optical_frame`이다.
- 영상과 탐지 결과는 Windows 촬영 시각을 그대로 공유한다. 호스트·컨테이너 시계가 맞아야 한다.
- `class_id`는 이 구현에서 `person`, `cell phone` 같은 클래스명이다.
- 검증하지 않은 CameraInfo/TF/3D 좌표는 발행하지 않는다. `/sorting/target`도 발행하지 않는다.
- 창은 ROS2에서 되돌아온 영상을 표시한다. 2초간 결과가 없으면 원본과 대기 문구를 표시한다.

## 현재 환경에서 확인

PowerShell:

```powershell
docker exec -e ROS_DOMAIN_ID=77 ros-humble-novnc bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list -t'
docker exec -e ROS_DOMAIN_ID=77 ros-humble-novnc bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic echo /laptop_camera/detections --once'
```

컨테이너 터미널에서 직접 실행할 때도 `export ROS_DOMAIN_ID=77`이 필요하다.
`rqt_image_view`가 설치되어 있다면 이 도메인에서 실행하고
`/laptop_camera/image_raw` 또는 `/laptop_camera/image_annotated`를 선택한다.

인식 창에서 `S`를 누르면 현재 표시 영상을 `results/webcam-ros2`에 저장한다.
평소에는 프레임/영상을 디스크에 저장하지 않는다.

## 종료

창의 `Q`/`Esc`/닫기는 웹캠과 Windows 송신기를 종료한다.
**ROS2 노드까지 함께 종료하려면** 프로젝트 디렉터리에서:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/stop_ros2_webcam.ps1
```

중단 파일로 정상 종료를 요청하며 프로세스를 강제 종료하지 않는다.

## 재실행

현재 컨테이너의 `/tmp` 환경과 호스트 `.runtime/bridge.token`을 사용하는 방법이다.
컨테이너 재생성 시에는 README의 Python 환경과 가중치를 먼저 복원해야 한다.
아래 명령은 기존 프로세스를 종료한 상태에서 실행한다.

1. 이전 중단 표시를 제거한다.

```powershell
cd 'C:\Users\kkm\Documents\New project\ai_worker_sorting'
Remove-Item -LiteralPath '.runtime/bridge.stop' -ErrorAction SilentlyContinue
docker exec ros-humble-novnc /tmp/worker_sorting_yolo_venv/bin/python -c "from pathlib import Path; Path('/tmp/worker_sorting_bridge/ros.stop').unlink(missing_ok=True)"
```

2. 첫 번째 PowerShell에서 Windows 송신기와 미리보기 창을 연다.

```powershell
python -m ai_worker_sorting.webcam_bridge --token-file .runtime/bridge.token --stop-file .runtime/bridge.stop --output results/webcam-ros2
```

3. 두 번째 PowerShell에서 컨테이너의 ROS2 노드를 시작한다.

```powershell
docker exec -d -e ROS_DOMAIN_ID=77 -e YOLO_CONFIG_DIR=/tmp/worker_sorting_yolo_config ros-humble-novnc bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/worker_sorting_ws_20261103/install/setup.bash && exec /tmp/worker_sorting_yolo_venv/bin/python -u -m ai_worker_sorting.ros_camera --ros-args --params-file /tmp/worker_sorting_ws_20261103/src/ai_worker_sorting/config/laptop_camera.yaml > /tmp/worker_sorting_bridge/ros_camera.log 2>&1'
```

설정은 `config/laptop_camera.yaml`이다. 모델 클래스와 confidence는 이 파일에서 바꾼다.
현재는 기본 COCO 가중치이며 상자 전용 학습은 하지 않았다.
인식 모드에는 카메라 내부/외부 보정이 필요하지 않지만 실물 좌표 계산에는 필요하다.

## 검증 기록

2026-11-03:

- 실제 웹캠의 ROS2 영상·탐지·표시 토픽 수신 확인.
- 8초 동안 원본 71개, 표시 영상 69개, 탐지 메시지 69개 수신.
- 69개의 공통 촬영 시각 확인. 사람·휴대폰 탐지.
- Windows 창에 컨테이너의 탐지 결과가 돌아오는 것 확인.
- 단위 테스트 19개 통과: 기하·작업 순서 14개, 브리지 인증·전송 5개.
- 패키지 colcon 빌드 통과.

결과는 `results/webcam-ros2/ros2-probe.json`에 저장했다.
이 결과는 실시간 영상 연결과 2D 탐지를 검증하며 거리 추정이나 집기 성공을 의미하지 않는다.

## 코드

- `ai_worker_sorting/webcam_bridge.py`: Windows 카메라, 인증된 JPEG 서버, 결과 표시.
- `ai_worker_sorting/ros_camera.py`: HTTP 수신·영상 발행, YOLO 노드·탐지 발행.
- `tests/ros_camera_probe.py`: 실제 ROS2 토픽과 시각 일치 검사.
- `.runtime/bridge.token`: 호스트 인증 파일, Git에서 제외.
- `/tmp/worker_sorting_bridge/token`: 컨테이너의 같은 인증 파일.

Docker 호스트 연결 참고: https://docs.docker.com/desktop/features/networking/
