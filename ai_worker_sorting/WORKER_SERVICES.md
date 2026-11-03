# ROS2 분류 작업 서비스

구현한 서버는 기본 `dry_run: true`, `mock_mode: false`다.
실제 AI WORKER에는 연결하지 않았다. 현재 컨테이너의 `ROS_DOMAIN_ID=77`에
기본 서버를 실행했다. Windows 웹캠 송신기는 종료된 상태라 인식 서비스에는
현재 새 탐지 입력이 없다. 서버가 좌표를 임의로 만들어 집기 요청을 수행하지 않는다.

## 인터페이스

| 서비스 | 타입 | 의미 |
|---|---|---|
| `/worker/sort_once` | `ai_worker_interfaces/srv/SortOnce` | 물건 종류로 단발 작업 접수 |
| `/worker/task_status` | `ai_worker_interfaces/srv/TaskStatus` | 작업 ID별 상태 조회 |
| `/worker/cancel_task` | `ai_worker_interfaces/srv/CancelTask` | 취소 요청 접수 |
| `/vision/detect_objects` | `ai_worker_interfaces/srv/DetectObjects` | 최신 탐지·가용한 3D 표면 좌표 반환 |

`/worker/status`는 작업 ID·종류·상태·단계를 담은 JSON `std_msgs/String` 이벤트다.
현재 상태의 기준은 `task_status` 서비스다. 이벤트를 놓쳐도 상태를 조회할 수 있다.

`accepted: true`는 접수 성공이다. 작업 성공은 `state: SUCCEEDED`로 확인한다.
실행 상태는 `QUEUED`, `RUNNING`, `SUCCEEDED`, `FAILED`, `CANCELED`다.
한 번에 한 작업만 접수한다. 미지원 종류·중복 작업은 접수 단계에서 거부한다.
작업 ID와 최근 100개 결과는 메모리에 저장하므로 재시작하거나 기록이 오래되면 사라진다.
요청 응답이 끊겼을 때 자동 재전송하는 클라이언트 로직은 포함하지 않았다.

## 처리 흐름

```text
SortOnce(target_class)
 → task_id 즉시 반환
 → /vision/detect_objects 호출
 → 클래스·신뢰도·관측 시각·좌표계·localized 검사
 → 작업 영역 안의 집기 후보 선택
 → 종류에 해당하는 바구니와 그리퍼 설정 선택
 → 열기 → 접근 → 내려가기 → 집기 → 들어 올리기 → 파지 확인
 → 해당 바구니로 이동 → 놓기 → 복귀
 → SUCCEEDED 또는 FAILED/CANCELED
```

작업 중 서비스 응답 스레드를 기다리게 하지 않는다. 별도 작업 스레드가 진행하며
상태 조회·취소 요청은 계속 처리한다. 작업에는 시간 제한이 있다.
취소 접수만으로 컨트롤러가 멈췄다고 응답하지 않는다. 모의 동작은 중단 후 `CANCELED`다.
실물 동작 도중 취소·오류로 컨트롤러 상태가 불확실하면 `FAILED`와 오류 잠금으로 처리한다.
상태 확인 후 서버를 재시작해야 새 실물 작업을 받을 수 있다.

## 모의 데모

기본 서버(도메인 77)와 충돌하지 않도록 데모는 도메인 78에서 실행한다.
첫 번째 PowerShell에서 컨테이너에 들어간다.

```powershell
docker exec -it -e ROS_DOMAIN_ID=78 ros-humble-novnc bash
```

컨테이너에서:

```bash
source /opt/ros/humble/setup.bash
source /tmp/worker_sorting_ws_20261103/install/setup.bash
ros2 launch ai_worker_sorting services.launch.py mock_mode:=true python_prefix:=/tmp/worker_sorting_yolo_venv/bin/python
```

두 번째 PowerShell에서도 **같은 도메인 78**로 컨테이너에 들어가고 위 두 `source` 명령을 실행한다.
그 뒤 다음을 호출한다.

```bash
ros2 service call /vision/detect_objects ai_worker_interfaces/srv/DetectObjects "{target_class: 'box'}"
ros2 service call /worker/sort_once ai_worker_interfaces/srv/SortOnce "{target_class: 'box'}"
```

응답의 `task_id`를 복사해 조회·취소한다.

```bash
ros2 service call /worker/task_status ai_worker_interfaces/srv/TaskStatus "{task_id: '반환된_ID'}"
ros2 service call /worker/cancel_task ai_worker_interfaces/srv/CancelTask "{task_id: '반환된_ID'}"
```

`bottle`도 지원하며 `box`와 다른 바구니를 사용한다.
데모 위치는 가상 값이며 YOLO가 실제 상자를 인식한 결과가 아니다.
기본 약 1.7초 모의 작업이라 취소 전에 이미 완료될 수 있다.
데모 실행 터미널의 Ctrl+C로 노드를 종료한다.

## 실제 탐지와 깊이 연결

`config/worker_services.yaml`에서 `mock_mode: false`를 유지한다.

1. YOLO의 `Detection2DArray` 토픽을 `detections_topic`으로 연결한다.
   `hypothesis.class_id`는 `box` 같은 클래스명을 사용한다.
2. RGB와 정렬된 깊이 Image, 대응 CameraInfo, 카메라→`base_link` TF를 연결한다.
3. 왜곡 보정·외부 좌표 보정·정렬 상태를 확인한 뒤 `enable_depth: true`, `calibrated: true`로 설정한다.
4. 실측 깊이 단위에 맞춰 `scale_16u`를 설정한다. 기본 16UC1은 0.001m/단위,
   32FC1은 m를 전제로 한다. 알려지지 않은 인코딩은 거부한다.
5. 인식 서비스가 반환한 표면 좌표를 실측·RViz와 비교한다.

영상 토픽을 계속 구독하고 최근 자료를 보관한다. 서비스 호출 시 탐지 촬영 시각에 맞는
깊이(기본 ±0.06초), 같은 optical frame과 크기의 CameraInfo를 선택한다.
TF도 탐지 촬영 시각으로 조회한다. TF가 없거나 관측이 오래되면 집기 좌표로 사용하지 않는다.

현재 깊이 추정은 탐지 박스 중앙 40% 영역의 유효 깊이 중앙값을 사용한다.
유효값 부족·큰 깊이 변화·작업 영역 밖 좌표를 거부한다. 고립된 물체와 중앙에
연속된 표면이 있다는 가정이다. 가림·구멍·겹친 물체는 분할 마스크나 별도 자세 추정이 필요하다.
rectified 단안 투영과 단위 R, stereo shift가 없는 P를 지원한다.

`DetectedObject.surface_pose`는 **보이는 표면의 위치**다. orientation은 물체 자세 추정값이 아니다.
`localized: false`인 탐지는 2D 확인용이며 작업 서버는 집기를 거부한다.
`simulated: true`인 탐지는 실제 제어 모드에서 거부한다.

## 종류별 분류와 실물 연결

`worker_services.yaml`의 `objects.box.*`, `objects.bottle.*`에 바구니 위치,
집기 높이 오프셋, 그리퍼 열기·닫기 값을 설정한다. 기본 자세는 위에서 집는 고정 TCP 방향이다.
물체별 6D 자세 추정이나 회전 대응은 구현하지 않았다.

현재 구현은 기존 `MoveItRobot` 연결부를 재사용한다. 실물 사용에는
AI WORKER의 실제 제어 API·컨트롤러 종류·TCP·그리퍼 단위·충돌 장면·파지 확인 서비스를
검증해야 한다. ROBOTIS의 MoveL/Cyclo 전용 어댑터는 아직 없다.
기본 서버가 실물 명령을 보내지는 않는다. 모의 동작의 파지 확인도 센서 검증이 아니다.

## 빌드와 파일

두 패키지는 ROS2 워크스페이스의 `src` 아래 **형제 디렉터리**로 둔다.

```text
src/
  ai_worker_interfaces/    # ament_cmake, msg/와 srv/ 생성
  ai_worker_sorting/       # ament_python, 서버와 제어 코드
```

```bash
colcon build --packages-select ai_worker_interfaces ai_worker_sorting --symlink-install
source install/setup.bash
```

서버 구현:

- `worker_service.py`: 비동기 작업 서비스, 클래스별 배치, 후보 검증.
- `vision_service.py`: 탐지 서비스, 모의 관측, RGB-D·TF 연결.
- `locator.py`: 깊이 표면 좌표와 강체 변환.
- `tasks.py`: 작업 기록, 동시 요청·취소·오류 잠금.
- `launch/services.launch.py`: 두 서버 실행. `mock_mode` 기본 false, `dry_run` 기본 true.

기존 Docker 환경은 Python 영상 라이브러리가 별도 venv에 있으므로 launch의
`python_prefix`를 위 예시처럼 지정한다. 코드를 바꾼 뒤 컨테이너에 복사·재빌드해야 한다.

## 현재 백그라운드 서버 종료

PowerShell, 프로젝트 폴더에서:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/stop_worker_services.ps1
```

작업·인식 서비스의 종료를 요청하며 웹캠 파이프라인은 별개다.
다시 실행할 때는 컨테이너의 `worker.stop`, `vision.stop` 파일을 지운다.

```powershell
docker exec ros-humble-novnc /tmp/worker_sorting_yolo_venv/bin/python -c "from pathlib import Path; [Path('/tmp/worker_sorting_bridge', n).unlink(missing_ok=True) for n in ['worker.stop', 'vision.stop']]"
```

이 중단 파일은 도메인과 무관한 같은 컨테이너 경로를 사용한다. 여러 데모를 동시에
운영하려면 각 설정 파일에서 서로 다른 `stop_file` 경로를 지정한다.

## 검증

- 단위 테스트 30개 통과: 기존 19개 + 작업 상태·동시성 6개 + 깊이 계산 5개.
- 실제 ROS2 서비스 호출 통합 검증 9개 시나리오 통과.
- 모의 box/bottle 분류, 중복 요청 거부, 취소, 서비스 시간 초과, 오래되거나 범위 밖의 관측 거부.
- 2D 전용 탐지와 모의 관측의 실물 제어 사용 거부.
- 합성 깊이 800mm + TF → 로봇 좌표 `(0.4, 0.0, 0.2)` → 모의 분류 완료.
- 무효 깊이 거부. 실제 깊이 카메라·AI WORKER 팔은 미검증.

통합 검증 결과: `results/services-smoke.json`.

```bash
ROS_DOMAIN_ID=78 /tmp/worker_sorting_yolo_venv/bin/python \
  /tmp/worker_sorting_ws_20261103/src/ai_worker_sorting/tests/ros_services_smoke.py \
  --output /tmp/worker_sorting_bridge/services-smoke.json
```

이 명령은 같은 도메인에 별도 작업 서버를 띄우지 않은 상태에서 실행한다.
