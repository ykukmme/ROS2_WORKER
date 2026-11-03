# AI WORKER 물건 집기 프로토타입

ROS2 + YOLO로 고정 테이블 위 물건 하나를 찾아 한쪽 팔로 바구니에 놓는다.
기본값은 `dry_run: true`. 모의 모드에서는 팔·그리퍼 액션 클라이언트를 만들지 않는다.

실제 AI WORKER용 RGB-D·YOLO·MoveL 실행은 [LIVE_ROBOT.md](LIVE_ROBOT.md)를 따른다.

**두 종류를 자동으로 색 구역에 배정하는 기능은 [AUTO_SORT.md](AUTO_SORT.md)를 참고한다.**
물건 종류를 입력하지 않고 준비·시작 서비스를 호출한다. 현재 검증은 모의 실행이다.

**작업 시작·상태 조회·취소 서비스는 [WORKER_SERVICES.md](WORKER_SERVICES.md)를 참고한다.**
별도 `ai_worker_interfaces` 패키지와 RGB-D 좌표 계산부를 추가했다.

**노트북 웹캠의 Docker ROS2 실시간 인식은 [LAPTOP_ROS2.md](LAPTOP_ROS2.md)를 참고한다.**
현재 영상 수신·YOLO 탐지·결과 표시까지 실제 연결했고, 실물 로봇 제어는 연결하지 않았다.

## 구현 범위

- 중앙 카메라의 YOLO 탐지 → 알려진 물체 상단 평면과 광선 교차 → TF로 로봇 기준 좌표 계산.
- 단발 작업 시작, 신선한 관측만 사용, 목표/접근/배치/복귀 위치의 작업 범위 검사.
- 집기 → 들어 올리기 → 외부 센서 기반 집기 확인 → 배치 → 복귀.
- MoveIt `MoveGroup` 및 `control_msgs/GripperCommand` 액션 연결부.
- 실패·중단 시 후속 동작 중단, 액션 취소 요청, 오류 잠금. 자동 복구·자동 재시도 없음.

**아직 없는 기능:** 깊이 카메라 처리, 손목 영상 위치 보정, 물건 회전 추정,
여러 종류 분류, 양팔 협업, 충돌 장면 자동 생성, 잡은 물체의 충돌 모델 부착,
센서 기반 집기 검증 노드. 실제 AI WORKER 모델별 구동 설정도 포함하지 않는다.
물건 하나/바구니 하나/고정 자세가 첫 구현 범위다.

## 1. ROS 없이 흐름 확인

Python 3.10 이상, 표준 라이브러리만 필요하다. 이 디렉터리에서:

```powershell
python -m ai_worker_sorting.demo
python -m unittest discover -s tests -v
```

좌표는 가상 값이다. `VERIFY_GRASP`도 모의 성공이며 실물 검증이 아니다.

## 2. 로봇 환경 확인

로봇의 공식 구동 환경에서 다음을 확인한다. 새 OS/드라이버를 임의로 설치하지 않는다.

```bash
printenv ROS_DISTRO
ros2 topic list -t
ros2 action list -t
ros2 control list_controllers
```

필요 정보:

| 항목 | 확인할 것 |
|---|---|
| 모델과 버전 | 명판, 설치된 ROBOTIS 소프트웨어 버전 |
| 중앙 카메라 | rectified Image, CameraInfo 토픽과 optical frame |
| TF | base frame에서 카메라 optical frame까지 촬영 시각의 변환 |
| 팔 | MoveIt planning group, TCP tool link, MoveGroup action |
| 그리퍼 | GripperCommand action 지원 여부, 열기/닫기 값, 힘 단위와 한도 |
| 장면 | 테이블, 바구니, 다른 팔과 주변 장애물의 충돌 모델 |

그리퍼가 `FollowJointTrajectory` 등 다른 인터페이스를 쓰면
`robot_interface.py`의 `gripper()`를 해당 컨트롤러에 맞춰 교체해야 한다.
토픽/액션 이름을 이 저장소의 예제 값으로 추측하지 않는다.

## 3. ROS2 패키지 설치

로봇과 맞는 Ubuntu/ROS2 환경에서 작업한다. 아래는 ROS2 Jazzy 예시다.
이 `ai_worker_sorting` 디렉터리를 `~/ros2_ws/src/ai_worker_sorting`에 복사한 뒤:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/ai_worker_sorting --ignore-src -r -y
colcon build --packages-select ai_worker_sorting --symlink-install
source install/setup.bash
```

YOLO용 `ultralytics`는 별도 Python 의존성이다. 로봇의 공식 컨테이너/호환 환경에
설치하고, Jetson의 기존 PyTorch/CUDA 조합을 유지한다. 패키지는 이를 자동 설치하지 않는다.
가중치는 로컬 파일 경로로 지정한다. 샘플 가중치나 데이터는 포함하지 않는다.
임의의 상자는 기본 COCO 모델로 인식되지 않을 수 있으므로 `box` 클래스를 학습한
가중치를 사용하거나 실제 모델에 있는 클래스로 `target_class`를 바꾼다.

## 4. 좌표 보정

1. 중앙 카메라의 내부 보정을 완료하고 **왜곡 보정된 영상**과 대응 CameraInfo를 준비한다.
2. 기준 마커/보정판 등으로 카메라 optical frame과 로봇 base 사이 외부 보정을 한다.
   움직이는 카메라는 로봇 관절 TF가 함께 갱신되어야 한다.
3. 상자를 똑바로 놓고 base frame에서 **상자 상단 높이**를 측정한다.
   `object_top_z`는 테이블 높이가 아니라 이 값이다.
4. TCP에서 실제 집기 지점까지의 오프셋과 그리퍼 자세를 측정한다.
   `grasp_offset`, `tool_quaternion`에 반영한다.
5. 테이블 여러 지점에서 출력 좌표와 실측 좌표를 비교한 후 `calibrated: true`로 설정한다.

현재 위치 추정은 탐지 사각형의 중심을 사용하므로 큰 원근 변화, 기울어진 물체,
겹친 물체에는 부정확할 수 있다. 단일 상자와 고정 높이 조건에서 먼저 검증한다.
입력은 단안 rectified 영상, 단위 회전 R, stereo shift가 없는 P를 전제로 한다.
깊이 정보나 임의 높이의 물체는 이 버전에서 처리하지 않는다.

## 5. 인식 + 모의 실행

`config/sorting.yaml`을 복사해 실제 토픽·프레임·가중치·보정값을 설정한다.
`dry_run: true`를 유지한다.

```bash
ros2 launch ai_worker_sorting sorting.launch.py config:=/absolute/path/sorting.yaml
```

다른 터미널에서:

```bash
ros2 topic echo /sorting/target
ros2 topic echo /sorting/status
```

물건을 놓고 좌표가 맞는지 확인한 뒤 한 사이클 실행:

```bash
ros2 service call /sorting/start std_srvs/srv/Trigger '{}'
```

| 인터페이스 | 형식 | 의미 |
|---|---|---|
| `/sorting/target` | geometry_msgs/PointStamped | 촬영 시각과 base frame을 가진 물체 상단 좌표 |
| `/sorting/status` | std_msgs/String | 단계 또는 오류 |
| `/sorting/start` | std_srvs/Trigger | 최신 목표로 한 사이클 시작 |
| `/sorting/stop` | std_srvs/Trigger | 취소 요청, 오류 잠금 |

오래된 영상/목표, 없는 TF, 잘못된 카메라 정보는 거부한다.
YOLO 처리 속도에 맞춰 카메라 FPS를 낮추는 편이 좋다. 시간 제한을 늘릴 때는
`max_image_age`와 `target_max_age`를 함께 검토한다. 로봇과 카메라의 시계가 맞아야 한다.

## 6. 실제 제어 연결

실제 모델의 공식 bringup과 MoveIt이 먼저 동작해야 한다.

- `planning_group`, `tool_link`, 두 액션 이름, 그리퍼 값·힘을 확인한다.
- 작업 범위, TCP 자세, 접근 높이, 바구니·복귀 좌표를 실측 값으로 교체한다.
- MoveIt의 planning scene에 테이블·바구니·반대쪽 팔·주변 장애물을 반영한다.
  운반 물체의 부피도 포함해야 한다. 이 패키지는 자동으로 장면을 만들지 않는다.
- 목표 pose 간 이동은 일반 MoveIt 경로다. 직선 하강/상승이나 TCP 자세 유지는
  경로 전체에 보장되지 않는다. 실제 환경에 맞게 Cartesian 경로 또는 경로 제약을 추가한다.
- 외부 `std_srvs/Trigger` 집기 확인 서비스를 만들고 `grasp_check_service`에 연결한다.
  `success: true`는 실제 센서가 물건 파지를 확인했을 때만 반환해야 한다.
  현재 저장소는 이 서비스를 제공하지 않으며 그리퍼의 `stalled`만으로 성공 처리하지 않는다.
- 시뮬레이터에서 검증한 뒤 `hardware_configured: true`, `dry_run: false`로 바꾼다.
  두 값은 시작 시 읽으며 실행 중 변경은 적용되지 않는다.

실물 연결은 아직 검증되지 않았다. Windows에서 기하 계산·작업 순서 테스트 14개,
Python 구문·패키지 설정 확인, 모의 데모를 통과했다.
기존 `ros-humble-novnc` 컨테이너의 ROS2 Humble/Python 3.10에서 colcon 빌드,
ROS2 통합 모의 검증, CPU YOLO 이미지 인식, 영상→YOLO→TF 투영→모의 집기 연동을
통과했다. 실제 카메라 보정과 MoveIt/그리퍼 제어는 아직 검증하지 않았다.

### 기존 Docker 컨테이너에서 다시 검증

현재 복사·빌드된 위치는 `/tmp/worker_sorting_ws_20261103`이다.
컨테이너를 삭제·재생성하면 없어지므로 원본은 호스트의 이 폴더에서 관리한다.
PowerShell에서:

```powershell
docker start ros-humble-novnc
docker exec ros-humble-novnc bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/worker_sorting_ws_20261103/install/setup.bash && python3 /tmp/worker_sorting_ws_20261103/src/ai_worker_sorting/tests/ros_smoke.py'
```

`ros_smoke.py`는 가짜 좌표를 ROS2 토픽으로 발행하고 시작/중단 서비스를 호출한다.
목표 없음·틀린 프레임·오래된 좌표 거부, 정상 모의 사이클 완료, 중단 잠금을 검증한다.
로봇 제어 액션에는 연결하지 않는다.

### YOLO 이미지 인식 검증 결과

2026-11-03, 같은 컨테이너에 별도 Python 가상환경을 만들었다.

| 항목 | 값 |
|---|---|
| Python 환경 | `/tmp/worker_sorting_yolo_venv` (`--system-site-packages`) |
| 라이브러리 | Ultralytics 8.4.199, torch 2.14.1+cpu, torchvision 0.29.2+cpu |
| 영상 호환성 | NumPy 1.26.4, OpenCV 4.11.0.86 |
| 테스트 입력 | Ultralytics 패키지에 포함된 `bus.jpg` |
| 가중치 | 공식 `yolo26n.pt`, `/tmp/worker_sorting_assets/yolo26n.pt` |
| 이미지 결과 | 신뢰도 0.5 이상 버스 1대·사람 4명 |
| ROS2 연동 | 가상 카메라/TF에서 좌표 생성, 모의 사이클 `DONE` |

가중치 출처:
`https://github.com/ultralytics/assets/releases/download/v8.4.0/yolo26n.pt`

로컬 `results/yolo_image/annotated.jpg`, `detections.json`에 탐지 결과를 저장했다.
`results/ros_vision_smoke.json`에는 통합 검증 결과, `results/python-packages.txt`에는
가상환경에 설치한 버전을 기록했다. 시스템에서 상속한 전체 ROS 의존성 잠금 파일은 아니다.

ROS2 검증에서 거리·카메라 보정값은 **전부 가상 값**이다. 버스 샘플 이미지를 사용한
소프트웨어 연결 검사이며 실제 물건의 좌표 정확도나 파지 성공을 입증하지 않는다.
기본 `config/sorting.yaml`의 실물 실행 설정은 바꾸지 않았다.

이미지 인식 재실행(PowerShell):

```powershell
docker exec ros-humble-novnc bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/worker_sorting_ws_20261103/install/setup.bash && YOLO_CONFIG_DIR=/tmp/worker_sorting_yolo_config /tmp/worker_sorting_yolo_venv/bin/python -m ai_worker_sorting.image_smoke --weights /tmp/worker_sorting_assets/yolo26n.pt --image /tmp/worker_sorting_assets/bus.jpg --output /tmp/worker_sorting_results/yolo_image --expect-class bus'
```

영상부터 모의 집기까지 재실행(별도 ROS_DOMAIN_ID=77):

```powershell
docker exec ros-humble-novnc bash -lc 'source /opt/ros/humble/setup.bash && source /tmp/worker_sorting_ws_20261103/install/setup.bash && ROS_DOMAIN_ID=77 YOLO_CONFIG_DIR=/tmp/worker_sorting_yolo_config /tmp/worker_sorting_yolo_venv/bin/python /tmp/worker_sorting_ws_20261103/src/ai_worker_sorting/tests/ros_vision_smoke.py --weights /tmp/worker_sorting_assets/yolo26n.pt --image /tmp/worker_sorting_assets/bus.jpg --output /tmp/worker_sorting_results/ros_vision_smoke.json'
```

환경을 다시 만들 때는 `python3.10-venv` 설치 후 아래처럼 구성한다.
CPU x86_64 테스트용이며 Jetson 실물 환경에 그대로 적용하지 않는다.

```bash
python3 -m venv --system-site-packages /tmp/worker_sorting_yolo_venv
/tmp/worker_sorting_yolo_venv/bin/python -m pip install --upgrade pip
/tmp/worker_sorting_yolo_venv/bin/python -m pip install 'torch==2.14.1+cpu' 'torchvision==0.29.2+cpu' --index-url https://download.pytorch.org/whl/cpu
/tmp/worker_sorting_yolo_venv/bin/python -m pip install 'ultralytics==8.4.199' 'numpy==1.26.4' 'opencv-python==4.11.0.86'
```

### 노트북 웹캠 실시간 인식

Windows의 카메라 0에서 실제 영상 수신과 사람 탐지를 확인했다.
웹캠은 Windows에서 직접 읽는다. 이 모드에는 Docker/ROS2/로봇 제어가 연결되지 않는다.
로컬 환경은 OpenCV 4.13.0, torch 2.12.1+cpu, Ultralytics 8.4.90이다.
기존 설치를 사용했으며 Windows에 추가 패키지를 설치하지 않았다.

PowerShell에서 프로젝트 디렉터리로 이동한 후 실행:

```powershell
cd 'C:\Users\kkm\Documents\New project\ai_worker_sorting'
python -m ai_worker_sorting.webcam --weights models/yolo26n.pt --camera 0 --output results/webcam-live
```

- `Q` / `Esc` / 창 닫기: 종료하고 웹캠 해제.
- `S`: 탐지 박스가 있는 현재 화면만 저장.
- 기본 실행은 영상을 녹화하거나 프레임을 자동 저장하지 않는다.
- 종료 시 `results/webcam-live/summary.json`에 처리량과 탐지 결과를 기록한다.
- 카메라 요청 해상도는 장치에 따라 적용되지 않을 수 있다.
- 기본 COCO 모델이므로 사람·컵·병 등 학습된 클래스만 인식한다. 상자 인식은 별도 학습이 필요하다.

현재 백그라운드로 실행한 인식 창은 `results/webcam-live/stop.request` 파일로도 종료할 수 있다.

```powershell
New-Item -ItemType File -Force 'results/webcam-live/stop.request'
```

같은 `--stop-file` 경로로 다시 실행하려면 기존 중단 파일을 먼저 제거한다.
가중치는 로컬 `models/yolo26n.pt`에 복사했으며 Git에서 제외했다.
웹캠 로그·수동 저장 사진도 Git에서 제외했다.

다음 실물 단계는 노트북 영상의 ROS2 전달 또는 로봇 중앙 카메라 영상 수신,
잡을 물건의 YOLO 학습/인식, 카메라↔로봇 좌표 보정 순서다.

`/sorting/stop`은 소프트웨어 취소 요청이며 물리 비상정지를 대신하지 않는다.
취소·통신 실패 후에는 로봇 상태를 직접 확인한다. 오류 후 노드를 재시작해야 다음 작업이 가능하다.
프로세스를 재시작하는 것만으로 기존 컨트롤러의 동작이 멈췄다고 판단하면 안 된다.

## 코드 위치

```text
ai_worker_sorting/
  core.py              기하 계산, 작업 순서, 모의 로봇
  perception.py        YOLO, CameraInfo, 촬영 시각 TF, 평면 투영
  task_manager.py      시작/중단, 관측 유효성, 오류 잠금
  robot_interface.py   MoveGroup/GripperCommand, 결과·시간 초과 검사
  demo.py              ROS 없는 모의 사이클
  image_smoke.py       로컬 이미지 YOLO 탐지·표시·JSON 저장
  webcam.py            노트북 웹캠 실시간 YOLO, Q 종료·S 저장
```

공식 참고:
- https://github.com/ROBOTIS-GIT/ai_worker
- https://docs.robotis.com/docs/systems/aiworker/resources/open_source/
- https://docs.ultralytics.com/modes/predict/
- https://github.com/moveit/moveit_msgs/blob/ros2/action/MoveGroup.action
