# Git 배포와 AI WORKER 설치

실물 제어를 실행하려면 [LIVE_ROBOT.md](ai_worker_sorting/LIVE_ROBOT.md)를 따른다.
이 문서의 기본 launch는 서버/모의 검증용이며, 실물용 launch는 RGB-D·YOLO·제어 연결을 함께 시작한다.

Git에는 두 ROS2 패키지의 소스·인터페이스·설정 예제만 배포한다.
로봇의 기존 구동 소프트웨어, 보정 파일, 드라이버, 모델 가중치는 별도로 유지한다.

## 1. 로봇 환경 확인

AI WORKER의 기존 ROS2 개발 컨테이너/터미널에서:

```bash
printenv ROS_DISTRO
python3 --version
command -v ros2
command -v colcon
```

현재 검증한 환경은 ROS2 Humble/Python 3.10이다. 로봇이 Jazzy 등 다른 배포판을
쓰면 해당 환경에서 빌드·모의 테스트를 다시 해야 한다. 로봇의 기존 배포판을 바꾸지 않는다.
ROBOTIS의 기존 bringup과 통신 설정을 사용한다.

주요 의존성은 `rclpy`, `rosidl_default_generators`, `vision_msgs`, `cv_bridge`,
`tf2_ros`, `message_filters`, NumPy/OpenCV다. 실물 MoveIt 연결에는
`moveit_msgs`, `control_msgs`, `shape_msgs`, `action_msgs`가 추가로 필요하다.
필요한 ROS 패키지는 각 `package.xml`에 선언되어 있다.

로봇의 기존 CUDA/PyTorch/JetPack을 유지한다. 이 저장소의 빌드 스크립트는
`apt upgrade`, `pip install`, 드라이버 설치, 공식 로봇 소스 수정을 수행하지 않는다.

## 2. 내려받기·빌드

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
git rev-parse HEAD
bash scripts/build_ros2.sh "$HOME/ros2_worker_ws"
source "$HOME/ros2_worker_ws/install/ros2_worker/setup.bash"
```

사용 중인 ROS2 환경을 먼저 source해야 한다. 소스는 Git 폴더에, 이 프로젝트의
빌드·설치 결과는 `~/ros2_worker_ws/{build,install,log}/ros2_worker`에 둔다.
`--symlink-install`을 사용하므로 실행 중에는 소스 폴더도 유지한다.
의존성 누락 시 빌드 로그에 나온 패키지를 기존 로봇 환경의 방식으로 준비한다.

개발 컨테이너가 재생성되면 사라지는 경로에 클론하지 말고, 로봇의 지속되는
작업 디렉터리 또는 기존 볼륨 마운트 안에 소스·설치 공간을 둔다.

## 3. 모의 테스트

먼저 실제 센서나 팔 연결 없이 서비스 흐름을 확인한다.

```bash
export ROS_DOMAIN_ID=79
ros2 launch ai_worker_sorting auto_sort.launch.py mock_mode:=true
```

OpenCV/NumPy가 별도 system-site-packages venv에 있다면:

```bash
ros2 launch ai_worker_sorting auto_sort.launch.py mock_mode:=true python_prefix:=/absolute/path/to/venv/bin/python
```

이 venv의 Python 버전은 ROS2의 Python 버전과 같아야 한다.
일반 Python venv를 새로 만들면서 로봇의 GPU 라이브러리를 교체하지 않는다.
모의 서비스 실행에는 YOLO 가중치가 필요하지 않다.

같은 환경·도메인의 다른 터미널에서:

```bash
source "$HOME/ros2_worker_ws/install/ros2_worker/setup.bash"
export ROS_DOMAIN_ID=79
ros2 service call /worker/prepare_auto_sort ai_worker_interfaces/srv/PrepareAutoSort '{}'
ros2 service call /worker/start_task ai_worker_interfaces/srv/StartTask "{session_id: '반환된_ID', max_objects: 0}"
ros2 service call /worker/task_status ai_worker_interfaces/srv/TaskStatus "{task_id: '반환된_작업_ID'}"
```

첫 터미널의 Ctrl+C로 종료한다. 79는 개발 테스트용 예시다. 실제 로봇 연결 때는
로봇의 ROS_DOMAIN_ID와 RMW/Zenoh 구성을 맞춰야 한다.

## 4. 실물 센서 연결용 설정

Git 예제를 직접 편집하는 대신 로봇 쪽 별도 설정 파일을 사용한다.

```bash
mkdir -p "$HOME/ros2_worker_config"
cp ai_worker_sorting/config/auto_sort.yaml "$HOME/ros2_worker_config/auto_sort.local.yaml"
```

다음 항목을 실제 장치에 맞춘다.

| 노드 | 설정 |
|---|---|
| `vision_service` | YOLO 탐지 토픽, RGB에 정렬된 깊이, CameraInfo, base frame |
| `zone_service` | 보정된 RGB 영상, 같은 깊이·CameraInfo, 테이프 색 |
| `worker_service` | 영상 크기에 맞는 집기 ROI, 물건 크기·높이·TCP, 팔·그리퍼 제어 정보 |

초기에는 `dry_run: true`, `mock_mode: false`, `hardware_configured: false`를 유지한다.
실제 TF와 RGB-D 보정을 확인한 후에만 두 인식 노드의 `enable_depth`, `calibrated`를 적용한다.
노드 기본값의 `/laptop_camera/*`는 개발 PC용이므로 실제 토픽으로 바꿔야 한다.
모델 가중치는 로봇의 별도 경로에 두고 직접 로컬 파일 경로로 지정한다.

```bash
ros2 launch ai_worker_sorting auto_sort.launch.py config:="$HOME/ros2_worker_config/auto_sort.local.yaml"
```

이 launch는 작업·인식·구역 서버만 시작한다. 실제 카메라 드라이버와 YOLO 탐지 노드는
별도 실행해야 한다. 이 저장소의 `ros_camera`는 Windows 웹캠용 HTTP 브리지이며,
AI WORKER의 USB/RGB-D 드라이버를 자동으로 시작하는 노드가 아니다.

실물 제어 전제와 현재 한계는 `ai_worker_sorting/AUTO_SORT.md`를 따른다.
이 저장소는 실제 AI WORKER에서의 설치·보정·팔 동작 검증을 완료하지 않았다.

## 5. 업데이트

관련 노드를 Ctrl+C로 종료한 후 소스 폴더에서:

```bash
git status --short
git pull --ff-only
bash scripts/build_ros2.sh "$HOME/ros2_worker_ws"
```

다시 새 터미널에서 기존 ROS2 환경과 이 프로젝트 overlay를 source하고 모의 테스트한다.
`msg`/`srv`가 바뀌면 같은 타입을 사용하는 PC·로봇 양쪽을 동일 커밋으로 맞추고
두 패키지를 함께 빌드해야 한다. 로봇 로컬 설정 파일은 Git 밖에 있어 업데이트로 덮어쓰지 않는다.

## 6. 배포에 포함하지 않는 것

- 웹캠 사진·영상·추론 결과와 실행 로그
- HTTP 브리지 인증 파일과 기타 자격증명
- YOLO/PyTorch 가중치와 TensorRT 엔진
- 로봇별 보정·네트워크·제어 로컬 설정
- 개발 PC의 assistant 설정과 캐시

테스트는 실행 후 결과 JSON을 생성한다. Git에 테스트 결과 파일 자체를 저장하지 않는다.
