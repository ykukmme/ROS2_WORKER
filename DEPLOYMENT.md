# 연결 PC에서 Git으로 설치하고 실행하기

AI WORKER에 연결한 **Ubuntu/ROS2 PC**에서 이 저장소를 내려받아 실행하는 절차다.
로봇의 공식 구동 환경과 같은 ROS2 배포판·호환되는 인터페이스를 사용한다.
개발 검증은 Humble에서 했으며, Jazzy 장비는 해당 환경에서 재빌드·검증해야 한다.

| 실행 위치 | 담당 |
|---|---|
| AI WORKER | 기존 bringup, Cyclo MoveL 제어기, RGB-D 드라이버, 관절 상태·TF 발행 |
| 연결 PC | 이 저장소 빌드, RGB-D·YOLO 인식, 테이프 구역 등록, 분류 작업 서버 |
| 연결 PC의 두 번째 터미널 | 준비·시작·상태·취소 명령 |

로봇에서는 공식 제어기를 실행하고 이 프로그램은 한 곳에서만 실행한다.
PC에서 로봇 USB 장치를 직접 열지 않고 로봇의 ROS2 영상·상태 토픽을 구독한다.
기본 속도는 이동 2cm/s·회전 0.15rad/s 기준, 각 이동 최소 요청 시간은 5초다.
실제 순간 속도는 Cyclo 보간·관절 제한에 따라 달라진다.

## 1. 로봇 쪽 준비

장비의 기존 절차대로 bringup, **MoveL 모드 Cyclo**, 카메라 드라이버를 실행한다.
설치된 모델·컨트롤러 버전을 사용한다. 상세 전제는 [실물 실행 안내](ai_worker_sorting/LIVE_ROBOT.md)를 따른다.

기본 연결은 다음과 같다. 실제 이름이 다르면 설정 생성 단계의 옵션으로 지정한다.

```text
/l_goal_move                                  robotis_interfaces/msg/MoveL
/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory
/joint_states                                sensor_msgs/msg/JointState
/camera/color/image_raw                       sensor_msgs/msg/Image
/camera/aligned_depth_to_color/image_raw       sensor_msgs/msg/Image
/camera/color/camera_info                      sensor_msgs/msg/CameraInfo
/tf, /tf_static
/ai_worker_movel_controller/get_parameters
```

중앙 카메라의 깊이는 RGB 영상에 정렬돼 있어야 한다. 카메라→작업 기준 좌표계와
실제 제어 링크의 TF도 있어야 한다. 카메라 외부 보정 자체는 이 프로그램이 수행하지 않는다.

## 2. PC에서 ROS2 연결 확인

PC와 로봇을 같은 네트워크에 연결하고 시간을 동기화한다. 수백 ms 이상의 시계 차이는
센서가 오래된 것으로 판정될 수 있다. 로봇의 ROS2 배포판, `ROS_DOMAIN_ID`, RMW 및
DDS/Zenoh 설정을 확인한 뒤 **PC의 모든 실행 터미널에 동일하게 적용**한다.

PC에서 설치된 ROS2와 필요한 ROBOTIS 인터페이스 overlay를 source한다.
아래 배포판 이름은 예시이며 로봇과 맞춘다.

```bash
source /opt/ros/humble/setup.bash
# ROBOTIS 인터페이스를 별도 workspace에서 빌드했다면 그 install/setup.bash도 source
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION
ros2 node list
ros2 topic list -t
ros2 topic hz /joint_states
```

`topic hz` 출력이 확인되면 Ctrl+C로 종료한다. **SSH/IP 연결만으로 ROS2 토픽이 연결되는 것은 아니다.**
DDS를 쓴다면 로컬호스트 전용 설정을 해제하고, 방화벽·인터페이스·멀티캐스트/피어 설정을 맞춘다.
Zenoh를 쓴다면 장비의 기존 router/endpoint 구성을 따른다. 다른 터미널의 오래된 discovery가
보이면 `ros2 daemon stop` 후 현재 환경에서 다시 확인한다.

Windows PC는 Ubuntu 실행 환경이나 로봇의 기존 Linux 개발 컨테이너에 SSH로 들어가
아래 절차를 실행한다. 후자는 인식·분류 노드도 로봇에서 실행된다. Windows Docker/WSL의
기본 NAT 환경에서 로봇 DDS가 자동 검색된다고 가정하지 않는다. 원격 영상 전송이 불안정하면
로봇의 기존 컨테이너에서 실행하고 PC는 SSH 터미널로 사용하는 방식이 간단하다.

## 3. Git 내려받기와 의존성

이하 명령은 선택한 실행 환경의 Bash 터미널에서 실행한다. 컨테이너라면 재생성해도
유지되는 볼륨에 소스·빌드 결과·설정을 둔다.

```bash
git clone --branch codex/initial-release https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
git rev-parse HEAD
```

실행 Python에 ROS2, NumPy, OpenCV, PyYAML, Ultralytics 및 호환 PyTorch가 필요하다.
PC의 GPU 환경에 맞는 기존 Python을 사용한다. CPU 실행도 가능하며 설정 생성 시 `--device cpu`를 준다.
YOLO 가중치는 PC의 별도 로컬 경로에 준비한다. 로봇에만 있는 가중치 경로를 PC에 그대로 지정하지 않는다.

ROS 패키지가 빠져 있다면 `package.xml` 기준으로 설치한다. `robotis_interfaces`는 로봇과 같은
버전을 준비해야 하며 일반 ROS 패키지 저장소에서 제공되지 않을 수 있다. 없다면 공식
[robotis_interfaces](https://github.com/ROBOTIS-GIT/robotis_interfaces)를 별도 workspace에서 빌드하고
source한 뒤 진행한다. 개발 검증 기준은 `c79afb6aa8d0900401ec38ecdb60b1a104929997`이다.
PC에는 Cyclo 제어기 실행 파일이 없어도 되지만 **MoveL 메시지 타입은 필요**하다.

기존 Python 환경을 선택하려면 매 터미널에서 아래 변수를 사용한다.

```bash
# 별도 환경이 필요할 때만 지정: ROS2와 동일한 Python 버전이어야 함
export WORKER_PYTHON=/absolute/path/to/existing/venv/bin/python
```

기본값은 `python3`이다. 빌드 스크립트는 패키지 설치·드라이버 교체·OS 업그레이드를 수행하지 않는다.

```bash
bash scripts/worker.sh check
bash scripts/worker.sh build
```

`check`는 라이브러리 import와 인터페이스 설치만 확인한다. 로봇으로 명령을 보내지 않는다.
빌드 결과는 기본 `~/ros2_worker_ws`에 둔다. 다른 경로는 `WORKER_WS`로 지정한다.
이후 wrapper가 해당 overlay를 자동 source한다. `--symlink-install`이므로 Git 소스 폴더를 유지한다.

## 4. 현장 설정 생성

**물건 크기·높이·그리퍼 길이를 숫자로 입력하지 않는 방법이다.**
카메라 스캔 후, 기존 조그 조작으로 그리퍼 기준 위치와 두 종류의 집기 자세를 한 번씩 지정한다.
이 도구는 팔·그리퍼 명령을 보내지 않는다. 자동 분류 worker는 종료한 상태여야 한다.
작업 범위 `WORKSPACE`와 카메라↔로봇 좌표 보정은 여전히 필요하다.

```bash
export WORKER_CONFIG="$HOME/ros2_worker_config/live.local.yaml"
STATE="$HOME/ros2_worker_config/teaching.local.yaml"
WORKSPACE='xmin,xmax,ymin,ymax,zmin,zmax'
WEIGHTS='/absolute/path/on/this/pc/best.pt'
# 첫 터미널: 인식·구역 노드만 실행. 카메라 드라이버는 로봇에서 실행 중이어야 함.
bash scripts/worker.sh calibrate weights:="$WEIGHTS" device:=cpu
```

두 번째 터미널에서도 같은 ROS2 환경·위 변수들을 설정한다. 수평인 같은 테이블에
빈 빨강·파랑 구역을 놓고, 팔을 치운 상태에서 두 종류의 물건 전체가 겹치지 않게 보이도록 한다.
각 종류에 속한 물건 중 가장 큰 것도 함께 보여준다.

```bash
bash scripts/worker.sh teach scan --state "$STATE" \
  --weights "$WEIGHTS" --workspace="$WORKSPACE" --device cpu
```

출력된 YOLO 종류 이름 두 개를 확인한다. 이제 물건·테이블·카메라는 움직이지 않는다.
로봇의 기존 조그 기능으로 위에서 집는 방향을 정한 뒤, 빈 테이블 위에 그리퍼를 둔다.
**작업 중 가장 아래로 내려오는 그리퍼 부분을 테이블 높이에 맞추고** 아래 명령으로 기록한다.
개폐에 따라 최저점이 달라지면 각각 같은 방향으로 맞춰 `tool`을 다시 기록한다. 가장 큰 길이를 보존한다.

```bash
bash scripts/worker.sh teach tool --state "$STATE" --tips-at-table
```

같은 방향을 유지하며 첫 종류의 물건을 실제로 집을 위치로 조그한다.
테이블과 닿지 않고 물건을 잡을 수 있는 자세인지 확인한 뒤 기록한다.
두 번째 종류도 반복한다. 물건은 특정 품목으로 제한하지 않고 첫 번째 종류·두 번째 종류로 안내한다.
현재 CLI의 아래 이름에는 `scan`에서 반환된 실제 인식 이름을 사용한다.

```bash
bash scripts/worker.sh teach grasp --state "$STATE" --class-name '첫_종류' --grasp-pose-confirmed
bash scripts/worker.sh teach grasp --state "$STATE" --class-name '둘째_종류' --grasp-pose-confirmed
# 두 종류 기록 후 팔을 작업물에서 떨어뜨린다. 집기 방향은 그대로 유지한다.
bash scripts/worker.sh teach write --state "$STATE" --output "$WORKER_CONFIG"
```

시작 안내는 **“팔을 작업물에서 떨어뜨린 뒤 시작하세요”**로 한다. 특정 고정 준비 자세는
요구하지 않는다. 물건·구역을 카메라에서 가리지 않고, 위로 이동할 공간이 있으며,
그리퍼에 물건을 들고 있지 않은 상태인지 확인한다. 그리퍼는 작업 시작 시 코드가 연다.

카메라 크기·높이에 여유값을 더하고, 테이블 기준 그리퍼 길이와 종류별 집기 오프셋을
자동 계산한다. 교시는 30분 안에 완료한다. 설정/교시 파일이 이미 있으면 새 경로로 시작한다.
탐지 영역 전체 깊이에서 최고점을 구한다. 깊이가 한 부분이라도 누락되거나 물건이 가려지거나
얇으면 설정을 거부한다. 보이지 않은 물건의 크기까지 추정하지 않는다.
작업 중 관측한 크기가 설정 범위를 넘으면 멈춘다. 종류·집는 방향·그리퍼가 바뀌면 다시 교시한다.

첫 터미널의 `calibrate`를 Ctrl+C로 종료한 뒤:

```bash
bash scripts/worker.sh doctor
```

`doctor`는 명령을 발행하지 않고 PC에서 실제 RGB·깊이·카메라 정보·관절·TF와 Cyclo
설정에 접근 가능한지 확인한다. PASS는 연결 진단이며 실제 좌표 정확도나 집기 성공 보장은 아니다.
카메라·프레임이 기본값과 다르면 `calibrate` launch의 해당 인자와 `teach scan`의
`--color-topic`, `--depth-topic`, `--camera-info-topic`, `--base-frame`을 같은 값으로 지정한다.
이미 정확한 치수를 알고 있다면 기존 [숫자 설정 방식](ai_worker_sorting/LIVE_ROBOT.md#3-실제-피드백에서-설정-생성)도 사용할 수 있다.

## 5. 실행과 작업 명령

첫 터미널에서:

```bash
bash scripts/worker.sh run
```

PC에서는 기본 `camera_driver:=external`을 유지해 로봇 카메라 토픽을 사용한다.
노드 실행만으로 팔은 움직이지 않는다. 두 개의 빈 테이프 구역과 떨어진 두 종류의 물건을 준비한다.

두 번째 터미널에서도 2단계의 ROS2 환경과 같은 `WORKER_WS`, `WORKER_CONFIG`,
필요 시 `WORKER_PYTHON`을 설정하고 같은 Git 폴더로 이동한다.

```bash
bash scripts/worker.sh prepare
# 반환된 session_id를 입력. 종류→구역 배정 rules를 확인한다.
bash scripts/worker.sh start --session-id '반환된_session_id' --max-objects 2
# start의 accepted는 접수 결과이며 실제 완료는 status로 확인한다.
bash scripts/worker.sh status --task-id '반환된_task_id'
bash scripts/worker.sh session --session-id '반환된_session_id'
```

`start`가 실제 작업 명령이다. 처음에는 물체 2개만 처리하고, 검증 후 `--max-objects 0`으로
남은 물체를 처리한다. 작업 제한 시간은 기본 600초이며 더 긴 작업은 YAML에서 조절한다.
응답이 끊겨도 `start`를 자동 재전송하지 않는다. 기존 작업을 확인한 뒤 진행한다.

```bash
bash scripts/worker.sh cancel --task-id '반환된_task_id'
bash scripts/worker.sh status --task-id '반환된_task_id'
```

취소 접수는 정지 완료가 아니다. 상태와 실제 장비를 확인한다. 네트워크가 끊기면 PC의
취소 명령도 로봇에 도달하지 못하므로 장비의 비상정지를 사용한다.
정상 종료는 작업이 끝난 뒤 첫 터미널에서 Ctrl+C로 한다.

## 6. Git 업데이트

진행 중 작업이 없음을 확인하고 프로그램을 종료한 뒤:

```bash
git status --short
git pull --ff-only origin codex/initial-release
bash scripts/worker.sh build
bash scripts/worker.sh doctor
bash scripts/worker.sh run
```

로컬 소스 수정이 있으면 pull 전에 별도 커밋/백업으로 정리한다. 강제로 덮어쓰지 않는다.
로컬 설정과 가중치는 Git 밖에 유지한다. 재시작하면 session/task ID는 다시 발급받는다.
`msg`/`srv`가 바뀌면 이 사용자 정의 인터페이스를 쓰는 다른 PC도 같은 커밋으로 재빌드한다.
현재 구성에서는 공식 로봇 구동 코드는 이 저장소의 사용자 정의 서비스를 사용하지 않는다.

## 연결 문제 확인

| 증상 | 확인 |
|---|---|
| `check` import 실패 | 실행 Python, source 순서, 누락 ROS 패키지·Ultralytics |
| 로봇 노드·토픽이 안 보임 | 동일 domain·호환 middleware, DDS/Zenoh 경로, 방화벽, 컨테이너 네트워크 |
| 노드는 보이지만 영상이 안 옴 | `ros2 topic hz`로 실제 데이터 확인, 대역폭·QoS·depth 정렬 |
| stale/clock 오류 | PC와 로봇 시간 동기화, 센서 지연 |
| TF/제어 링크 오류 | 실제 카메라 외부 보정·Cyclo `l_gripper_name`·기준 좌표계 |
| doctor PASS 후 prepare 실패 | 빈 목적 구역 2개, source ROI, 두 종류 YOLO 탐지, 테이프 색 |

실제 장비에는 아직 접속하지 않았다. 소프트웨어 검증 범위는 [검증 기록](RELEASE_CHECKS.md)에 있다.
