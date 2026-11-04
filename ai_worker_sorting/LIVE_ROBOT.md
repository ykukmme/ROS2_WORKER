# 실제 AI WORKER 실행

연결 PC에서 Git으로 설치하고 조작하는 전체 절차는 [DEPLOYMENT.md](../DEPLOYMENT.md)를 따른다.
이 문서는 실제 제어 인터페이스·현장 치수·보정 설정을 설명한다.

이 실행 경로는 `DryRobot`을 사용하지 않는다.
실제 RGB-D 영상과 관절/TF를 읽고 ROBOTIS Cyclo 컨트롤러에 `MoveL` 및 그리퍼 명령을 보낸다.
VLA/VLM은 사용하지 않는다.

장비에 SSH로 접속할 수 없어 실제 로봇에서 시험하지는 않았다. 공식 메시지와
컨트롤러 에뮬레이터를 사용해 `dry_run=false` 전체 경로를 검증했다.
카메라 좌표 보정은 필요하다. 정확한 URDF·TF를 전제로 그리퍼 길이는 모델에서 계산하고,
물건 크기는 카메라 스캔으로 추정한다. 두 종류의 집기 자세는 한 번씩 지정한다.

## 실행 구조

```text
실제 RGB + RGB에 정렬된 깊이 + CameraInfo
 → rgbd_input: 같은 맵으로 색/깊이 왜곡 보정
 → rgbd_detector: 실제 로컬 YOLO 가중치 추론
 → vision_service: 깊이·TF로 물건 표면 위치 계산
 → zone_service: 색 테이프 영역의 실제 좌표 계산
 → auto_worker: 두 종류 자동 배정·반복 집기·배치
 → robotis_robot: 공식 MoveL/관절 궤적 발행, 실제 TCP/관절 피드백 확인
```

기존 Windows 웹캠 HTTP 브리지는 이 경로에서 사용하지 않는다.
카메라는 작업 동안 고정하고 물건은 서로 떨어져 있어야 한다.
두 종류 모두 위에서 같은 그리퍼 방향으로 잡을 수 있어야 한다.
표면 중심을 집기 위치로 사용하는 버전이므로 얇은 물건·기울어진 물건·겹친 물건은 별도 집기 처리가 필요하다.

## 지원하는 컨트롤러 계약

공식 `cyclo_motion_controller_ros`의 AI Worker MoveL 컨트롤러를 기준으로 한다.

| 연결 | 기본값 |
|---|---|
| MoveL 명령 | `/l_goal_move`, `robotis_interfaces/msg/MoveL` |
| 그리퍼 입력 | `/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory` |
| 그리퍼 메시지 | `trajectory_msgs/msg/JointTrajectory` |
| 실제 관절 상태 | `/joint_states`, `sensor_msgs/msg/JointState` |
| TCP | 실행 중인 Cyclo의 `l_gripper_name` 파라미터에서 읽음 |
| 그리퍼 관절 | `gripper_l_joint1` |

Cyclo의 목표 좌표는 모델 루트 좌표계다. `controller_frame` 기본값은 `base_link`이며,
작업 좌표계가 다르면 어댑터가 위치·방향을 이 좌표계로 직접 변환해서 보낸다.

그리퍼 기본 명령은 공식 예제의 관절값 `open=0.0`, `closed=0.7`이다.
이는 미터 단위의 그리퍼 폭이 아니다. Cyclo의 **raw 입력**으로 그리퍼 관절 하나를 보내며,
팔 전체 궤적의 최종 발행자는 Cyclo 하나로 유지한다.
서로 다른 제어기가 같은 하드웨어 궤적 토픽을 동시에 발행하지 않게 구성한다.

참고한 공식 API:

- [ROBOTIS MoveL 메시지](https://github.com/ROBOTIS-GIT/robotis_interfaces/blob/c79afb6aa8d0900401ec38ecdb60b1a104929997/msg/MoveL.msg)
- [Cyclo AI Worker MoveL 컨트롤러](https://github.com/ROBOTIS-GIT/cyclo_control/blob/8e5d0d1668c8de16b264a344ba283f31d508dd52/cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp)
- [ROBOTIS 실제 픽앤플레이스 설명](https://docs.robotis.com/docs/systems/aiworker/resources/technical_story/isaac_centerpose/)

이전 Cyclo 버전에 raw 그리퍼 입력이나 MoveL 메시지가 없다면 해당 버전에 맞는
공식 제어 환경이 필요하다. 이 코드가 로봇의 기존 제어 환경을 임의로 업그레이드하지는 않는다.

## 1. 업데이트·빌드

로봇의 기존 ROS2/ROBOTIS 환경을 source한 터미널에서:

```bash
cd ROS2_WORKER
git pull --ff-only
ros2 pkg prefix robotis_interfaces
ros2 pkg prefix cyclo_motion_controller_ros
bash scripts/build_ros2.sh "$HOME/ros2_worker_ws"
source "$HOME/ros2_worker_ws/install/ros2_worker/setup.bash"
```

YOLO를 실행할 Python 환경에 `ultralytics`, 로봇에 맞는 PyTorch/CUDA, OpenCV가 있어야 한다.
가중치는 별도로 준비한다. 이 작업을 위해 로봇의 기존 JetPack/PyTorch를 교체할 필요는 없다.
ROS2와 같은 Python 버전의 기존 환경을 사용한다.

## 2. 공식 로봇 구동과 카메라

기존 AI WORKER bringup과 올바른 모델의 Cyclo 컨트롤러를 실행한다. 예:

```bash
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=movel model:=bg2
```

모델이 SG2면 `model:=sg2`를 사용한다. 사진만으로 모델을 확정하지 않았으므로
설치된 로봇의 모델/URDF와 맞춰야 한다. 이미 컨트롤러가 실행 중이면 중복 실행하지 않는다.

RealSense의 기존 드라이버가 있다면 RGB와 **RGB에 정렬된** 깊이를 활성화한다.
새 드라이버를 실행하는 경우 장치 일련번호로 머리 카메라를 선택한다.
손목에도 카메라가 있으므로 첫 USB 장치를 임의 선택하지 않는다.

```bash
ros2 launch realsense2_camera rs_launch.py camera_namespace:='' camera_name:=camera \
  serial_no:=_카메라_일련번호 align_depth.enable:=true enable_sync:=true
```

로봇 본체의 일련번호와 카메라 일련번호는 다르다.
기본 카메라 토픽은 다음이며 실제 드라이버 이름에 맞춰 설정할 수 있다.

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

## 3. 실제 피드백에서 설정 생성

**치수 입력 없는 기본 절차는 [배포 가이드 4단계](../DEPLOYMENT.md#4-현장-설정-생성)를 따른다.**
아래는 치수를 이미 알고 있을 때 사용하는 기존 수동 설정 방식이다.

그리퍼를 원하는 **위에서 집는 방향**으로 조그한 뒤 설정을 생성한다.
현재 TCP 방향을 읽어 고정 집기 방향으로 저장한다. 이 명령 자체는 로봇을 움직이지 않는다.

아래 변수는 실측 값으로 채운다.

```bash
WORKSPACE='xmin,xmax,ymin,ymax,zmin,zmax'
MAX_FOOTPRINT='두_물건의_최대_가로세로_크기_m'
MAX_HEIGHT='두_물건의_최대_높이_m'
GRASP_OFFSET='물건_윗면에서_TCP까지_Z오프셋_m'
TOOL_BELOW_TCP='제어_링크_아래로_내려오는_그리퍼_최대_길이_m'
WEIGHTS='/absolute/path/to/best.pt'

ros2 run ai_worker_sorting configure_live \
  --output "$HOME/ros2_worker_config/live.local.yaml" \
  --weights "$WEIGHTS" --workspace "$WORKSPACE" \
  --object-size "$MAX_FOOTPRINT" --max-object-height "$MAX_HEIGHT" \
  --grasp-offset "$GRASP_OFFSET" --tool-below-tcp "$TOOL_BELOW_TCP" --device 0
```

숫자 형식 예시는 workspace가 `0.1,0.8,-0.5,0.5,0.5,1.4`, 길이는 `0.06`이다.
이는 형식 설명이며 사용자 장비에 맞춘 좌표가 아니다. 작업 범위에는 준비/복귀 위치와
테이블 위 최대 물건 높이+이송 여유까지 포함해야 한다.
그립 지점이 윗면보다 2cm 아래라면 `--grasp-offset=-0.02`처럼 지정한다.

카메라·TCP·토픽이 기본값과 다르면 다음 옵션으로 맞춘다.

```text
--color-topic /실제_RGB_토픽
--depth-topic /RGB정렬_깊이_토픽
--camera-info-topic /실제_CameraInfo
--joint-states-topic /실제_joint_states
--base-frame base_link
--controller-frame base_link
--controller-node /ai_worker_movel_controller
--tool-link 실제_l_gripper_name
--movel-topic /l_goal_move
--gripper-topic /leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory
--gripper-joint gripper_l_joint1
--source-roi left,top,right,bottom
```

이미 보정된 ZED 등의 RGB/depth 입력이면 `--input-is-rectified`를 사용한다.
일반 RGB 웹캠 영상에는 사용할 수 없다. 실제 깊이 영상이 필요하다.
설정 도구는 카메라 정보·관절·TCP/카메라 TF·컨트롤러 구독 타입이 없으면 설정 생성을 거부한다.
실제 Cyclo의 제어 링크·명령 토픽·그리퍼 관절을 파라미터 서비스로 확인한다.
다른 TCP 링크는 명령 전에 거부한다. 기본 TCP는 자동으로 선택한다.
기존 카메라↔로봇 TF를 읽는 도구이며, 잘못 장착된 카메라의 외부 보정을 대신 수행하지는 않는다.

## 4. 실물용 스택 실행

기본 이동 설정은 아래와 같다. 이미 생성한 설정 파일에 이동 값이 있으면 아래 값으로 수정한다.
`cartesian_speed`는 이동 시간 계산 기준이며 실제 순간 최고 속도를 보장하는 제한은 아니다.

```yaml
worker_service:
  ros__parameters:
    cartesian_speed: 0.02  # 2 cm/s 기준
    angular_speed: 0.15   # rad/s 기준
    move_duration: 5.0    # 각 이동에 최소 5초 요청
    action_timeout: 60.0
    task_timeout: 600.0
```

예를 들어 방향 변화 없이 20cm 이동하면 최소 10초를 요청한다.

```bash
ros2 launch ai_worker_sorting ai_worker_live.launch.py \
  config:="$HOME/ros2_worker_config/live.local.yaml"
```

별도 Python 환경을 써야 하면 `python_prefix:=/absolute/path/to/python`을 추가한다.
이 launch는 RGB-D 전처리·실제 YOLO·인식·구역·작업 서버를 모두 실행한다.
`dry_run=false`, `mock_mode=false`인 실제 설정만 허용하며 모의 데이터로 대체하지 않는다.

카메라 드라이버를 이 launch에서 함께 시작하려면, 기존 드라이버가 중지된 상태에서
`camera_driver:=realsense camera_serial:=카메라_일련번호`를 추가한다.
카메라 일련번호를 생략하면 시작하지 않는다.

기본 YOLO 필터는 `person`을 분류 대상에서 제외한다. 그 외 물건은 모델이 제공하는
클래스명으로 자동 구분한다. 작업 외 대상은 집기 ROI 밖으로 제외한다.

## 5. 구역 등록·작업 시작

빈 테이프 구역 두 개와 서로 떨어진 두 종류의 물건을 카메라에 보이게 놓는다.
카메라/테이블은 작업 중 움직이지 않도록 한다.

```bash
ros2 service call /worker/prepare_auto_sort ai_worker_interfaces/srv/PrepareAutoSort '{}'
ros2 service call /worker/start_task ai_worker_interfaces/srv/StartTask \
  "{session_id: '응답의_세션_ID', max_objects: 0}"
```

노드를 시작하는 것만으로 팔은 움직이지 않는다. `start_task`가 작업 시작 명령이다.
상태·취소는 기존 `/worker/task_status`, `/worker/cancel_task`를 사용한다.

## 실물 경로에서 확인하는 것

- RGB와 깊이를 동일한 왜곡 보정 맵으로 처리하고 촬영 시각을 유지한다.
- 깊이와 TF가 유효한 표면 좌표만 집기 대상으로 사용한다.
- 준비한 TCP를 복귀 위치로 읽고, 물건 높이·집기 오프셋으로 실제 배치 높이를 계산한다.
- 물건과 그리퍼의 TCP 아래 길이까지 반영해 올린 뒤 수평 이송하고, 복귀도 높은 경유점을 거친다.
- MoveL 완료는 실제 TCP 위치·방향과 새 관절 피드백으로 확인한다.
- 그리퍼 완료는 실제 관절값으로 확인한다. 파지는 별도로 새 영상에서 물건이
  출발점에서 사라지고 TCP 근처로 올라왔는지 확인한다.
- 배치 후 새 영상에서 목적지에 같은 종류가 있고 출발점에는 없는지 확인한다.
- 피드백 소실·미도달·확인 실패는 성공 처리하지 않는다.
- 취소는 현재 측정 TCP와 그리퍼 위치를 유지하도록 요청한다. 물리 비상정지를 대신하지 않는다.

현재 컨트롤러/프로필은 고립된 물건과 정리된 테이블에서의 고정 방향 집기를 대상으로 한다.
외부 장애물 전체의 3D 충돌 장면을 자동 생성하는 기능은 포함하지 않는다.

## 검증 기록

- 단위 테스트 63개.
- 기존 분류·서비스 ROS2 시나리오 18개.
- 공식 `robotis_interfaces` 1.3.0 API와 PC 실행 도구·메뉴 백엔드로 13개 시나리오.
- 모델 기반 길이·카메라 크기 추정·무명령 교시·종류별 집기 설정 생성 6개 시나리오.
- 실제 로컬 YOLO 추론, RGB-D 전처리, 설정 생성, `dry_run=false` 두 종류 배치,
  복귀/배치 높이, 미도달, 취소, 관절 피드백 소실을 확인했다.
- 제어 검증에는 `/live_test/*` 토픽의 에뮬레이터를 사용했다. 실물 로봇 성공률 검증은 아니다.
