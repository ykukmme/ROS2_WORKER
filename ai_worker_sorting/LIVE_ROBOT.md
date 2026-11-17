# Docker에서 실제 AI WORKER 연결

공용 PC 실행은 [현장 안내](../docker/SHARED_PC.md), 제작·업데이트는 [Docker 가이드](../DOCKER.md)를 따른다.
연결 PC에 ROS2·YOLO를 따로 설치하거나 호스트에서 ROS2 환경을 source할 필요는 없다.

## 역할과 실행 흐름

| 위치 | 준비·실행 |
|---|---|
| AI WORKER 측 | 기존 공식 bringup·MoveL 제어기·카메라 드라이버·TF |
| 공용 Ubuntu PC | Jazzy Docker·터미널 메뉴·인식·분류 |
| Docker 볼륨 | `/data/config/live.local.yaml`과 교시·로그 |

```text
RGB + 정렬된 깊이 + CameraInfo
 → rgbd_input → rgbd_detector → vision_service
 → zone_service → auto_worker → robotis_robot
 → 공식 MoveL/그리퍼 제어기 → TCP/관절 피드백 확인
```

공식 제어기·카메라는 로봇 측의 검증된 기존 절차로 시작한다.
이미 실행 중이면 중복으로 시작하지 않는다. 로봇·카메라의 정확한 모델을 사용한다.
기본 이미지에는 하드웨어 드라이버·Cyclo 제어기 실행 파일을 포함하지 않는다.
PC와 로봇의 ROS2 배포판·domain·호환 middleware·시간을 맞춘다.

## 카메라와 교시

RGB-D·카메라 정보·카메라↔로봇 TF가 필요하다. 일반 RGB 웹캠만으로 실물 집기를 시작하지 않는다.
기본 입력:

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

카메라는 고정하고 물건은 서로 떨어져 전체 깊이가 보여야 한다.
두 종류는 위에서 같은 그리퍼 방향으로 집을 수 있어야 한다.
메뉴의 `1 초기 설정`에서 작업 범위와 두 종류의 집기 자세를 지정한다.
정확한 URDF·TF를 전제로 그리퍼 길이를 모델에서 계산하므로 빈 테이블 기준 교시는 없다.
커스텀 모델·연결은 [Docker 고급 설정](../ADVANCED_SETUP.md)을 따른다.

## 제어 인터페이스

공식 Cyclo AI Worker MoveL 컨트롤러 기준이다.

| 연결 | 기본값 |
|---|---|
| MoveL | `/l_goal_move`, `robotis_interfaces/msg/MoveL` |
| 그리퍼 입력 | `/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory` |
| 메시지 | `trajectory_msgs/msg/JointTrajectory` |
| 관절 피드백 | `/joint_states`, `sensor_msgs/msg/JointState` |
| 제어 TCP | Cyclo의 `l_gripper_name` 파라미터 |
| 그리퍼 관절 | `gripper_l_joint1` |

그리퍼 기본값 `open=0.0`, `closed=0.7`은 관절값이다.
제어 링크·토픽·관절 계약을 확인하고 불일치하면 명령을 거부한다.
`controller_frame` 기본값은 `base_link`다. 작업 좌표계를 제어 좌표계로 변환한다.

이미지에는 고정 리비전의 `robotis_interfaces` 1.3.0과 `ffw_description` 2.2.6 자산이 포함된다.
실제 메시지·형상·장착·TCP와 일치해야 한다. 로봇 공식 제어 환경을 자동 교체하지 않는다.

## 이동 시간과 종료

| 설정 | 기본값 |
|---|---|
| `cartesian_speed` | 0.02 m/s |
| `angular_speed` | 0.15 rad/s |
| `move_duration` | 최소 요청 5초 |
| `action_timeout` | 60초 |
| `task_timeout` | 600초 |

속도는 이동 시간 계산 기준이며 실제 순간 최고 속도를 보장하는 제한은 아니다.
시작·상태·취소는 메뉴를 사용한다. 실제 정지 확인 후 메뉴를 닫는다.
`docker stop/kill`을 로봇 정지 수단으로 사용하지 않는다.

공식 메시지·제어기 에뮬레이터로 검증했다. 실물 네트워크·좌표 정확도·집기는 현장 확인이 필요하다.
[검증 기록](../RELEASE_CHECKS.md)을 참고한다.
기존 개별 명령·수동 치수 절차는 [이전 기록](../docs/archive/NATIVE_LIVE_ROBOT.md)에 있다.
