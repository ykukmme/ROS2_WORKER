# 두 종류 자동 구역 분류

사용자가 물건 종류·목적지를 입력하지 않아도 다음을 수행한다.

1. 서로 다른 두 색의 닫힌 테이프 사각형 검출.
2. 집기 영역에서 신뢰도 기준을 넘은 서로 다른 YOLO 클래스 두 개 선택.
3. 클래스명과 구역 ID를 정렬하여 일대일 배정하고 세션 동안 고정.
4. 같은 종류는 같은 구역의 서로 다른 빈자리에 배치.
5. 새 물건 종류, 구역 포화, 취소, 좌표 오류 시 후속 동작 중단.

VLA/VLM/자연어 모델은 사용하지 않는다. YOLO가 서로 다른 라벨로 구분할 수 있는
두 종류에 적용한다. 같은 라벨의 다른 물건을 별도 종류로 구분하지는 않는다.
자동 배정은 재현 가능한 순서이며 매 프레임 무작위로 바뀌지 않는다.

## 현재 구현·검증 범위

- 실제 코드로 HSV 마스킹, 닫힌 경계 추출, 구역 등록, 자동 배정, 빈자리 예약을 구현했다.
- 합성 영상의 색 테이프 검출과 ROS2 서비스 연동을 검증했다.
- 가상 `cup`, `can` 두 종류, 총 네 물건을 모의 팔로 분류했다.
  이 클래스는 기존 단발 서버의 `box`, `bottle` 설정에 없어도 동작한다.
- 기본 `dry_run: true`, `mock_mode: false`다.
- 실물 카메라의 색 범위·깊이·TF, AI WORKER 팔, 실제 점유·파지·배치는 아직 현장 검증하지 않았다.
- 일반 MoveIt 연결부는 재사용한다. 실물 모드에는 측정된 구역·물건 좌표,
  검증된 공통 집기 프로필·하드웨어 설정·파지 확인 서비스가 필요하다.
- 시범용 모의 좌표를 실제 로봇 동작에 사용하지 않는다.

## ROS2 서비스

| 서비스 | 타입 | 역할 |
|---|---|---|
| `/workspace/scan_zones` | `ScanZones` | 색 구역 검출·가능한 경우 로봇 좌표 계산 |
| `/worker/prepare_auto_sort` | `PrepareAutoSort` | 구역과 두 종류를 읽어 자동 배정·세션 생성 |
| `/worker/start_task` | `StartTask` | 지정 세션으로 반복 분류 시작 |
| `/worker/session_status` | `SessionStatus` | 배정 규칙·배치 개수·자리 상태 조회 |
| `/worker/task_status` | `TaskStatus` | 작업 진행·완료·실패 조회 |
| `/worker/cancel_task` | `CancelTask` | 취소 요청 |

위 타입은 모두 `ai_worker_interfaces/srv/타입명`이다.
자동 구역 서버에서는 고정 바구니를 사용하는 `/worker/sort_once`를 비활성화했다.
기존 단발 분류 코드는 `services.launch.py`로 별도 실행할 수 있다.

준비 응답 예시:

```yaml
success: true
session_id: "..."
rules:
  - object_class: can
    zone_id: blue_zone
  - object_class: cup
    zone_id: red_zone
```

## 모의 데모 실행

먼저 PowerShell에서 컨테이너에 들어간다. 기본 웹캠 도메인 77과 분리해 79를 사용한다.

```powershell
docker exec -it -e ROS_DOMAIN_ID=79 ros-humble-novnc bash
```

컨테이너에서:

```bash
source /opt/ros/humble/setup.bash
source /tmp/worker_sorting_ws_20261103/install/setup.bash
ros2 launch ai_worker_sorting auto_sort.launch.py mock_mode:=true python_prefix:=/tmp/worker_sorting_yolo_venv/bin/python
```

다른 터미널도 같은 도메인으로 컨테이너에 들어가 두 `source` 명령을 실행한다.
물건 종류를 입력하지 않고 준비를 요청한다.

```bash
ros2 service call /worker/prepare_auto_sort ai_worker_interfaces/srv/PrepareAutoSort '{}'
```

응답의 `session_id`로 시작한다. `max_objects: 0`은 현재 인식 가능한 대상이
더 없을 때까지 처리하며, 같은 빈 화면을 반복 읽어 완료로 판단하지 않는다.

```bash
ros2 service call /worker/start_task ai_worker_interfaces/srv/StartTask "{session_id: '반환된_세션_ID', max_objects: 0}"
ros2 service call /worker/session_status ai_worker_interfaces/srv/SessionStatus "{session_id: '반환된_세션_ID'}"
```

반환된 `task_id`로 진행 상태와 취소를 요청한다.

```bash
ros2 service call /worker/task_status ai_worker_interfaces/srv/TaskStatus "{task_id: '반환된_작업_ID'}"
ros2 service call /worker/cancel_task ai_worker_interfaces/srv/CancelTask "{task_id: '반환된_작업_ID'}"
```

모의 관측은 유한한 네 물건과 가상 구역을 사용한다. 실제 카메라 인식 결과가 아니다.
모의 실행에서는 처리한 가상 물건 ID를 기억해 같은 물건을 다시 집지 않는다.

## 실제 영상으로 구역·배정 확인

1. 서로 다른 두 색으로 배치 구역을 각각 닫힌 사각형으로 표시하고 처음에는 비운다.
2. 집기 영역에 YOLO가 구분할 수 있는 두 종류의 물건을 놓는다.
3. `config/auto_sort.yaml`에서 영상·탐지 토픽, 색, `source_roi`를 맞춘다.
4. `mock_mode: false`로 실행하고 `prepare_auto_sort`를 호출한다.
5. `/workspace/image_zones` 영상과 준비 응답의 배정을 확인한다.

기본 색은 red/blue이며 green/yellow도 코드에서 지원한다. 색 범위는 `zones.py`에 있다.
현재는 색당 한 개의 닫힌 사각형만 허용한다. 끊어진 테이프, 같은 색의 여러 구역,
채워진 색 패치, 겹치는 구역은 거부한다. 조명·색·테이블 무늬에 맞춰 현장 조정이 필요하다.

`source_roi: [left, top, right, bottom]`는 탐지 영상의 **픽셀 단위**다.
기본 `[0,0,640,360]`은 현재 노트북 브리지 해상도다. 실제 카메라 해상도에 맞춰 바꾼다.
배치 구역 내부의 물건은 집기 후보에서 제외한다. 영상에 잡힌 사람 같은 작업 외 대상은
집기 ROI 밖으로 제외해야 한다. 카메라·테이블·테이프는 세션 동안 고정한다.

깊이 없는 웹캠으로도 구역·종류 배정은 확인할 수 있지만, 측정된 3D 구역이 없으므로
모의 팔 동작 시작도 거부한다. 2D 픽셀을 임의의 로봇 거리로 바꾸지 않는다.

## RGB-D·집기·배치 설정

`vision_service`와 `zone_service` 양쪽에 정렬된 깊이, CameraInfo, TF를 연결하고
보정 확인 후 `enable_depth: true`, `calibrated: true`로 설정한다.
깊이 토픽 등은 기존 `WORKER_SERVICES.md`의 설정을 따른다.

구역의 안쪽 네 모서리를 깊이로 계산한다. 네 점이 로봇 기준 수평 테이블 위에
있지 않거나 좌표가 유효하지 않으면 3D 구역으로 사용하지 않는다.

| 공통 프로필 | 의미 |
|---|---|
| `object_size` | 두 종류를 모두 덮는 최대 정사각형 바닥 크기(m), 기본 0.06 |
| `slot_margin` | 물건 주위 여유(m), 기본 0.01 |
| `place_offset` | 테이블 위 배치 시 TCP 높이(m), 기본 0.05 |
| `auto_grasp_offset` | 인식 표면에서 집기 TCP까지 Z 오프셋(m) |
| 기존 그리퍼·자세 설정 | 두 종류 모두에 적용하는 고정 집기 프로필 |

종류 이름을 몰라도 분류할 수 있지만, **두 물건 모두 이 공통 크기·높이·집기 자세에 맞아야 한다.**
인식된 물건의 크기나 최적 집기 방향을 자동 추정하는 기능은 없다.
임의의 크기·재질·기울기의 모든 물건을 다루는 구현은 아니다.

실물 동작은 `dry_run: false` 외에도 기존 `hardware_configured`, MoveIt·그리퍼 설정과
`shared_grasp_verified: true`가 필요하다. 가상 구역/물건은 실물 모드에서 거부한다.
실물 모드에서 배치 직전 탐지된 점유 물건을 확인하고, 배치 후에는 새 관측에서
목적지에 같은 종류가 나타났으며 출발 위치에서 사라졌는지 확인한다.
확인되지 않으면 실패로 기록하고 해당 자리는 재사용하지 않는다.
YOLO가 보지 못하는 장애물의 충돌 회피는 별도의 실제 planning scene 구성이 필요하다.

## 자리와 세션 상태

```text
EMPTY → RESERVED → OCCUPIED
               └→ UNKNOWN (중단·실패)
```

구역 경계에서 물건 크기와 여유만큼 떨어진 격자를 생성한다. 예약/점유/불명 자리는 재사용하지 않는다.
빈자리가 없으면 `ZONE_FULL`로 중단하며 다른 구역으로 목적지를 바꾸지 않는다.
작업 중 세 번째 종류가 나타나면 `NEW_CLASS`로 중단한다.

같은 세션으로 나머지 물건을 이어 처리할 수 있다. 사용한 세션의 매핑을 다시 만들 수는 없다.
새 작업 공간으로 바꾸려면 물건을 치우고 서버를 재시작해 다시 준비한다.
현재 자리 점유는 세션 메모리에 저장되며 외부 사람이 물건을 옮기는 상황의 완전한 점유 추적은 없다.

## 종료·재시작

프로젝트 폴더의 PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/stop_auto_sort.ps1
```

다시 실행하려면 기존 프로세스가 종료된 후 중단 표시를 제거한다.

```powershell
docker exec ros-humble-novnc /tmp/worker_sorting_yolo_venv/bin/python -c "from pathlib import Path; [Path('/tmp/worker_sorting_bridge',n).unlink(missing_ok=True) for n in ['auto-worker.stop','auto-vision.stop','zones.stop']]"
```

같은 컨테이너의 여러 도메인을 동시에 운영하면 중단 파일 경로도 별도로 지정한다.
기본 서버는 ROS_DOMAIN_ID=77, 모의 데모·통합 검증은 79를 사용한다.

## 검증

- 단위 테스트 39개 통과.
- 자동 분류 ROS2 통합 검증 9개 시나리오 통과.
- 기존 작업 서비스 통합 검증 9개 시나리오 재통과.
- 패키지 2개 colcon 빌드 통과.
- 실제 카메라의 테이프 장면·실물 팔 동작은 미검증.

검증 결과: `results/auto-smoke.json`.

코드: `zones.py`, `zone_service.py`, `auto_worker.py`, `config/auto_sort.yaml`,
`launch/auto_sort.launch.py`, `tests/ros_auto_smoke.py`.
