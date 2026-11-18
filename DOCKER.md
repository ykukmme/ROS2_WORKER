# Jazzy Docker 실행 가이드

기본 이미지는 `ghcr.io/ykukmme/ros2-worker:jazzy`이며 Linux amd64 / Ubuntu 24.04 / Python 3.12 / CPU용이다.
코드·ROS2·공식 인터페이스·형상 자산·YOLO·기본 가중치를 포함한다.
일반 실행은 Docker Engine과 사용 권한이 필요하다. 이미지는 공개 GHCR에서 로그인 없이 받는다.
GPU·Jetson은 별도 환경이 필요하다.

**최초 다운로드부터 실제 첫 분류까지는 [처음부터 실행 가이드](docker/SHARED_PC.md)를 따른다.**
이 문서는 데이터 이전·이미지 제작·오프라인 배포 등 Docker 운영 세부 사항을 다룬다.

## 공용 PC에서 온라인 실행

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
bash worker.sh
```

최초 실행은 이미지가 없으면 자동 다운로드한다. 이후에는 기존 이미지를 재사용한다.
호스트 ROS2·Python 설치나 이미지 빌드는 필요 없다.
Git이 없으면 [공용 PC 안내](docker/SHARED_PC.md)의 실행기 파일 다운로드 방법을 사용한다.

로봇의 공식 bringup·MoveL 제어기·RGB-D 카메라·TF는 AI WORKER 측에서 준비한다.
PC와 로봇의 배포판·domain·호환 middleware·시간을 맞춘다.
기본값은 Jazzy / domain 0 / Fast DDS다. 다를 때만 실행 전에 지정한다:

```bash
export ROS_DOMAIN_ID=실제_로봇_도메인_숫자
# Cyclone DDS 장비인 경우에만:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash worker.sh
```

Linux host network/IPC를 사용한다. 방화벽·카메라 외부 보정·별도 DDS/Zenoh 설정을 대신하지는 않는다.
공개 이미지 준비와 익명 접근 검증은 [배포 관리자 안내](PUBLISHING.md)를 따른다.

## 메뉴와 설정

[터미널 메뉴](TERMINAL_MENU.md)의 `1 초기 설정` → `6 연결 검사` → `2 물건·구역 확인` → `3 분류 시작` 순서를 따른다.
가중치 경로·추론 장치는 자동 선택한다. 기본 CPU 이미지는 CPU를 사용한다.
작업 범위는 저장값을 재사용하며 없으면 최초 한 번 입력한다.
두 종류별 집기 자세는 기존 조그로 지정한다. 그리퍼 형상은 올바른 URDF·TF에서 계산한다.
테이프는 서로 뚜렷이 다른 두 색의 닫힌 사각형을 사용한다. 색 지정 없이 인식하고 메뉴에 감지 색과 위치를 표시한다.
기존 볼륨에 `zone_colors: [red, blue]`가 저장돼 있어도 새 이미지의 구역 검출은 자동 모드로 동작한다.
다음 묶음은 이전 작업의 실제 종료 확인 후 메뉴를 닫고 구역을 비운 뒤 다시 실행한다. 시작한 세션은 재사용하지 않는다.

검증된 작업 범위가 이미 있을 때만 `WORKER_WORKSPACE`에
실제 로봇 좌표의 `xmin,xmax,ymin,ymax,zmin,zmax`를 쉼표로 지정한다.
이미 있는 설정의 범위를 우선한다. 임의 범위를 만들어 사용하지 않는다.

## 데이터 보관

Docker 볼륨 `ros2-worker-data`를 컨테이너의 `/data`로 사용한다.

| 파일 | 내용 |
|---|---|
| `/data/config/live.local.yaml` | 저장한 교시·장비 설정 |
| `/data/config/runtime.local.yaml` | 현재 추론 장치에 맞춘 실행용 설정 |
| `/data/config/menu-logs/` | 메뉴·노드 로그 |
| `/data/ultralytics/` | YOLO 사용자 설정 |

컨테이너를 지워도 볼륨은 유지된다. 볼륨과 이미지는 PC의 Docker 저장소에 남는다.
호스트 작업 폴더에 `worker-data/`를 만들지 않는다.
다른 장비는 `WORKER_VOLUME=ros2-worker-robot-a`처럼 영문·숫자·하이픈 이름으로 분리한다.

기존 설정을 가져올 때는 메뉴 종료와 실제 장비 정지를 확인한다.
가져올 폴더는 `config/live.local.yaml`과 필요한 `models/`를 포함해야 한다.
초기 설정·진단으로 대상 볼륨에 파일을 만들기 전에 가져온다:

```bash
bash worker.sh import-data /기존/worker-data
```

대상에 파일이 있으면 거부한다. 기존 폴더는 삭제하지 않는다.
커스텀 모델은 `/data/models/best.pt`처럼 컨테이너 경로로 맞춘다.
읽지 못하는 커스텀 모델을 기본 모델로 몰래 교체하지 않는다.
파일 변경·URDF 지정은 [고급 설정](ADVANCED_SETUP.md)을 따른다.

## 온라인 업데이트와 버전 고정

실제 장비 정지·작업 종료를 확인하고 메뉴를 닫은 뒤:

```bash
git pull
bash worker.sh update
bash worker.sh
```

실행 중인 기본 컨테이너가 있으면 `update`·`load`를 거부한다.
다운로드 실패 시 메뉴를 열지 않으며 기존 이미지·볼륨을 초기화하지 않는다.
특정 버전은 `WORKER_IMAGE`에 공개 GHCR의 `sha-전체커밋해시` 태그나 digest로 지정한다.
이미지는 기본 브랜치의 이미지 관련 변경에 대해 GitHub Actions가 빌드·검사한 뒤 게시한다.

의존성·내장 모델 확인은 `bash worker.sh check`다. 로봇 명령은 보내지 않는다.
출력의 `ROS2: jazzy`, `Python: 3.12...`로 버전을 확인한다.
메뉴의 `6 연결 검사`는 로봇 측 연결을 진단한다.

## 로컬 개발 이미지

Git과 Docker Engine/Compose가 있는 Ubuntu 호스트에서 저장소를 받은 뒤:

```bash
bash scripts/docker_worker.sh build
WORKER_IMAGE=ros2-worker:jazzy bash worker.sh
```

빌드는 인터넷과 수 GB의 공간이 필요하다. 실패하면 빌드 오류를 먼저 해결한다.
공개 이미지와 로컬 이미지를 구분하므로 로컬 코드를 실행할 때 위 `WORKER_IMAGE`를 지정한다.
다른 지원 배포판은 빌드 전에 `WORKER_ROS_DISTRO`로 지정한다. 호스트의 `ROS_DISTRO`로 자동 선택하지 않는다.

## USB 오프라인 배포

공개 이미지를 받아서 USB용 배포 폴더를 만들 때:

```bash
bash worker.sh pull
WORKER_IMAGE=ghcr.io/ykukmme/ros2-worker:jazzy bash scripts/docker_worker.sh export
```

직접 빌드한 로컬 이미지는 `bash scripts/docker_worker.sh export`로 내보낸다.
`dist/ros2-worker-jazzy` 폴더 전체를 실행 PC로 옮긴다.
코드가 포함된 이미지·실행기·태그·체크섬·안내만 전달하며 현장 설정·교시·로그는 넣지 않는다.

받은 폴더에서 `bash worker.sh`를 실행한다.
이미지가 없으면 체크섬 검증 후 동봉 이미지를 등록하므로 인터넷·Git은 필요 없다.
같은 태그의 이미지가 이미 있는 새 번들은 `bash worker.sh load`로 명시적으로 갱신한다.
온라인 `update`와 USB `load`를 구분한다.

## GPU 개발 환경

공개 `jazzy` 이미지는 CPU용이다. NVIDIA GPU는 호스트 드라이버·Container Toolkit이 추가로 필요하다.
개발 PC에서 GPU용 이미지를 직접 빌드하고 선택한다:

```bash
export WORKER_GPU=1
bash scripts/docker_worker.sh build
WORKER_IMAGE=ros2-worker:jazzy bash worker.sh
```

고정 버전 PyTorch의 GPU 의존성을 사용한다. GPU·Jetson 실행은 미검증이다.

작업 중 입력이 끊기면 메뉴와 서버를 유지하며 취소·상태 확인을 시도한다.
실패/상태 불명에서 자동 종료하지 않는다. 실제 정지 확인 후 종료하며 `docker stop/kill`을 로봇 정지 용도로 사용하지 않는다.
