# ROS2_WORKER

ROBOTIS AI WORKER용 RGB-D·YOLO 물건 분류 프로그램.
두 종류를 인식해 색 테이프 구역에 같은 종류끼리 배치한다.
기본 공개 배포는 ROS2 Jazzy / Ubuntu 24.04 / Python 3.12 / x86_64 CPU다.

## Ubuntu에서 실행

**처음 사용한다면 [이미지 다운로드부터 첫 실제 분류까지](docker/SHARED_PC.md)를 순서대로 따른다.**
Docker 확인·이미지 받기·로봇 연결·테이프 구역 준비·초기 교시·연결 검사·첫 작업·다음 묶음 실행을 안내한다.

Docker Engine과 사용 권한이 준비된 Ubuntu PC에서:

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
bash worker.sh
```

첫 실행에 `ghcr.io/ykukmme/ros2-worker:jazzy` 이미지를 자동 다운로드하고 메뉴를 연다.
GitHub 로그인·Docker 로그인·로컬 빌드·호스트 ROS2/Python 설치는 필요 없다.
다음부터는 같은 폴더에서 `bash worker.sh`만 실행한다.
이미지가 있으면 네트워크 다운로드 없이 기존 이미지를 사용한다.
Git이 없는 PC는 [실행기 파일만 받는 방법](docker/SHARED_PC.md)을 사용한다.

AI WORKER 측에는 공식 제어기·카메라·TF가 준비돼 있어야 한다.
첫 사용은 `1 초기 설정` → `6 연결 검사` → `2 물건·구역 확인` → `3 분류 시작` 순서다.
작업 범위와 두 종류의 집기 자세는 최초 설정이 필요하다. 가중치·추론 장치는 자동 선택한다.
설정·교시·로그는 Docker 볼륨 `ros2-worker-data`에 유지된다.
한 번 시작한 세션은 재사용하지 않는다. 다음 묶음은 실제 작업 종료 확인 후 메뉴를 닫고 구역을 비운 뒤 다시 실행한다.

## 업데이트

실제 장비 정지·작업 종료를 확인하고 메뉴를 닫은 뒤:

```bash
git pull
bash worker.sh update
bash worker.sh
```

업데이트는 명시적으로 실행한 경우에만 이미지를 다시 받는다.
실행 중인 기본 컨테이너가 있으면 이미지 업데이트를 거부한다.

## 문서

| 문서 | 용도 |
|---|---|
| [처음부터 실행 가이드](docker/SHARED_PC.md) | 이미지 다운로드부터 초기 설정·실물 첫 분류·다음 묶음까지 |
| [Docker 가이드](DOCKER.md) | 통신·데이터·로컬 빌드·USB 배포 |
| [터미널 메뉴](TERMINAL_MENU.md) | 초기 교시·분류·상태·취소 |
| [설치 안내](DEPLOYMENT.md) | 설치 경로 선택 |
| [고급 설정](ADVANCED_SETUP.md) | 커스텀 모델·URDF |
| [배포 관리자 안내](PUBLISHING.md) | GitHub Actions·GHCR 최초 공개 설정 |
| [실물 연결](ai_worker_sorting/LIVE_ROBOT.md) | 로봇 측 준비와 제어 계약 |
| [자동 분류](ai_worker_sorting/AUTO_SORT.md) | 분류 규칙·격리된 모의 데모 |
| [작업 서비스](ai_worker_sorting/WORKER_SERVICES.md) | 서비스 계약 |
| [검증 기록](RELEASE_CHECKS.md) | 소프트웨어 검증 범위 |
| [이전 기록](docs/archive/README.md) | 직접 설치·Windows·Humble 실험 |

## 검증 범위

Jazzy 이미지 빌드, 단위 테스트 72개, 공식 MoveL/그리퍼 소프트웨어 경로 13개,
모델 기반 교시 6개와 배포·볼륨·메뉴를 검증했다.
실제 AI WORKER 연결·집기와 GPU 하드웨어 실행은 미검증이다.

코드는 Apache-2.0이다. 별도 라이브러리·가중치에는 각각의 배포 조건이 적용된다.
