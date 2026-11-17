# ROS2_WORKER

ROBOTIS AI WORKER용 ROS2 물건 분류 프로그램.
RGB-D와 YOLO로 두 종류를 인식하고, 색 테이프 구역에 같은 종류끼리 배치한다.
기본 배포는 ROS2 Jazzy / Ubuntu 24.04 / Python 3.12 / x86_64 CPU다.

## 공용 PC에서 사용

PC에는 Docker Engine과 사용 권한만 준비한다. ROS2·Python·Git·Compose를 따로 설치할 필요는 없다.
AI WORKER의 공식 제어기·카메라·좌표 보정은 로봇 측에서 준비한다.
이미지가 포함된 배포 폴더에서:

```bash
bash worker.sh
```

이미지가 없으면 자동 등록 후 메뉴가 열린다.
새 배포 폴더로 업데이트할 때는 `bash worker.sh load`를 먼저 실행한다.
메뉴에서 `1 초기 설정` 후 `3 분류 시작`을 사용한다.
작업 범위와 두 종류의 집기 자세는 최초 설정이 필요하다.
가중치 경로와 추론 장치는 자동 선택한다.

설정·교시·로그는 Docker 볼륨 `ros2-worker-data`에 유지된다.
설정 경로는 컨테이너의 `/data/config/live.local.yaml`이다.
자세한 절차는 [공용 PC 안내](docker/SHARED_PC.md)와 [터미널 메뉴](TERMINAL_MENU.md)를 따른다.

## 이미지를 만드는 PC

Git과 Docker Engine/Compose가 있는 Ubuntu 호스트에서:

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
bash scripts/docker_worker.sh build
bash scripts/docker_worker.sh export
```

`dist/ros2-worker-jazzy` 폴더 전체를 공용 PC에 전달한다.
공개 저장소이므로 GitHub 로그인은 필요 없다. 이미지는 Git이나 공개 레지스트리에 게시하지 않는다.
제작 PC에서 바로 실행할 때는 `bash scripts/docker_worker.sh menu`를 사용한다.

## 문서

| 문서 | 용도 |
|---|---|
| [공용 PC 안내](docker/SHARED_PC.md) | 받은 이미지 실행·업데이트 |
| [Docker 가이드](DOCKER.md) | 이미지 제작·통신·데이터 보관 |
| [터미널 메뉴](TERMINAL_MENU.md) | 초기 설정·분류·상태·취소 |
| [설치 안내](DEPLOYMENT.md) | Docker 설치 경로 선택 |
| [고급 설정](ADVANCED_SETUP.md) | Docker 콘솔·커스텀 모델·URDF |
| [실물 연결](ai_worker_sorting/LIVE_ROBOT.md) | 로봇 측 준비와 제어 계약 |
| [자동 분류](ai_worker_sorting/AUTO_SORT.md) | 분류 규칙·격리된 모의 데모 |
| [작업 서비스](ai_worker_sorting/WORKER_SERVICES.md) | 서비스 계약·상태 |
| [검증 기록](RELEASE_CHECKS.md) | 검증 범위 |
| [이전 개발 기록](docs/archive/README.md) | 직접 설치·Windows·Humble 실험 |

## 검증 범위

Jazzy 이미지 빌드, 단위 테스트 68개, 자동 분류·작업 서비스, 공식 MoveL/그리퍼 메시지 경로 13개,
모델 기반 교시 6개, 오프라인 YOLO 추론과 배포·볼륨·메뉴를 소프트웨어 환경에서 검증했다.
실제 AI WORKER 연결·집기와 GPU 하드웨어 실행은 미검증이다.

코드 라이선스는 Apache-2.0이다. 별도 라이브러리·가중치에는 각 배포 조건이 적용된다.
