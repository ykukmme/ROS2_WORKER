# Ubuntu 설치·배포 안내

기본 실행 방식은 Jazzy Docker다. 환경에 맞는 문서로 진행한다.

| 환경 | 필요한 항목 | 실행 |
|---|---|---|
| 배포 폴더를 받은 공용 PC | Docker Engine·사용 권한 | 배포 폴더에서 `bash worker.sh` |
| Git 소스로 이미지를 만드는 PC | Git·Docker Engine/Compose·인터넷 | 저장소에서 `bash scripts/docker_worker.sh build` |

## 공용 PC

[공용 PC 안내](docker/SHARED_PC.md)를 따른다.
PC에 ROS2·Python을 설치하거나 ROS2 환경을 source할 필요는 없다.
최초 설정과 일상 사용은 [터미널 메뉴](TERMINAL_MENU.md)에 있다.

## 제작 PC와 업데이트

[Docker 가이드](DOCKER.md)를 따른다. GitHub 로그인은 필요 없다.
기존 저장소에서는 `git pull` 후 이미지를 다시 빌드하고 내보낸다.
공용 PC에는 새 배포 폴더 전체를 전달하고 `bash worker.sh load`로 갱신한다.
Git 소스만 업데이트하면 이미 만들어진 이미지의 코드는 바뀌지 않는다.

## 로봇 측 준비

공식 제어기·카메라·TF는 AI WORKER 측에서 실행한다. [실물 연결](ai_worker_sorting/LIVE_ROBOT.md)을 따른다.
기존 호스트 직접 설치 절차는 [이전 기록](docs/archive/NATIVE_INSTALL.md)에 보관한다.
