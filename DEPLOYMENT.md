# Ubuntu 설치·배포 안내

기본 방식은 공개 GHCR Jazzy 이미지를 자동 다운로드하는 Docker 실행기다.

**처음 설치·운용하는 경우 [처음부터 실행 가이드](docker/SHARED_PC.md)를 순서대로 따른다.**
이미지 다운로드, 실행기 받기, 의존성 확인, 공식 제어기·카메라 준비, 두 종류 교시, 첫 실제 작업까지 한 문서에 있다.

| 환경 | 방법 |
|---|---|
| Git이 있는 공용 PC | `git clone` 후 저장소에서 `bash worker.sh` |
| Git이 없는 공용 PC | 실행기 파일만 저장해 `bash worker.sh` |
| 인터넷이 없는 PC | 이미지가 포함된 USB 배포 폴더에서 `bash worker.sh` |
| 로컬 코드를 개발하는 PC | 소스 이미지 빌드 후 `WORKER_IMAGE=ros2-worker:jazzy bash worker.sh` |

공용 PC에는 Docker Engine·사용 권한만 준비한다.
Git·브라우저·curl 중 가능한 방법으로 실행기를 받는다.
GitHub·Docker 로그인, 호스트 ROS2/Python 설치, 로컬 이미지 빌드는 일반 실행에 필요 없다.

다운로드·업데이트는 [공용 PC 안내](docker/SHARED_PC.md),
교시·메뉴 조작은 [터미널 메뉴](TERMINAL_MENU.md)를 따른다. 첫 사용 순서는 `1` → `6` → `2` → `3`이다.
로컬 빌드·오프라인 배포는 [Docker 가이드](DOCKER.md)에 있다.
Git만 업데이트하면 기존 이미지는 바뀌지 않으므로 이미지 갱신에는 `bash worker.sh update`가 필요하다.

공식 제어기·카메라·TF는 AI WORKER 측에서 실행한다. [실물 연결](ai_worker_sorting/LIVE_ROBOT.md)을 따른다.
자동 이미지 배포와 최초 공개 설정은 [배포 관리자 안내](PUBLISHING.md)에 있다.
기존 직접 설치 절차는 [이전 기록](docs/archive/NATIVE_INSTALL.md)에 보관한다.
