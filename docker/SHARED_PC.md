# 공용 Ubuntu PC에서 실행

Docker Engine과 사용 권한을 준비한다. 실행 환경은 Jazzy / Python 3.12 / x86_64 CPU다.
AI WORKER 측에는 기존 공식 제어기·카메라·TF가 실행 중이어야 한다.
호스트 ROS2·Python 설치나 GitHub·Docker 로그인은 필요 없다.

## Git으로 받기

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
bash worker.sh
```

첫 실행에 공개 GHCR 이미지를 내려받고 메뉴를 연다.
이후에는 같은 폴더에서 `bash worker.sh`만 실행한다. 저장된 이미지와 설정을 재사용한다.

## Git 없이 실행기만 받기

브라우저로 실행기 파일을 저장하거나, curl이 있으면 다음 명령을 사용한다:

```bash
curl -fL https://raw.githubusercontent.com/ykukmme/ROS2_WORKER/codex/initial-release/scripts/run_worker.sh -o worker.sh
bash worker.sh
```

이미지 다운로드에는 최초 한 번 인터넷과 수 GB의 Docker 저장 공간이 필요하다.
일반 실행에서는 이미 있는 이미지를 자동 업데이트하지 않는다.
다운로드가 실패하면 메뉴를 열지 않는다. `denied`·`unauthorized`가 나오면 네트워크와 GHCR 공개 상태를 확인한다.
공개 이미지는 로그인 없이 받을 수 있어야 하며, 공개 여부는 배포자가 [관리자 안내](../PUBLISHING.md)에 따라 설정한다.

## 처음 메뉴를 사용할 때

`1 초기 설정` → 작업 범위 최초 입력 → 두 종류 스캔·집기 자세 교시·저장.
그 다음 `3 분류 시작`을 선택하고 안내를 확인한다.
가중치·추론 장치는 자동 선택한다. 팔은 작업물에서 떨어뜨리고 그리퍼는 빈 상태로 시작한다.
실제 작업은 별도 확인 후 시작한다. 자세한 화면은 [터미널 메뉴](../TERMINAL_MENU.md)를 따른다.

기본 domain은 0, middleware는 Fast DDS다. 로봇과 다를 때만 실제 값으로 지정한다:

```bash
export ROS_DOMAIN_ID=실제_로봇_도메인_숫자
# Cyclone DDS를 사용하는 로봇일 때만:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash worker.sh
```

## 저장 위치

| 항목 | Docker 볼륨 내부 경로 |
|---|---|
| 설정·교시 | `/data/config/live.local.yaml` |
| 실행용 설정 | `/data/config/runtime.local.yaml` |
| 메뉴·노드 로그 | `/data/config/menu-logs/` |

볼륨 이름은 `ros2-worker-data`다. 컨테이너를 지워도 유지되며 PC의 Docker 저장소에 남는다.
다른 장비는 `export WORKER_VOLUME=ros2-worker-robot-a`처럼 영문·숫자·하이픈으로 분리한다.
다른 PC로 설정이 자동 이동하지는 않는다.

## 업데이트

실제 장비 정지·작업 종료를 확인하고 메뉴를 닫은 뒤 Git으로 받은 경우:

```bash
git pull
bash worker.sh update
bash worker.sh
```

실행기만 받은 경우에는 새 실행기 파일을 같은 다운로드 주소에서 저장한 뒤
`bash worker.sh update`, `bash worker.sh`를 실행한다.
업데이트는 이미지만 바꾸며 설정 볼륨을 초기화하지 않는다.
진단만 실행하려면 `bash worker.sh check`를 사용한다.
실행 중인 기본 컨테이너가 있으면 `update`·`load`를 거부한다.

## USB 오프라인 배포

이미지·`image.txt`·체크섬이 포함된 배포 폴더를 받았다면 그 폴더에서 `bash worker.sh`로 실행한다.
이미지가 없으면 체크섬을 검사하고 동봉된 이미지를 등록한다. 인터넷·Git은 필요 없다.
새 USB 배포 폴더는 `bash worker.sh load`로 등록한다. 온라인 업데이트 명령과 구분한다.
이전 터미널에 `WORKER_IMAGE`를 지정했다면 번들 기본 이미지를 쓰기 전에 `unset WORKER_IMAGE`로 해제한다.
기존 설정 가져오기와 커스텀 모델은 [Docker 가이드](../DOCKER.md)를 따른다.

`docker stop/kill`을 실제 로봇 정지 수단으로 사용하지 않는다.
GPU·실물 네트워크·집기는 현장 검증이 필요하다.
