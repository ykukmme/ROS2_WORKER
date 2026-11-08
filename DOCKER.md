# Ubuntu PC용 Docker 실행

코드·ROS2·공식 인터페이스·형상 자산·YOLO 라이브러리·기본 `yolo26n.pt`를 이미지에 포함한다.
PC에는 Docker Engine/Compose 플러그인과 Git이 필요하다. 호스트에 ROS2나 Python 패키지를 따로 설치할 필요는 없다.
기본 빌드는 Ubuntu x86_64 PC/ROS2 Humble/CPU용이다. Jetson은 기존 JetPack 공식 환경을 사용해야 한다.

## 처음 실행

로봇의 공식 bringup·MoveL 제어기·카메라는 기존 방식으로 실행한다.
로봇과 같은 ROS2 배포판·domain·middleware를 사용한다. 로봇이 Humble, domain 0, Fast DDS라면 기본값을 사용할 수 있다.
다르면 빌드·실행 전에 실제 값을 적용한다. 예:

```bash
# 장비가 Jazzy인 경우에만:
export WORKER_ROS_DISTRO=jazzy
# domain도 로봇 값이 0과 다를 때만 실제 값으로 지정:
export ROS_DOMAIN_ID=실제_로봇_도메인_숫자
```

Ubuntu Bash에서:

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
bash scripts/docker_worker.sh build
bash scripts/docker_worker.sh menu
```

공개 저장소이므로 GitHub 로그인은 필요 없다. 첫 빌드는 인터넷과 수 GB의 저장 공간이 필요하며
공식 패키지·모델을 내려받는다. 빌드가 실패하면 메뉴를 실행하지 말고 빌드 오류를 먼저 해결한다.
Docker 권한이 없다면 관리자가 Docker 사용 권한을 설정하거나 `sudo -E bash scripts/docker_worker.sh ...`를 사용한다.
권한 방식은 설치 환경에 맞춘다. 스크립트는 Docker 소켓을 컨테이너에 전달하지 않는다.

## 메뉴에서 필요한 입력

`1 초기 설정`을 선택한다.

- 가중치 경로: 묻지 않는다. 이미지의 기본 모델을 사용한다.
- 추론 장치: 묻지 않는다. PyTorch가 사용할 수 있는 GPU가 있으면 GPU, 없으면 CPU를 선택한다.
- 작업 범위: 기존 설정 또는 `WORKER_WORKSPACE`가 있으면 묻지 않는다. 둘 다 없으면 처음 한 번만 입력한다.
- 그 다음에는 물건 스캔 → 첫 번째 종류 집기 자세 → 두 번째 종류 집기 자세 → 팔을 작업물에서 떨어뜨리고 저장한다.

작업 범위는 실제 로봇 좌표의 `xmin,xmax,ymin,ymax,zmin,zmax`다. 모델만으로 테이블·주변 장애물까지
확인할 수 없으므로 임의 범위로 대체하지 않는다. 운영자가 검증한 범위가 이미 있으면:

```bash
export WORKER_WORKSPACE='검증한_숫자6개를_쉼표로_입력'
```

이 값은 실제 숫자로 바꿔야 한다. 저장한 설정이 있으면 해당 범위를 우선한다.
기본 모델은 학습된 클래스만 구분한다. 해당 모델이 구분하지 못하는 두 종류는 별도 학습 모델이 필요하다.

## 다음부터

처음과 같은 ROS2/domain/middleware 환경변수를 적용한 터미널에서:

```bash
cd ~/ROS2_WORKER
bash scripts/docker_worker.sh menu
```

메뉴의 `3 분류 시작`을 선택하면 물건·구역 확인도 자동 처리한다.
실제 시작에는 별도의 `y` 확인이 필요하며, 컨테이너 시작만으로 로봇을 움직이지 않는다.
메뉴 사용법은 [TERMINAL_MENU.md](TERMINAL_MENU.md)를 따른다.
작업 중 입력이 끊기면 메뉴 PID 1과 작업 서버를 유지하며 취소 요청·상태 확인을 시도한다.
실패/상태 불명은 자동 종료하지 않는다. 실제 장비 정지를 확인한 뒤에만 관리자가 컨테이너를 종료한다.
실행 중인 로봇의 정지를 대신해 `docker stop/kill`을 사용하지 않는다.

## 데이터 보관과 모델 변경

`worker-data/`가 컨테이너의 `/data`로 연결된다.

| PC 경로 | 내용 |
|---|---|
| `worker-data/config/live.local.yaml` | 저장한 교시·장비 설정 |
| `worker-data/config/runtime.local.yaml` | 현재 CPU/GPU에 맞춘 실행용 설정. 원본 교시 설정은 유지 |
| `worker-data/config/menu-logs/` | 메뉴·노드 로그 |
| `worker-data/ultralytics/` | YOLO 사용자 설정 |

컨테이너를 지워도 위 폴더는 유지된다. `worker-data/`는 Git과 이미지 빌드 대상에서 제외한다.
기존 native 설정을 가져오려면 위 `live.local.yaml` 위치에 복사하고, 모델 경로도 컨테이너에서 읽을 수 있게 맞춘다.
기존 커스텀 모델이 없을 때 기본 모델로 몰래 교체하지 않고 오류로 알린다.
커스텀 모델은 `worker-data/models/best.pt`에 두고 설정의 가중치 경로를 `/data/models/best.pt`로 바꾼다.

이미지에는 `robotis_interfaces` 1.3.0과 `ffw_description` 2.2.6 자산이 고정 리비전으로 포함된다.
실제 로봇의 모델·TCP가 해당 자산과 일치해야 한다. 다른 패키지/형상이면 올바른 자산을 추가해야 한다.
카메라 외부 보정·공식 제어기는 이미지가 대신 설치하거나 실행하지 않는다.

## GPU를 사용하는 PC

기본 이미지는 CPU용 PyTorch다. NVIDIA GPU를 사용하려면 PC의 드라이버·NVIDIA Container Toolkit과
GPU 호환 PyTorch 이미지가 필요하다. 아래 설정으로 GPU용 의존성을 선택해 다시 빌드한다:

```bash
export WORKER_GPU=1
bash scripts/docker_worker.sh build
bash scripts/docker_worker.sh menu
```

이 모드에서는 PyPI의 고정 버전 PyTorch와 GPU 컨테이너 설정을 사용한다. CPU 이미지보다 크다.
GPU 호환 wheel이 없는 플랫폼은 빌드가 실패하며, Jetson GPU 구성으로 간주하지 않는다.
하드웨어 GPU 실행은 현재 개발 환경에서 검증하지 않았다.

## 업데이트와 문제 확인

작업 종료와 실제 장비 정지를 확인하고 메뉴를 종료한 뒤:

```bash
git pull
bash scripts/docker_worker.sh build
bash scripts/docker_worker.sh menu
```

의존성과 내장 모델만 확인하려면 `bash scripts/docker_worker.sh check`를 사용한다. 로봇 명령은 보내지 않는다.
Linux host network/IPC를 사용하지만 domain·middleware·방화벽·DDS/Zenoh 설정까지 자동 해결되지는 않는다.
기본 Fast DDS 또는 Cyclone DDS를 사용한다. Zenoh·DDS 설정 파일이 필요한 장비는 해당 런타임/파일을 별도로 맞춰야 한다.
배포판·모델이 다른 실제 장비와의 네트워크 및 실물 집기는 현장 확인이 필요하다.
