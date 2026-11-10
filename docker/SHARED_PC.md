# 공용 Ubuntu PC에서 실행

관리자가 Docker Engine 사용 권한을 제공해야 한다. Git·ROS2·Python·Compose 설치는 필요 없다.
컨테이너는 Ubuntu 24.04 x86_64 / ROS2 Jazzy / Python 3.12 / CPU 기본 배포다.
로봇 배포판과 다른 이미지는 사용하지 않는다.
AI WORKER의 공식 제어기·카메라는 로봇 측에서 기존 방식으로 실행한다.

## 실행

받은 배포 폴더 전체를 USB 또는 PC에 두고 그 폴더에서 실행한다:

```bash
bash worker.sh
```

최초 실행은 `SHA256SUMS`로 이미지 파일을 검사한 뒤 Docker에 자동 등록한다.
다음부터 같은 명령으로 메뉴가 열린다. 인터넷이나 GitHub 로그인이 필요 없다.
기본 ROS domain은 0, middleware는 Fast DDS다. 로봇과 다르면 실행 전에 실제 값으로 지정한다:

```bash
export ROS_DOMAIN_ID=실제_로봇_도메인_숫자
# Cyclone DDS를 사용하는 로봇인 경우에만:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bash worker.sh
```

메뉴에서 `1 초기 설정` → 작업 범위 최초 입력 → 두 종류 스캔·집기 자세 교시·저장.
그 다음 `3 분류 시작`을 선택하고 안내를 확인한다. 가중치와 추론 장치는 자동 선택한다.
팔을 작업물에서 떨어뜨리고, 빈 그리퍼로 시작한다. 실제 시작은 별도 확인 후 수행한다.

## 저장 위치

설정·교시·로그는 Docker 전용 볼륨 `ros2-worker-data`에 보관한다.
PC 작업 폴더에 `worker-data`를 만들지 않으며 컨테이너를 지워도 설정은 유지된다.
볼륨과 이미지는 실제로 PC의 Docker 저장소에 남는다. 다른 PC로 설정이 자동 이동하지는 않는다.
여러 장비의 설정은 `export WORKER_VOLUME=ros2-worker-장비별이름`으로 분리한다.
여러 로봇을 동시에 제어하는 용도가 아니다. 동일 이름 컨테이너가 있으면 중복 실행을 거부한다.

기존 `worker-data` 폴더를 한 번 옮길 때는 메뉴를 종료하고 실제 장비가 정지한 상태에서:

```bash
bash worker.sh import-data /기존/ROS2_WORKER/worker-data
```

볼륨에 파일이 있으면 가져오기를 거부한다. 커스텀 모델은 함께 옮기고 설정 경로를
`/data/models/best.pt`처럼 컨테이너 경로로 맞춘다. 기존 폴더는 자동 삭제하지 않는다.

## 새 이미지로 업데이트

작업 종료·실제 장비 정지·메뉴 종료 후 새 배포 폴더에서:
이전 터미널에 `WORKER_IMAGE`를 지정했다면 먼저 `unset WORKER_IMAGE`로 해제한다.

```bash
bash worker.sh load
bash worker.sh
```

이전 Humble 배포 폴더 대신 Jazzy 배포 폴더 전체를 사용한다. `image.txt`가 `ros2-worker:jazzy`인지 확인한다.
기존 볼륨 설정은 유지된다. 의존성 확인만 하려면 `bash worker.sh check`를 사용한다.
프로그램 종료가 실제 로봇 정지를 대신하지 않는다. 작업 중 `docker stop/kill`을 사용하지 않는다.
GPU 배포 이미지는 호스트 NVIDIA 드라이버·Container Toolkit이 추가로 필요하고 `WORKER_GPU=1`로 실행한다.
GPU·실물 네트워크와 집기는 현장 검증이 필요하다.
