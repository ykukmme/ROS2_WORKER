# Docker 고급 설정

일반 사용은 [터미널 메뉴](TERMINAL_MENU.md)를 따른다.
이 문서는 다운로드한 `ghcr.io/ykukmme/ros2-worker:jazzy` 이미지와 Docker 볼륨의 파일을 직접 다루는 경우에 사용한다.
메뉴 종료와 실제 장비 정지를 확인한 뒤 진행한다.

## 파일 확인·가져오기

기존 설정 폴더는 [Docker 가이드](DOCKER.md)의 `import-data`로 가져온다.
폴더에는 `config/live.local.yaml`과 필요한 `models/`가 있어야 한다.
대상 볼륨에 파일이 있으면 가져오기를 거부한다.

파일 확인은 Ubuntu 호스트에서 다음 콘솔을 연다:

```bash
docker run --rm -it --name ros2-worker-tools --user 1000:1000 \
  --mount type=volume,src=ros2-worker-data,dst=/data \
  ghcr.io/ykukmme/ros2-worker:jazzy bash
```

다음은 **컨테이너 콘솔 안**에서 실행한다:

```bash
ls /data/config
mkdir -p /data/models
```

필요한 파일은 다른 호스트 터미널에서 복사한다. USB 경로는 실제 파일 위치로 바꾼다:

```bash
docker cp /media/usb/best.pt ros2-worker-tools:/data/models/best.pt
docker cp /media/usb/robot.urdf ros2-worker-tools:/data/models/robot.urdf
```

콘솔에서 `exit`하면 종료되며 볼륨의 파일은 유지된다.
다른 볼륨을 쓰면 위 명령의 `src`도 해당 이름으로 바꾼다.
커스텀 모델 사용 시 기존 검증된 YAML의 가중치 경로를 `/data/models/best.pt`로 맞춘다.
URDF는 확장된 XML이어야 하며 메시 경로도 컨테이너에서 읽을 수 있어야 한다.
모델·토픽·좌표계·그리퍼가 바뀌면 메뉴에서 초기 설정을 다시 수행한다.

## 다른 URDF·설정 경로로 메뉴 실행

일반 `worker.sh`의 설정 위치는 `/data/config/live.local.yaml`이다.
호스트의 `WORKER_CONFIG`·`WORKER_ROBOT_URDF`는 일반 실행기에서 컨테이너로 전달하지 않는다.
다른 파일은 Docker에 명시적으로 전달한다.

아래 예시는 기본 설정 경로와 `/data/models/robot.urdf`를 사용한다.
로봇 domain·middleware가 기본값과 다르면 실제 값으로 바꾼다.
공식 노드의 `robot_description`을 사용할 때는 `WORKER_ROBOT_URDF` 행을 생략한다.

```bash
docker run --rm -it --name ros2-worker --stop-timeout 30 \
  --network host --ipc host --user 1000:1000 \
  --mount type=volume,src=ros2-worker-data,dst=/data \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e WORKER_CONFIG=/data/config/live.local.yaml \
  -e WORKER_ROBOT_URDF=/data/models/robot.urdf \
  ghcr.io/ykukmme/ros2-worker:jazzy menu
```

메뉴가 교시·설정·분류 작업을 관리한다. 별도 서버나 서비스 시작 명령을 함께 실행하지 않는다.
다른 파라미터 노드는 일반 실행기에서도 `WORKER_ROBOT_DESCRIPTION_NODE`로 지정할 수 있다.
기본 자산과 다른 로봇 형상은 해당 자산을 컨테이너에 추가해야 한다.
통신·제어 계약은 [실물 연결](ai_worker_sorting/LIVE_ROBOT.md)을 따른다.
과거 직접 설치와 개별 교시 명령은 [이전 기록](docs/archive/NATIVE_ADVANCED.md)에 있다.
