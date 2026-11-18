# 실물 없이 Humble/noVNC에서 실행

`ros-humble-novnc-docker` 계열 환경에 공식 AI WORKER SG2 모델, MuJoCo, 가상 RGB-D 카메라, YOLO와 ROS2 분류 서비스를 넣은 실행 모드다.
실제 로봇·카메라·호스트 ROS2 설치는 필요 없다. Ubuntu PC에는 Docker Engine·Compose와 사용 권한만 필요하다.

## 1. 코드 받기

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
```

이미 받았다면 해당 폴더에서 `git pull`을 실행한다. 공개 저장소이므로 GitHub 로그인은 필요 없다.

## 2. 시뮬레이션 이미지 빌드

```bash
bash scripts/sim_worker.sh build
```

Humble용 의존성 이미지와 noVNC 시뮬레이션 이미지를 순서대로 만든다.
첫 빌드는 인터넷으로 공개 이미지·공식 모델·의존성을 받고 가상 영상용 YOLO를 학습한다. PC 성능에 따라 시간이 걸린다. 다음 빌드는 변경 없는 단계를 캐시에서 재사용한다.

YOLO 가중치·장면·작업 범위·CPU 추론 설정·집기 자세가 이미지에 들어간다. 실행 시 경로 입력, 실측, 조그 교시, 수동 학습은 필요 없다.

기존 `ros-humble-novnc-docker` 이미지를 그대로 기반으로 사용하려면:

```bash
WORKER_NOVNC_BASE=ros-humble-novnc-docker-ros-humble-novnc:latest bash scripts/sim_worker.sh build
```

기본값은 원본 기반 이미지 `tiryoh/ros2-desktop-vnc:humble`이다. 이 시뮬레이션 이미지는 실기용 `:jazzy` 이미지와 별개다. 현재 시뮬레이션 안내는 로컬 빌드 기준이며, 공개 레지스트리에서 받을 수 있다고 가정하지 않는다.

## 3. 실행

```bash
bash scripts/sim_worker.sh
```

터미널에 메뉴가 열린다. 장면 화면은 같은 PC의 브라우저에서 [noVNC](http://localhost:6080)에 접속하면 보인다. noVNC의 기본 접속 비밀번호는 `ubuntu`이며 포트는 PC 내부에서만 접근하도록 묶여 있다.

장면에는 두 종류의 가상 물건과 서로 다른 두 색 테이프 구역이 있다. 메뉴는 다음 순서로 사용한다.

1. `1 인식·구역 확인`: 카메라 영상에서 YOLO와 색 구역을 확인하고 종류별 구역 배정을 고정한다.
2. `2 분류 시작`: 실제 MuJoCo 접촉 연산으로 집어서 해당 구역으로 옮긴다.
3. `3 상태`: 진행 단계와 성공·실패 상태를 확인한다.
4. 필요하면 `4 중지`: 실행 경로를 취소하고 정지를 확인한다.
5. `0 종료`: 시뮬레이터와 화면을 닫는다.

메뉴를 여는 것만으로 분류 작업을 시작하지 않는다. 다음 묶음이나 중지 후 재시작은 메뉴를 종료하고 다시 실행한다. 장면과 분류 세션은 새로 만들어지며, 학습 파일과 결과는 Docker 볼륨에 유지된다.

## 4. 자동 통합 시험

```bash
bash scripts/sim_worker.sh test
```

메뉴·noVNC를 띄우지 않고 인식, 구역 배정, 집기, 운반, 배치를 순서대로 실행한다. 성공 조건은 다음 모두다.

- 실제 YOLO 추론으로 두 종류를 구별했다.
- 영상에서 서로 다른 두 색 구역을 찾았다.
- 각 물건에서 양쪽 손가락 접촉과 지속적인 상승이 확인됐다.
- 배치 후 영상 재인식으로 목적지 존재와 출발지 제거가 확인됐다.
- 별도 평가기가 모든 물건의 실제 물리 위치가 배정 구역 안이며 테이블 위에 있는지 확인했다.

물건을 TCP에 강제로 붙이거나 정답 좌표로 옮겨서 성공시키지 않는다. 물건 종류·위치 정답은 YOLO나 분류 결정에 주입하지 않는다.

위쪽 카메라에서는 집은 물건이 그리퍼에 가려질 수 있어, **집기 확인은 가상 접촉·상승 센서**, **배치 확인은 YOLO 영상**으로 수행한다. 실기 모드의 기존 영상 기반 집기 검증은 유지한다.

실패하면 명령은 0이 아닌 종료 코드를 반환한다. 마지막 결과는 볼륨의 `/sim-data/simulation_result.json`에 기록된다. 다음 명령으로 확인할 수 있다.

```bash
docker compose -p ros2-worker-simulation -f compose.sim.yaml run --rm --entrypoint cat simulation /sim-data/simulation_result.json
```

## 5. 추가 명령

```bash
# 장면의 RGB, 깊이, 구역, YOLO 크기 추정을 검사
bash scripts/sim_worker.sh inspect

# 필요할 때만 가상 영상용 모델 재학습
bash scripts/sim_worker.sh train
```

이미지에 포함된 기본 모델을 사용하면 재학습은 필요 없다. 가상 물건용 가중치의 실물 인식 성능은 별도 검증 대상이다.

## 연결과 중지

- 시뮬레이션은 전용 Docker 네트워크와 ROS domain 219를 사용한다. 호스트 네트워크·특권 실행·실제 장치 마운트는 사용하지 않는다.
- 카메라·제어 토픽은 `/sim/`을 사용하며 실기 제어기를 실행하지 않는다.
- 중지 후 늦게 도착한 명령은 차단한다. 새 실행은 새 컨테이너에서 시작한다.
- 물리 동작 확인은 해당 모델·장면의 결과다. 실제 로봇의 정확한 본체 모델·카메라 보정·마찰·구동 성능 검증을 대신하지 않는다.

세부 연결: [설계서](docs/HUMBLE_SIMULATION_DESIGN.md).

검증 기록(2026-11-18): Ubuntu 22.04·ROS2 Humble 컨테이너에서 이미지 내장 YOLO로 두 종류 물건 4개 분류, 접촉·상승·영상 배치·실제 위치 평가 통과. noVNC 화면 표시와 실행 중 취소·지연 명령 차단을 확인했다. 단위 91개 및 기존 실기·교시 소프트웨어 회귀 20개 시나리오도 통과했다.
