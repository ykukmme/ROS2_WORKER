# Ubuntu에서 로그인 없이 설치하고 사용하기

**의존성 설치를 줄이려면 [Docker 방식](DOCKER.md)을 사용한다.** 아래는 호스트에 직접 설치하는 고급 경로다.

**공개 저장소이므로 GitHub 로그인·토큰·SSH 키가 필요 없다.** Git 사용자 이름·이메일 설정도
필요 없다. Ubuntu PC에서 HTTPS로 내려받고, 터미널 메뉴로 설정·분류를 진행한다.

## 1. 로봇과 Ubuntu 환경 준비

| 위치 | 역할 |
|---|---|
| AI WORKER | 기존 bringup, MoveL 모드 Cyclo 제어기, RGB-D 카메라 실행 |
| 연결 Ubuntu PC | 이 저장소의 메뉴·인식·분류 실행 |

장비와 같은 ROS2 배포판 및 호환 ROBOTIS 인터페이스가 준비돼 있어야 한다.
이 문서는 ROS2 설치 자체나 카메라 외부 보정을 대신하지 않는다. 개발 검증은 Humble에서 했다.
장비가 Jazzy이면 해당 환경에서 빌드한다. 로봇의 기존 배포판을 바꾸지 않는다.

PC 터미널에서 ROS2 환경을 적용한다:

```bash
# Humble 예시. 장비가 Jazzy라면 humble을 jazzy로 바꾼다.
source /opt/ros/humble/setup.bash
# ROBOTIS 인터페이스가 별도 workspace에 있으면 해당 install/setup.bash도 source
```

아래는 환경 확인이 필요할 때만 실행하는 진단 명령이다:

```bash
ls /opt/ros
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION
command -v ros2
command -v colcon
ros2 pkg prefix robotis_interfaces
```

마지막 명령이 실패하면 로봇과 같은 버전의 `robotis_interfaces`를 먼저 설치/빌드한다.
공식 소스도 공개 HTTPS 주소로 로그인 없이 내려받을 수 있다:
`https://github.com/ROBOTIS-GIT/robotis_interfaces.git`.
PC에는 Cyclo 실행 파일이 없어도 되지만 MoveL 메시지 타입은 필요하다.

## 2. Git 설치와 공개 저장소 내려받기

Git이 없다면:

```bash
sudo apt update
sudo apt install git
```

여기서 `sudo`가 묻는 것은 **Ubuntu 사용자 비밀번호**이며 GitHub 비밀번호가 아니다.

```bash
cd "$HOME"
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd "$HOME/ROS2_WORKER"
```

공개 저장소이므로 일반적인 Ubuntu 환경에서는 로그인 없이 내려받는다.
기본 브랜치가 자동 선택되므로 브랜치 옵션도 필요 없다.
이미 폴더가 있으면 다시 clone하지 말고 7단계를 따른다.
컨테이너에서는 재생성해도 유지되는 볼륨에 소스·설정·빌드 결과를 둔다.

## 3. 의존성 확인과 최초 빌드

1단계에서 source한 같은 터미널에서:

```bash
cd "$HOME/ROS2_WORKER"
bash scripts/worker.sh build
```

필요한 항목은 ROS2 패키지(`package.xml`), NumPy·OpenCV·PyYAML·Ultralytics·호환 PyTorch다.
YOLO 가중치는 PC의 별도 로컬 파일로 준비한다. Git에는 가중치가 포함되지 않는다.
의존성 오류가 생겼을 때만 `bash scripts/worker.sh check`로 누락 항목을 확인한다.

기존 별도 Python 환경을 사용하는 경우에만 실행 전에 지정한다:

```bash
export WORKER_PYTHON=/absolute/path/to/existing/venv/bin/python
```

이 Python은 ROS2와 버전이 같아야 한다. 기본값은 `python3`다.
빌드 스크립트는 의존성을 자동 설치하거나 OS/CUDA/PyTorch를 교체하지 않는다.
빌드 결과는 `~/ros2_worker_ws`에 둔다. 소스와 연결해 설치하므로 `~/ROS2_WORKER`도 유지한다.

## 4. 현장 설정 생성

AI WORKER에서 기존 절차대로 공식 구동·MoveL 제어기·카메라를 실행한다.
PC와 로봇의 시간, `ROS_DOMAIN_ID`, RMW 및 DDS/Zenoh 구성을 맞춘다.
도메인 번호를 임의로 정하지 말고 로봇의 설정을 사용한다.

로봇 토픽 연결을 별도로 확인하고 싶을 때만 PC에서 실행한다:

```bash
ros2 node list
ros2 topic list -t
ros2 topic hz /joint_states
```

주기가 출력되면 Ctrl+C로 끝낸다. 이름이 다르면 실제 관절 토픽을 사용한다.
RGB·RGB에 정렬된 깊이·CameraInfo와 카메라/제어 링크 TF도 접근 가능해야 한다.
SSH/IP 연결만으로 ROS2 토픽이 자동 연결되는 것은 아니다.

```bash
cd "$HOME/ROS2_WORKER"
bash scripts/worker.sh menu
```

**`1 초기 설정`**을 선택한다.

1. 내장 모델이 없는 native 환경에서만 가중치 경로를 입력한다. 작업 범위는 저장값이 없을 때만 입력하고 추론 장치는 자동 선택한다.
2. 두 종류의 물건 전체와 빈 테이프 구역을 보이게 놓고 Enter를 누른다.
3. 기존 조그 기능으로 첫 번째 종류의 집기 자세를 지정하고 Enter를 누른다.
4. 같은 방향으로 두 번째 종류도 반복한다.
5. **팔을 작업물에서 떨어뜨린 뒤**, 집기 방향을 유지하고 Enter를 누른다.

물건·그리퍼 치수는 실측 숫자로 입력하지 않는다. 작업 범위와 카메라 좌표 보정은 필요하다.
RH-P12-RN 형상·장착 위치·TCP가 URDF·TF에 정확히 반영돼 있다고 가정한다.
모델에서 그리퍼 길이를 계산하므로 빈 테이블 기준 지정은 생략한다.
실행 PC에서 `robot_description`과 그리퍼 STL 자산을 읽을 수 있어야 하며, 없으면 설정을 중단한다.
물건을 특정 품목으로 제한하지 않으며, 현재 인식 이름을 자세 지정 때 함께 표시한다.
인식 노드 실행·스캔·설정 저장은 메뉴가 처리한다. 자세 조그는 사용자가 수행한다.

설정은 `~/ros2_worker_config/live.local.yaml`에 저장된다. 같은 장비·물건·집기 방향이면 재사용한다.
카메라 토픽·좌표계가 기본값과 다른 장비는 [고급 설정](ADVANCED_SETUP.md#4-현장-설정-생성)으로
로컬 설정을 먼저 만든다. 상세 화면 설명은 [터미널 메뉴 안내](TERMINAL_MENU.md)를 따른다.

## 5. 분류 시작·상태·취소

```text
2 물건·구역 확인
3 분류 시작
4 작업 상태
5 작업 취소
6 연결 검사
0 종료
```

**평소에는 `3 분류 시작`을 바로 선택한다.** 시작 과정에서 물건·구역을 자동 확인한다.
`2`는 작업을 시작하지 않고 배정만 미리 보고 싶을 때 사용한다. 처음에는 기본 2개만 처리한다.
팔은 작업물과 떨어져 있고 카메라를 가리지 않으며, 위로 이동할 공간이 있어야 한다.
그리퍼는 비어 있어야 한다. 고정 준비 자세나 미리 열린 그리퍼는 요구하지 않는다.

최종 확인에 `y`를 입력해야 실제 작업을 시작한다. 상태·배치 개수는 자동 갱신한다.
`c` + Enter로 취소를 요청하고 Enter로 메뉴에 돌아온다. 작업 ID를 입력할 필요는 없다.
취소 접수·실패 상태가 실제 정지를 뜻하지는 않는다. 상태가 불명확하면 실제 장비를 확인한다.

## 6. 다음부터 실행하기

새 터미널에서 최초 실행과 같은 ROS2/ROBOTIS·통신·Python 환경을 적용한 뒤:

```bash
# Humble 예시. 장비의 배포판으로 바꾼다.
source /opt/ros/humble/setup.bash
# 필요한 ROBOTIS overlay와 WORKER_PYTHON도 처음과 같게 적용
cd "$HOME/ROS2_WORKER"
bash scripts/worker.sh menu
```

설치·업데이트가 없으면 빌드하지 않는다. GitHub 로그인도 필요 없다.
로봇 공식 구동·카메라는 실행 중이어야 하며, 메뉴 외에 분류 서버를 중복 실행하지 않는다.

## 7. 로그인 없이 업데이트

작업이 끝나고 장비가 정지했음을 확인한 뒤 메뉴에서 `0 종료`를 선택한다.
같은 ROS2 환경의 터미널에서:

```bash
cd "$HOME/ROS2_WORKER"
git status --short
git pull
bash scripts/worker.sh build
bash scripts/worker.sh menu
```

로컬 코드를 수정했다면 pull 전에 별도로 백업/정리한다. 강제로 덮어쓰지 않는다.
로컬 설정·가중치는 저장소 밖에 보관하므로 업데이트로 덮어쓰지 않는다.

## 문제 확인

| 증상 | 확인 |
|---|---|
| Git 인증 오류 | 위 공개 HTTPS 주소·브랜치인지, 네트워크·프록시 설정 확인 |
| `check` 실패 | 실행 Python, source 순서, 누락 의존성 |
| 로봇 토픽이 안 보임 | domain·호환 middleware, 방화벽, DDS/Zenoh 경로 |
| 오래된 데이터 오류 | PC/로봇 시간 동기화와 센서 지연 |
| 초기 설정 실패 | 빈 구역 2개, 완전히 보이는 두 종류, 유효한 전체 깊이·TF |

로그는 `~/ros2_worker_config/menu-logs/`에 있다. 실물 집기는 아직 미검증이며
소프트웨어 검증 범위는 [검증 기록](RELEASE_CHECKS.md)에 있다.
