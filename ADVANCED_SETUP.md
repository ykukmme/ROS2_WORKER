# 고급 설정: 개별 명령

일반 사용자는 [Ubuntu 설치·메뉴 가이드](DEPLOYMENT.md)를 따른다. 아래는 커스텀 토픽·좌표계 또는 개별 명령이 필요한 경우의 절차다.

## 4. 현장 설정 생성

**물건 크기·높이·그리퍼 길이를 숫자로 입력하지 않는 방법이다.**
카메라 스캔 후, 기존 조그 조작으로 그리퍼 기준 위치와 두 종류의 집기 자세를 한 번씩 지정한다.
이 도구는 팔·그리퍼 명령을 보내지 않는다. 자동 분류 worker는 종료한 상태여야 한다.
작업 범위 `WORKSPACE`와 카메라↔로봇 좌표 보정은 여전히 필요하다.

```bash
export WORKER_CONFIG="$HOME/ros2_worker_config/live.local.yaml"
STATE="$HOME/ros2_worker_config/teaching.local.yaml"
WORKSPACE='xmin,xmax,ymin,ymax,zmin,zmax'
WEIGHTS='/absolute/path/on/this/pc/best.pt'
# 첫 터미널: 인식·구역 노드만 실행. 카메라 드라이버는 로봇에서 실행 중이어야 함.
bash scripts/worker.sh calibrate weights:="$WEIGHTS" device:=cpu
```

두 번째 터미널에서도 같은 ROS2 환경·위 변수들을 설정한다. 수평인 같은 테이블에
빈 빨강·파랑 구역을 놓고, 팔을 치운 상태에서 두 종류의 물건 전체가 겹치지 않게 보이도록 한다.
각 종류에 속한 물건 중 가장 큰 것도 함께 보여준다.

```bash
bash scripts/worker.sh teach scan --state "$STATE" \
  --weights "$WEIGHTS" --workspace="$WORKSPACE" --device cpu
```

출력된 YOLO 종류 이름 두 개를 확인한다. 이제 물건·테이블·카메라는 움직이지 않는다.
로봇의 기존 조그 기능으로 위에서 집는 방향을 정한 뒤, 빈 테이블 위에 그리퍼를 둔다.
**작업 중 가장 아래로 내려오는 그리퍼 부분을 테이블 높이에 맞추고** 아래 명령으로 기록한다.
개폐에 따라 최저점이 달라지면 각각 같은 방향으로 맞춰 `tool`을 다시 기록한다. 가장 큰 길이를 보존한다.

```bash
bash scripts/worker.sh teach tool --state "$STATE" --tips-at-table
```

같은 방향을 유지하며 첫 종류의 물건을 실제로 집을 위치로 조그한다.
테이블과 닿지 않고 물건을 잡을 수 있는 자세인지 확인한 뒤 기록한다.
두 번째 종류도 반복한다. 물건은 특정 품목으로 제한하지 않고 첫 번째 종류·두 번째 종류로 안내한다.
현재 CLI의 아래 이름에는 `scan`에서 반환된 실제 인식 이름을 사용한다.

```bash
bash scripts/worker.sh teach grasp --state "$STATE" --class-name '첫_종류' --grasp-pose-confirmed
bash scripts/worker.sh teach grasp --state "$STATE" --class-name '둘째_종류' --grasp-pose-confirmed
# 두 종류 기록 후 팔을 작업물에서 떨어뜨린다. 집기 방향은 그대로 유지한다.
bash scripts/worker.sh teach write --state "$STATE" --output "$WORKER_CONFIG"
```

시작 안내는 **“팔을 작업물에서 떨어뜨린 뒤 시작하세요”**로 한다. 특정 고정 준비 자세는
요구하지 않는다. 물건·구역을 카메라에서 가리지 않고, 위로 이동할 공간이 있으며,
그리퍼에 물건을 들고 있지 않은 상태인지 확인한다. 그리퍼는 작업 시작 시 코드가 연다.

카메라 크기·높이에 여유값을 더하고, 테이블 기준 그리퍼 길이와 종류별 집기 오프셋을
자동 계산한다. 교시는 30분 안에 완료한다. 설정/교시 파일이 이미 있으면 새 경로로 시작한다.
탐지 영역 전체 깊이에서 최고점을 구한다. 깊이가 한 부분이라도 누락되거나 물건이 가려지거나
얇으면 설정을 거부한다. 보이지 않은 물건의 크기까지 추정하지 않는다.
작업 중 관측한 크기가 설정 범위를 넘으면 멈춘다. 종류·집는 방향·그리퍼가 바뀌면 다시 교시한다.

첫 터미널의 `calibrate`를 Ctrl+C로 종료한 뒤:

```bash
bash scripts/worker.sh doctor
```

`doctor`는 명령을 발행하지 않고 PC에서 실제 RGB·깊이·카메라 정보·관절·TF와 Cyclo
설정에 접근 가능한지 확인한다. PASS는 연결 진단이며 실제 좌표 정확도나 집기 성공 보장은 아니다.
카메라·프레임이 기본값과 다르면 `calibrate` launch의 해당 인자와 `teach scan`의
`--color-topic`, `--depth-topic`, `--camera-info-topic`, `--base-frame`을 같은 값으로 지정한다.
이미 정확한 치수를 알고 있다면 기존 [숫자 설정 방식](ai_worker_sorting/LIVE_ROBOT.md#3-실제-피드백에서-설정-생성)도 사용할 수 있다.

## 5. 실행과 작업 명령

첫 터미널에서:

```bash
bash scripts/worker.sh run
```

PC에서는 기본 `camera_driver:=external`을 유지해 로봇 카메라 토픽을 사용한다.
노드 실행만으로 팔은 움직이지 않는다. 두 개의 빈 테이프 구역과 떨어진 두 종류의 물건을 준비한다.

두 번째 터미널에서도 2단계의 ROS2 환경과 같은 `WORKER_WS`, `WORKER_CONFIG`,
필요 시 `WORKER_PYTHON`을 설정하고 같은 Git 폴더로 이동한다.

```bash
bash scripts/worker.sh prepare
# 반환된 session_id를 입력. 종류→구역 배정 rules를 확인한다.
bash scripts/worker.sh start --session-id '반환된_session_id' --max-objects 2
# start의 accepted는 접수 결과이며 실제 완료는 status로 확인한다.
bash scripts/worker.sh status --task-id '반환된_task_id'
bash scripts/worker.sh session --session-id '반환된_session_id'
```

`start`가 실제 작업 명령이다. 처음에는 물체 2개만 처리하고, 검증 후 `--max-objects 0`으로
남은 물체를 처리한다. 작업 제한 시간은 기본 600초이며 더 긴 작업은 YAML에서 조절한다.
응답이 끊겨도 `start`를 자동 재전송하지 않는다. 기존 작업을 확인한 뒤 진행한다.

```bash
bash scripts/worker.sh cancel --task-id '반환된_task_id'
bash scripts/worker.sh status --task-id '반환된_task_id'
```

취소 접수는 정지 완료가 아니다. 상태와 실제 장비를 확인한다. 네트워크가 끊기면 PC의
취소 명령도 로봇에 도달하지 못하므로 장비의 비상정지를 사용한다.
정상 종료는 작업이 끝난 뒤 첫 터미널에서 Ctrl+C로 한다.

## 6. Git 업데이트

진행 중 작업이 없음을 확인하고 프로그램을 종료한 뒤:

```bash
git status --short
git pull
bash scripts/worker.sh build
bash scripts/worker.sh doctor
bash scripts/worker.sh run
```

로컬 소스 수정이 있으면 pull 전에 별도 커밋/백업으로 정리한다. 강제로 덮어쓰지 않는다.
로컬 설정과 가중치는 Git 밖에 유지한다. 재시작하면 session/task ID는 다시 발급받는다.
`msg`/`srv`가 바뀌면 이 사용자 정의 인터페이스를 쓰는 다른 PC도 같은 커밋으로 재빌드한다.
현재 구성에서는 공식 로봇 구동 코드는 이 저장소의 사용자 정의 서비스를 사용하지 않는다.

## 연결 문제 확인

| 증상 | 확인 |
|---|---|
| `check` import 실패 | 실행 Python, source 순서, 누락 ROS 패키지·Ultralytics |
| 로봇 노드·토픽이 안 보임 | 동일 domain·호환 middleware, DDS/Zenoh 경로, 방화벽, 컨테이너 네트워크 |
| 노드는 보이지만 영상이 안 옴 | `ros2 topic hz`로 실제 데이터 확인, 대역폭·QoS·depth 정렬 |
| stale/clock 오류 | PC와 로봇 시간 동기화, 센서 지연 |
| TF/제어 링크 오류 | 실제 카메라 외부 보정·Cyclo `l_gripper_name`·기준 좌표계 |
| doctor PASS 후 prepare 실패 | 빈 목적 구역 2개, source ROI, 두 종류 YOLO 탐지, 테이프 색 |

실제 장비에는 아직 접속하지 않았다. 소프트웨어 검증 범위는 [검증 기록](RELEASE_CHECKS.md)에 있다.
