# ROS2 작업 서비스

현장 실행은 [Jazzy Docker 메뉴](../TERMINAL_MENU.md)를 사용한다.
메뉴가 세션·작업 ID와 시작 응답을 관리한다.
메뉴 외부의 실물 시작 명령은 작업 추적과 어긋나므로 함께 사용하지 않는다.

## 자동 분류 계약

| 서비스 | 요청 | 주요 응답 |
|---|---|---|
| `/worker/prepare_auto_sort` | 빈 요청 | `success`, `session_id`, `workspace_id`, `rules`, `message` |
| `/worker/start_task` | `session_id`, `max_objects` | `accepted`, `task_id`, `message` |
| `/worker/task_status` | `task_id` | `found`, `state`, `stage`, `message` |
| `/worker/session_status` | `session_id` | `found`, `workspace_id`, `rules`, `placed_count`, `slots_json`, `message` |
| `/worker/cancel_task` | `task_id` | `accepted`, `message` |

타입은 순서대로 `ai_worker_interfaces/srv/PrepareAutoSort`, `StartTask`, `TaskStatus`, `SessionStatus`, `CancelTask`다.
`max_objects=0`은 남은 물건 전체다. 한 번에 한 작업만 접수한다.
좌표·종류·구역·교시·제어기 계약을 검사하고 조건을 충족하지 못하면 동작을 거부한다.

`accepted=true`는 접수 결과이며 완료가 아니다.
상태는 `QUEUED`, `RUNNING`, `SUCCEEDED`, `FAILED`, `CANCELED`다.
취소 접수는 실제 정지를 보장하지 않는다. 제어 상태가 불확실하면 실패·오류 잠금으로 처리한다.
시작 응답이 끊기면 재전송하지 않고 기존 작업을 확인한다. ID는 프로세스 재시작 시 재사용하지 않는다.

## Docker 모의 데모에서 조회

[자동 분류 문서](AUTO_SORT.md)의 모의 컨테이너를 실행한 뒤 Ubuntu 호스트에서:

```bash
docker exec -it ros2-worker-demo /opt/worker/entrypoint.sh bash
```

다음은 **모의 컨테이너 안**에서 실행한다. ID는 데모 응답으로 바꾼다.

```bash
ros2 service call /worker/task_status ai_worker_interfaces/srv/TaskStatus \
  "{task_id: '반환된_작업_ID'}"
ros2 service call /worker/session_status ai_worker_interfaces/srv/SessionStatus \
  "{session_id: '반환된_세션_ID'}"
ros2 service call /worker/cancel_task ai_worker_interfaces/srv/CancelTask \
  "{task_id: '반환된_작업_ID'}"
```

실물에서는 메뉴의 `4 작업 상태`·`5 작업 취소`를 사용한다.

## 개발용 단발 서비스

`services.launch.py`의 `SortOnce`·`DetectObjects`는 별도 단발 개발 경로다.
기존 바구니 분류·Windows/Humble 시연은 [과거 기록](../docs/archive/HUMBLE_SERVICES.md)에 있다.
실물 연결 검증 범위는 [검증 기록](../RELEASE_CHECKS.md)을 따른다.
