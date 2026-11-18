# 두 종류 자동 구역 분류

현장 실행은 [Docker 메뉴](../TERMINAL_MENU.md)를 사용한다.
`2`는 배정 미리 보기, `3`은 확인 후 시작이다. 종류·구역·작업 ID를 직접 입력하지 않는다.

## 분류 규칙

1. 색을 사전에 지정하지 않고 영상에서 서로 다른 색의 닫힌 테이프 구역 두 개를 검출한다.
2. 집기 영역에서 YOLO가 구별한 두 종류를 선택한다.
3. 종류명과 구역을 정렬해 일대일 배정하고 세션 동안 유지한다.
4. 같은 종류를 해당 구역의 서로 다른 빈자리에 놓는다.
5. 새 종류·구역 포화·취소·좌표 오류가 발생하면 후속 동작을 중단한다.

특정 테이프 색 목록은 사용하지 않는다. 감지 구역은 `zone_1`·`zone_2`로 표시하며 실제 색상과 화면 위치를 메뉴에서 확인한다.
두 색이 너무 비슷하거나 배경 대비가 부족하면 거부한다. 닫힌 사각 구역 후보가 세 개 이상이어도 임의로 두 개를 선택하지 않는다.
기존 YAML의 `zone_colors`는 이전 설정 호환을 위해 받아들이지만 감지할 색을 제한하지 않는다. 새 설정은 `[auto]`로 저장한다.
YOLO가 같은 라벨로 인식한 물건을 다른 종류로 구분하지는 않는다.
교시 범위를 넘는 크기·높이, 가림·깊이 누락은 거부한다.
목적 구역 안의 물건을 다시 집지 않으며 파지·배치를 관측으로 확인한다.
실물 전제는 [실물 연결](LIVE_ROBOT.md)을 따른다.

## 서비스

| 서비스 | `ai_worker_interfaces/srv` 타입 |
|---|---|
| `/workspace/scan_zones` | `ScanZones` |
| `/worker/prepare_auto_sort` | `PrepareAutoSort` |
| `/worker/start_task` | `StartTask` |
| `/worker/session_status` | `SessionStatus` |
| `/worker/task_status` | `TaskStatus` |
| `/worker/cancel_task` | `CancelTask` |

[서비스 계약](WORKER_SERVICES.md)을 참고한다. 실물 메뉴 외부에서 시작 서비스를 함께 호출하지 않는다.

## 개발용 Docker 모의 데모

로봇 네트워크와 연결하지 않는 별도 컨테이너다. 일반 메뉴·설정 볼륨을 사용하지 않는다.
이미지가 등록된 Ubuntu 호스트에서:

```bash
docker run --rm -it --name ros2-worker-demo --network none \
  --tmpfs /data:rw,mode=1777 -e ROS_DOMAIN_ID=79 \
  ghcr.io/ykukmme/ros2-worker:jazzy ros2 launch ai_worker_sorting auto_sort.launch.py \
  mock_mode:=true python_prefix:=/opt/worker_venv/bin/python
```

내장 `auto_sort.yaml`의 `dry_run: true`를 사용한다.
다른 호스트 터미널에서 같은 컨테이너 콘솔을 연다:

```bash
docker exec -it ros2-worker-demo /opt/worker/entrypoint.sh bash
```

다음은 **모의 컨테이너 안**에서만 실행한다:

```bash
ros2 service call /worker/prepare_auto_sort ai_worker_interfaces/srv/PrepareAutoSort '{}'
ros2 service call /worker/start_task ai_worker_interfaces/srv/StartTask \
  "{session_id: '응답의_세션_ID', max_objects: 0}"
```

응답의 `task_id`로 상태를 조회한다. 조회·취소는 [서비스 문서](WORKER_SERVICES.md)를 따른다.
종료는 첫 터미널에서 Ctrl+C다.
이전 Windows/Humble 실행은 [보관 문서](../docs/archive/HUMBLE_AUTO_SORT.md)에 있다.
