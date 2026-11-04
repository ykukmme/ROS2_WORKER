# ROS2_WORKER

ROBOTIS AI WORKER용 ROS2 물건 분류 프로토타입.
YOLO가 구분한 두 종류를 색 테이프 구역에 자동 배정하고 같은 종류를 같은 구역에 놓도록 구성한다.
VLA/VLM은 사용하지 않는다.

**연결 PC에서 Git으로 설치·실행하려면 [DEPLOYMENT.md](DEPLOYMENT.md)를 따른다.**
공개 저장소이므로 Ubuntu에서 GitHub 로그인·토큰·SSH 키 없이 내려받고 업데이트할 수 있다.
실제 RGB-D·YOLO·ROBOTIS MoveL·그리퍼 피드백을 연결하는 실행 코드를 포함한다.
실물 장비 접속 없이 컨트롤러 에뮬레이터로 검증했으므로 카메라 보정과 동작은 현장에서 확인해야 한다.
일반 데모는 `dry_run: true`이며, 실물 launch는 별도 측정 설정을 요구한다.

| 디렉터리 | 내용 |
|---|---|
| `ai_worker_interfaces` | 작업·취소·구역·자동 분류 ROS2 서비스/메시지 |
| `ai_worker_sorting` | YOLO 영상 연결, RGB-D 좌표 계산, 구역 검출, 작업 서버 |
| `scripts` | 기존 ROS2 환경에서 독립 빌드하는 도구 |

## 연결 PC에서 내려받기

로봇과 통신하는 Ubuntu/ROS2 PC의 Bash에서 실행한다. ROS2 배포판·네트워크 설정을 로봇과 맞춘다.
Windows PC라면 로봇의 기존 Linux 개발 컨테이너에 SSH 접속해 같은 절차를 실행할 수도 있다.

```bash
GIT_TERMINAL_PROMPT=0 git -c credential.helper= clone --branch codex/initial-release https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
# 로봇에서 사용하는 ROS2 환경을 먼저 source한 상태여야 한다.
bash scripts/worker.sh check
bash scripts/worker.sh build
bash scripts/worker.sh menu
```

기존 로봇 패키지를 덮어쓰거나 OS/CUDA/PyTorch를 자동 업그레이드하지 않는다.
설정 생성 → 연결 진단 → 실행 → 작업 시작 → Git 업데이트 명령은 [DEPLOYMENT.md](DEPLOYMENT.md)에 있다.
**평소 사용은 [터미널 메뉴 안내](TERMINAL_MENU.md)를 따른다.** 메뉴가 인식 노드·설정·작업 ID를 관리한다.
AI WORKER에는 공식 제어기·카메라가 실행 중이어야 한다. PC에서는 `doctor`로 연결을 검사한다.
물건·그리퍼 치수는 직접 입력하는 대신 카메라 스캔과 1회 자세 교시로 설정할 수 있다.

## 모의 분류

```bash
export ROS_DOMAIN_ID=79
ros2 launch ai_worker_sorting auto_sort.launch.py mock_mode:=true
```

같은 ROS2 환경·도메인의 다른 터미널:

```bash
ros2 service call /worker/prepare_auto_sort ai_worker_interfaces/srv/PrepareAutoSort '{}'
ros2 service call /worker/start_task ai_worker_interfaces/srv/StartTask "{session_id: '응답의_세션_ID', max_objects: 0}"
```

`accepted`는 작업 접수 결과다. 완료 여부는 `/worker/task_status`로 확인한다.

## 문서

- [로봇 설치·업데이트](DEPLOYMENT.md)
- [실제 AI WORKER 실행](ai_worker_sorting/LIVE_ROBOT.md)
- [자동 색 구역 분류](ai_worker_sorting/AUTO_SORT.md)
- [작업 서비스](ai_worker_sorting/WORKER_SERVICES.md)
- [Windows 웹캠 → Docker ROS2](ai_worker_sorting/LAPTOP_ROS2.md)

하위 문서의 `/tmp/worker_sorting_*`, Windows 경로는 개발 PC 실험 환경이다.
새 로봇에서는 이 문서와 `DEPLOYMENT.md`의 경로를 사용한다.
실험 결과·영상·인증 파일·모델 가중치는 Git에 포함하지 않는다.

## 검증

- Docker ROS2 Humble/Python 3.10 단위 테스트 57개.
- Docker ROS2 Humble/Python 3.10에서 패키지 2개 빌드와 서비스 통합 시나리오 18개.
- 실제 노트북 웹캠 → ROS2 → YOLO 영상 연결 검증.
- 공식 MoveL·그리퍼 메시지와 PC 실행 도구·메뉴 백엔드로 13개 시나리오 검증(제어기 에뮬레이터).
- 치수 입력 없는 카메라 스캔·자세 교시·설정 생성 5개 시나리오 검증.
- 실제 로봇 장비에서의 ROS2 배포판·센서·컨트롤러 호환성은 아직 미검증.

단위 테스트에는 NumPy, OpenCV, PyYAML이 필요하다.

```bash
cd ai_worker_sorting
python3 -m unittest discover -s tests -v
```

코드 라이선스는 패키지 선언에 따라 Apache-2.0이다. YOLO 라이브러리·가중치 등
별도 구성요소의 라이선스는 각각의 배포 조건을 따른다.
