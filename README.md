# ROS2_WORKER

ROBOTIS AI WORKER용 ROS2 물건 분류 프로토타입.
YOLO가 구분한 두 종류를 색 테이프 구역에 자동 배정하고 같은 종류를 같은 구역에 놓도록 구성한다.
VLA/VLM은 사용하지 않는다.

**현재 검증 범위는 모의 제어다. 실제 AI WORKER의 카메라 보정·팔 동작은 현장 연결이 필요하다.**
기본값은 `dry_run: true`, `mock_mode: false`이며 클론·빌드만으로 팔이 움직이지 않는다.

| 디렉터리 | 내용 |
|---|---|
| `ai_worker_interfaces` | 작업·취소·구역·자동 분류 ROS2 서비스/메시지 |
| `ai_worker_sorting` | YOLO 영상 연결, RGB-D 좌표 계산, 구역 검출, 작업 서버 |
| `scripts` | 기존 ROS2 환경에서 독립 빌드하는 도구 |

## AI WORKER에서 내려받기

로봇의 기존 ROS2 개발 컨테이너/터미널에서 실행한다.

```bash
git clone https://github.com/ykukmme/ROS2_WORKER.git
cd ROS2_WORKER
# 로봇에서 사용하는 ROS2 환경을 먼저 source한 상태여야 한다.
bash scripts/build_ros2.sh "$HOME/ros2_worker_ws"
source "$HOME/ros2_worker_ws/install/ros2_worker/setup.bash"
```

기존 로봇 패키지를 덮어쓰거나 OS/CUDA/PyTorch를 자동 업그레이드하지 않는다.
설치 전제·업데이트·모의 테스트·실물 설정은 [DEPLOYMENT.md](DEPLOYMENT.md)를 따른다.

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
- [자동 색 구역 분류](ai_worker_sorting/AUTO_SORT.md)
- [작업 서비스](ai_worker_sorting/WORKER_SERVICES.md)
- [Windows 웹캠 → Docker ROS2](ai_worker_sorting/LAPTOP_ROS2.md)

하위 문서의 `/tmp/worker_sorting_*`, Windows 경로는 개발 PC 실험 환경이다.
새 로봇에서는 이 문서와 `DEPLOYMENT.md`의 경로를 사용한다.
실험 결과·영상·인증 파일·모델 가중치는 Git에 포함하지 않는다.

## 검증

- Windows/Python 3.11 단위 테스트 39개.
- Docker ROS2 Humble/Python 3.10에서 패키지 2개 빌드와 서비스 통합 시나리오 18개.
- 실제 노트북 웹캠 → ROS2 → YOLO 영상 연결 검증.
- 실제 로봇 장비에서의 ROS2 배포판·센서·컨트롤러 호환성은 아직 미검증.

단위 테스트에는 NumPy와 OpenCV가 필요하다.

```bash
cd ai_worker_sorting
python3 -m unittest discover -s tests -v
```

코드 라이선스는 패키지 선언에 따라 Apache-2.0이다. YOLO 라이브러리·가중치 등
별도 구성요소의 라이선스는 각각의 배포 조건을 따른다.
