# AI WORKER 분류 패키지

기본 경로는 Jazzy Docker의 RGB-D·YOLO·ROBOTIS MoveL 자동 분류다.
두 종류를 색 테이프 구역에 배정하며 VLA/VLM은 사용하지 않는다.
현장 사용은 [공용 PC 안내](../docker/SHARED_PC.md)와 [터미널 메뉴](../TERMINAL_MENU.md)를 따른다.
이미지 제작·업데이트는 [Docker 가이드](../DOCKER.md)에 있다.

## 구성

| 노드·모듈 | 역할 |
|---|---|
| `rgbd_input` | 색·정렬 깊이·CameraInfo 동기화와 보정 |
| `rgbd_detector` | 로컬 YOLO 추론 |
| `vision_service` | 깊이·TF로 물건 좌표 계산 |
| `zone_service` | 색 테이프 구역 검출 |
| `auto_worker` | 두 종류 배정·빈자리 예약·분류 |
| `robotis_robot` | MoveL·그리퍼 명령과 피드백 확인 |
| `terminal_menu` | 교시·설정·시작·상태·취소 |
| `configure_auto`·`tool_model` | 스캔·종류별 자세·URDF 형상 계산 |

카메라 외부 보정과 올바른 URDF·TF는 현장에 준비돼 있어야 한다.
양팔 협업·손목 카메라 보정·임의 물체 집기 자세 생성은 포함하지 않는다.
두 종류는 YOLO가 구별하며 같은 그리퍼 방향으로 집을 수 있어야 한다.

## 상세 문서

- [실물 연결](LIVE_ROBOT.md)
- [자동 분류와 Docker 모의 데모](AUTO_SORT.md)
- [작업 서비스](WORKER_SERVICES.md)
- [Docker 고급 설정](../ADVANCED_SETUP.md)
- [검증 기록](../RELEASE_CHECKS.md)
- [과거 프로토타입](../docs/archive/LEGACY_PROTOTYPE.md)
- [과거 Windows 웹캠 실험](LAPTOP_ROS2.md)

단발 바구니 분류·MoveIt·웹캠 브리지는 이전 개발 경로로 보존한다.
일상 사용은 Docker 메뉴로 진행한다.
