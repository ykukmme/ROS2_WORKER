# 배포 관리자 안내

공개 이미지 주소는 `ghcr.io/ykukmme/ros2-worker:jazzy`다. 배포 대상은 Linux amd64 / Jazzy / CPU다.
Ubuntu 사용자에게 패키지 토큰이나 Docker 로그인을 요구하지 않는다.

현재 패키지는 공개 상태이며, [게시 실행](https://github.com/ykukmme/ROS2_WORKER/actions/runs/44248368635)에서 자동 테이프 색 검출 이미지 게시와 익명 접근 검사를 통과했다.
일반 Ubuntu 사용자는 아래 최초 공개 설정을 수행할 필요가 없다.
검증한 소스는 `ff52dcbb1d42e9d0f99d14e295a2d7e5fb9815fd`이며 이미지 digest는
`sha256:c7a4c7cdb09eb061a17a0ffb3d67f6cea9d1268a338419a81f38372ac53f8016`이다.

## 자동 배포

`.github/workflows/publish-image.yml`은 기본 브랜치 `codex/initial-release`의 이미지 관련 변경과 수동 실행에 반응한다.
공식 checkout Action은 확인한 커밋 SHA로 고정한다.
GitHub의 Ubuntu 24.04 실행기에서 다음을 순서대로 수행한다:

1. 이미지를 빌드한다.
2. 실행기 회귀·볼륨·의존성 검사와 이미지 내부 단위 테스트, 제어기/교시 에뮬레이터 검증을 수행한다.
3. 통과한 이미지를 `sha-전체커밋해시` 태그로 업로드한다.
4. 같은 이미지에 `jazzy` 태그를 지정한다.
5. 별도 권한 없는 작업에서 익명 manifest 접근을 검사한다.

실패한 빌드·테스트는 업로드 단계로 진행하지 않는다.
인증에는 작업 전용 `GITHUB_TOKEN`과 `packages: write`를 사용하며 별도 PAT를 저장하지 않는다.
비밀 값이나 현장 볼륨은 이미지에 포함하지 않는다.
공식 제어기·카메라 드라이버는 로봇 측 기존 환경을 사용한다.

## 최초 공개 설정

개인 계정에 처음 생성한 GHCR 패키지는 기본적으로 비공개다.
공개 Git 저장소와 연결돼 있어도 패키지 공개 여부는 별도다.

첫 업로드 후 GitHub 계정의 Packages에서 `ros2-worker`를 열고
Package settings → Change visibility → Public을 선택한다.
이 설정은 배포 계정에서 한 번 수행하며, Ubuntu 실행 PC의 로그인 절차와는 관계없다.
공개 전에는 `verify-public` 작업이 실패한다. 공개 설정 후 해당 실패 작업만 다시 실행한다.

```bash
gh run rerun 실행_ID --failed
```

실제 배포 완료 기준은 빌드·검사·업로드와 `verify-public`이 모두 통과하는 것이다.
사용자 PC와 같은 로그인 없는 Docker 설정으로 실제 pull과 메뉴도 확인한다.

## 업데이트와 버전 고정

사용자는 장비 정지·메뉴 종료 후 `bash worker.sh update`로 새 이미지를 받는다.
일반 실행은 기존 로컬 이미지를 재사용한다.
특정 배포 버전을 재현할 때는 `WORKER_IMAGE`에 GHCR의 `sha-전체커밋해시` 태그나 digest를 지정한다.
로컬 개발 이미지와 GPU 이미지는 일반 CPU 공개 이미지와 구분한다.

## 공식 참고

- [GitHub 이미지 게시](https://docs.github.com/en/actions/tutorials/publish-packages/publish-docker-images)
- [GHCR 공개·익명 접근](https://docs.github.com/en/packages/learn-github-packages/configuring-a-packages-access-control-and-visibility)
