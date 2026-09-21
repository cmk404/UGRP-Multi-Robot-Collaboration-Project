# Colab 시뮬레이션·평가 실행

2026-09-21 요청으로 앞으로 시뮬레이션·렌더링·교사 데이터 생성·학습·평가를 Colab에서 실행한다. 코드 편집·단위 테스트·Git·결과 검토와 실물 MasterPi 연결은 로컬에 남긴다. Colab 사용 불가 시 무거운 실험을 Mac에서 임의로 대신 시작하지 않는다.

## 시작

[실행 노트북](../notebooks/simulation_colab.ipynb)을 Colab에서 연다. 비공개 GitHub 접근과 Colab 로그인은 사용자 계정 권한이 필요하다. Chrome `강 / kcm0127@gmail.com`을 사용한다. Drive 저장 없이 실행하려면 [임시 노트북](https://colab.research.google.com/notebooks/empty.ipynb)에 셀을 복사한다. Drive 마운트·조회·동기화는 하지 않는다.

1. CPU 런타임으로 시작하고 Secrets에 `UGRP_GITHUB_TOKEN`을 추가한다. 저장소 읽기 권한만 있으면 된다. 토큰은 코드·출력·Git·ZIP에 쓰지 않는다.
2. `REF`를 이번 실험 브랜치로 지정한다. 노트북 기본값은 이 변경의 `codex/colab-simulation`; 병합 후 `main`으로 바꿀 수 있다. 얕은 clone도 추적 중인 실험 자료를 받으므로 다운로드 용량·시간이 발생한다. 실행 SHA를 출력하고 작업 도중 checkout을 변경하지 않는다. 입력 코드 변경은 먼저 커밋한다.
3. Python 3.12 독립 환경 `/content/ugrp-sim-env`에 시뮬레이션 의존성을 설치한다. CPU OSMesa가 기본이며 GPU EGL은 별도 실행 검증 대상이다. MuJoCo 물리 계산 자체의 GPU 가속을 주장하지 않는다.
4. 무료 `sim_quickstart`를 실행하여 `ok`, 이동 거리, 12프레임과 카메라 GIF를 확인한다. 세션은 작업이 끝나면 정리된다.
5. `run_job(module, args)`로 필요한 실행기·평가기만 실행한다. `{output}`은 새로운 작업 폴더의 `result` 경로로 치환된다. 실패도 `run.json`, `run.log`, 패키지 목록과 ZIP으로 남는다.
6. 각 작업의 ZIP과 SHA-256 파일을 내려받고 로컬에서 아래를 확인한다. 실제 로컬 파일 확인 전에는 회수 완료로 보고하지 않는다.

```sh
shasum -a 256 -c colab-실행ID.zip.sha256
unzip -l colab-실행ID.zip
```

`run.json`의 소스 SHA·명령·종료 코드와 결과 내부 평가를 확인한다. 종료 코드 0은 프로세스 정상 종료이며 로봇 성공을 뜻하지 않는다. 실험의 성공률·시간·행동 수·비용, 전체 실패 및 영상 검토 범위는 `experiments/<ID>/`에 별도로 남긴다. 원본 ZIP은 로컬 `outputs/`에 보존하며 로컬 보관을 원격 백업으로 표현하지 않는다.

## 실행 범위와 준비 조건

| 작업 | Colab 실행 경로 | 추가 조건 |
|---|---|---|
| 물리·카메라 데모 | `scripts.sim_quickstart` | 모델·GPU 불필요 |
| 두 로봇 RGB 교통 | `scripts.run_rgb_traffic` | `--out-dir {output}`; 별도 audit 필요 |
| 3대 fixture 장면 | `scripts.run_research_dispatch` | `--planner fixture --motion-smoke`; LLM 운반 성공 아님 |
| 실제 모델 계획·운반 | `scripts.run_dispatch_e2e`, 기존 실험 실행기 | Colab에서 닿는 인증된 endpoint, 모델 사용 권한, 입력 가중치와 데이터 |
| ACT 학습·평가 | 기존 학습기와 별도 ACT 환경 | `/content/ugrp-act-env`; 아래 버전 경계 준수 |
| 실물 MasterPi | 로컬 장치 네트워크 | Colab 이전 대상 아님 |

Mac의 `127.0.0.1` 프록시는 Colab에서 접근할 수 없다. 인증되지 않은 공개 터널을 자동으로 만들지 않는다. 모델 endpoint와 비밀값은 해당 클라이언트가 지원하는 환경변수/Secrets에 전달하며 출력하거나 결과에 저장하지 않는다. 로컬 전용 raw 데이터·모델은 clone에 포함되지 않으므로 명시적으로 업로드하고 원본 해시와 경로를 검증한다. 기존 실행기의 Mac Python 기본 경로는 `--help`를 확인하여 `/content/ugrp-sim-env/bin/python` 또는 ACT Python으로 지정한다.

ACT의 `requirements-reference-act.txt`와 시뮬레이션 requirements는 NumPy/OpenCV 버전이 다르므로 합쳐 설치하지 않는다. ACT 환경을 따로 만들고 `scripts/patch_reference_act.py`를 실행한 뒤 CUDA·실제 forward/backward·체크포인트 검사를 수행한다. 최근 입력 비교 학습/재개 노트북은 [PR #84](https://github.com/kcm0127-dotcom/ugrp/pull/84)의 별도 변경이다. 이 PR은 그것을 병합하거나 기존 학습 세션을 중단하지 않는다.

## 세션 중단과 데이터 보존

Colab의 VM 수명·유휴 종료·GPU 가용성은 변동되며 장시간 상시 실행을 보장하지 않는다([공식 FAQ](https://research.google.com/colaboratory/faq.html)). 작업을 조건별 유한 실행으로 나누고 각 작업 종료 때 결과를 내려받는다. VM 삭제 전 다운로드하지 못한 결과는 잃을 수 있다. 런타임 강제 삭제·프로세스 강제 종료 시 finally 블록과 자동 ZIP 생성도 보장하지 않는다.

일반 시뮬레이션의 중간 물리 상태 복원은 제공하지 않는다. 완료 조건은 보존하고 실패·미완료 조건을 새 출력 폴더에서 다시 실행한다. 학습기 전용 checkpoint 재개는 소스·데이터·설정·환경 해시가 맞는 경우에만 사용한다. 자동 재접속·유료 자원 구매·상시 브리지·기존 Mac 프로세스 중단은 포함하지 않는다.

## 검증 상태

로컬에서 `tests/test_colab_simulation.py`와 `tests/test_ugrp_session.py` 12개가 통과했다. 정상/실패 실행의 소스·종료 코드·ZIP 해시 보존, 기존 출력 보호, 수정된 소스 거부, 세션 정리를 확인했다. 노트북 코드 셀도 컴파일했다. GitHub Linux CI는 새 실행기로 물리 이동·렌더링·ZIP 해시를 확인하도록 연결했다. 이 검사는 실제 Colab VM 검증과 구분한다. Colab의 로그인·런타임이 아직 확인되지 않았으므로 실제 Colab 설치·물리 렌더링·학습·전체 코호트·결과 다운로드는 미검증이다. 무료 데모 확인 후에만 모델 비용이 발생하는 전체 비교로 확장한다. 로봇 관측 제한, 교사/정적 지도 예외, 카메라 FOV와 weld OFF는 기존 지침을 유지한다.
