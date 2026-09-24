# 학습 모델 다운로드·검증·배포

학습 코드와 가중치는 함께 버전을 관리한다. 저장소의 [모델 목록](../configs/model_artifacts.json)은 모델 ID, 원래 학습/실행 소스, Release 주소, 파일별 크기·SHA-256과 검증 범위를 연결한다. 큰 가중치는 GitHub Release asset으로 배포하고 Git 이력에 직접 넣지 않는다. [GitHub Releases](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)는 파일별 2 GiB 미만의 바이너리 배포를 지원한다.

## 현재 제공 범위

첫 배포 `models-20260923-v1`은 다음 두 **과거 실험 모델**이다. 표준 RGB 제어기를 ACT로 자동 교체하지 않는다.

| 모델 ID | 내용 | 판정 범위 |
| --- | --- | --- |
| `act-pair-carry-seed18-20260918` | 20260918 seed의 128px·chunk8 ACT, config/adapter, RGB grasp/stage, 저장 계획·TOP 기준 영상 | [2026-09-18 비교](../experiments/2026-09-18-act-pair-carry/README.md)의 가중치; 최신 모델이나 일반화된 성공 모델이 아님 |
| `act-pair-carry-seed19-20260919` | 같은 구성, 20260919 seed | 실패한 비교 모델도 원래 가중치로 보존 |
| `act-expanded-20260922` | 최종 제어기 검증에서 사용한 128px/history4 모델 | `unavailable`: 기록된 원본을 아직 찾지 못함. 위 두 모델로 대체하지 않음 |
| `act-action-first-study-seed24-20260924` | 128px/history4·chunk8 ACT, 8,000회 학습 중 step 1,500 선택 | 개발 4/4 에피소드 종료 누락, 새 물리 비교 0/2 완료. 실패 비교 모델을 보존하며 기본 제어기로 쓰지 않음 |

각 ZIP에는 가중치·설정·보조 자산·출처를 합쳐 18개 파일이 있다. 학습 데이터 전체와 raw 실험 영상은 포함하지 않는다. 다운받았다는 사실은 운반 성공이나 과거 전체 실험 재현을 뜻하지 않는다. 과거 실행의 소스·조건·분모는 원 실험 보고서를 따른다.

2026-09-23 공개 Release에서 두 ZIP을 다시 내려받아 archive와 내부 36개 파일의 크기·해시를 확인했다. 기존 Mac ACT 환경에서 각각 `pair_carry_act_worker`의 `ready: true`와 정상 종료를 확인했다. 새 물리 운반 시험은 수행하지 않았다. 배포 원본·GitHub asset 검증·다운로드 및 로딩 기록은 배포 작업의 로컬 `outputs/model-release-20260923-v1/`에 보존한다.

## 내려받기

Python 3.12와 프로젝트 checkout에서 실행한다. 이 단계는 PyTorch·MuJoCo 설치, 모델 계정, 시뮬레이션 실행을 요구하지 않는다. 네트워크 다운로드는 `fetch`를 명시했을 때만 수행한다.

```sh
python3 -m scripts.sim_cli models list
python3 -m scripts.sim_cli models show act-pair-carry-seed18-20260918
python3 -m scripts.sim_cli models fetch act-pair-carry-seed18-20260918
python3 -m scripts.sim_cli models verify act-pair-carry-seed18-20260918
```

기본 위치는 `outputs/models/<모델 ID>/`다. `fetch --output DIR`, `verify --path DIR`로 다른 위치를 선택한다. ZIP과 내부의 모든 파일을 크기·해시로 확인한 뒤 새 폴더로 설치하며 기존 폴더를 덮어쓰지 않는다. 이미 같은 자산이 있으면 다시 검증한다. `--archive LOCAL_ZIP`은 따로 내려받은 원본 ZIP에도 같은 검사를 적용한다. 누락된 모델을 요청하면 명확히 거절한다.

GitHub의 [Release 페이지](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/releases/tag/models-20260923-v1)에는 동일 ZIP·모델 목록·`SHA256SUMS`를 함께 제공한다. CLI가 아직 포함되지 않은 checkout에서는 Release에서 ZIP을 직접 받을 수 있다. 새 다운로드 도구의 PR 병합과 Release asset 업로드는 별도 상태다.

새 action-ACT 모델은 별도 [비교 모델 Release](https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project/releases/tag/models-20260924-action-act-v1)에 보존한다. 이 태그에는 해당 모델을 등록한 CLI 코드도 포함되므로 main 병합 전에도 다음처럼 받을 수 있다. 태그의 배포 코드와 실제 학습 소스 `12f8e6dda76e39b3ec612f247deb4835a2ff50bc`는 별도로 기록한다.

```sh
git clone --branch models-20260924-action-act-v1 --depth 1 \
  https://github.com/cmkang131/UGRP-Multi-Robot-Collaboration-Project.git ugrp-action-model
cd ugrp-action-model
python3 -m scripts.sim_cli models fetch act-action-first-study-seed24-20260924
python3 -m scripts.sim_cli models verify act-action-first-study-seed24-20260924
```

두 번째 학습은 모델 내보내기에 실패했으므로 내려받을 모델이 없다. 첫 모델로 그 결과를 대체하지 않는다. 모델마다 포함 자산과 검증 범위는 목록·ZIP의 `provenance.json`·Release 검증 기록을 함께 확인한다.

## 실제 ACT 로딩

ACT의 선택 의존성은 [requirements-reference-act.txt](../requirements-reference-act.txt)에 고정돼 있다. 기존 ACT 환경이 있으면 재사용한다. 새 팀원의 설치는 [Ubuntu 안내](ubuntu_quickstart.md)의 환경 분리를 따른다. Mac의 기존 환경은 `/Users/changmin/Project-Runtimes/ugrp/.venv-reference-act`다.

ACT 환경의 Python으로 다음 명령을 실행하면 가중치·설정·adapter를 실제로 로딩하고 `ready: true`를 반환한 뒤 종료한다.

```sh
python -m scripts.pair_carry_act_worker \
  --model-dir outputs/models/act-pair-carry-seed18-20260918/act < /dev/null
```

새 action-ACT 모델은 입력 처리 worker가 다르다. 같은 ACT 환경에서 아래 명령을 사용한다.

```sh
python -m scripts.carry_input_worker \
  --model-dir outputs/models/act-action-first-study-seed24-20260924/act < /dev/null
```

이 검사는 두 RGB 입력과 고정/자기 발행 문맥을 받는 추론 worker의 로딩 검사다. 물리 운반은 기존 표준 `dispatch-skills` workflow에서 명시적으로 선택한다. 예를 들어 설치된 자산을 실행기에 전달하는 계획은 다음처럼 검사할 수 있다.

```sh
python3 -m scripts.sim_cli workflow plan dispatch-skills -- \
  --executor skills --variant open --seed 11 \
  --plan-replay outputs/models/act-pair-carry-seed18-20260918/plan.json \
  --grasp-model-dir outputs/models/act-pair-carry-seed18-20260918/rgb/grasp \
  --stage-model-dir outputs/models/act-pair-carry-seed18-20260918/rgb/varied \
  --reference-top outputs/models/act-pair-carry-seed18-20260918/reference-top.jpg \
  --carry-act-model outputs/models/act-pair-carry-seed18-20260918/act \
  --carry-act-python /absolute/path/to/act-environment/bin/python \
  --contact-profile local_contact_fine --max-wall-s 360
```

`plan`은 실행하지 않는다. 실제 재생은 실행 환경·예산·소스 고정을 확인하고 같은 인자로 `workflow run`을 사용한다. 이 예는 2026-09-18 전체 코호트의 재현 명령이 아니며 새로운 물리 성공 근거도 아니다. 과거 실험 전체를 재현하려면 그 소스 SHA와 프로토콜의 모든 조건·입력을 추가로 갖춰야 한다.

## 이후 모델을 올리는 절차

1. 실험에서 선택/사용한 모델마다 새 ID를 정하고 가중치·config·adapter와 실행에 필요한 자산을 준비한다. 원 학습/실행 SHA, 데이터 출처, 모델 선택 기준, 성공·실패·미검증 범위를 기록한다. 불필요한 중간 체크포인트는 생략할 수 있지만 보고한 비교 모델은 보존한다.
2. 모델 목록에 파일별 상대 경로·크기·SHA-256과 새 Release URL을 등록한다. 원본이 없으면 `unavailable`과 기대 해시를 남긴다. 파일에는 인증정보와 개인 런타임 설정을 포함하지 않는다.
3. 정확히 그 파일들만 있는 staging 폴더를 만들고 패키징한다. `pack`은 파일 목록과 해시를 확인하고 ZIP의 크기·해시를 반환한다. 최초 pack에는 registry의 archive 해시를 미리 알 필요가 없다.

   ```sh
   python3 -m scripts.sim_cli models pack MODEL_ID --source STAGING_DIR --output NEW_ARCHIVE.zip
   ```

4. 반환된 archive 크기·해시를 모델 목록에 기록하고 테스트·커밋·PR로 검토 가능하게 만든다. 버전 태그에 모델 ZIP·모델 목록·체크섬을 업로드한다. 기존 asset과 모델 ID를 덮어쓰지 않고 새 버전으로 추가한다.
5. 업로드 후 GitHub에서 새 빈 폴더로 다운로드하고 ZIP·전체 파일 해시 및 실제 모델 로딩을 확인한다. 새 checkout에서 실행한 명령·코드 SHA·환경·검증 범위를 로컬 배포 기록에 남긴다. 다운로드/로딩과 실제 물리 시험의 성공을 구분한다.

모델 배포는 선택한 가중치와 포함 자산의 원격 보관이다. 학습 원시 데이터·원본 영상 전체의 원격 백업이나 원본 삭제 허가가 아니다. 기존 실험과 번들 기록은 그대로 보존한다.
