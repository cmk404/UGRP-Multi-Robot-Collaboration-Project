# PR 9개 통합 검토 결과

정식 PR #31 → #32 → #33 → #34 → #40 → #36 → #38 → #27 → #28을 별도 통합 브랜치에 반영했다. **충돌 없음, 오프라인 595 tests + 154 subtests 통과, 기존 기록 55/55 감사 통과, 새 회귀 실행 9/9 예상 결과 일치**다. main 병합은 사용자 승인 후 진행한다.

## 검토와 변경 범위

- [프로토콜](protocol.md)의 9개 HEAD가 모두 통합 커밋의 조상이다. 원래 PR 이력을 보존했다. 기준 main은 `266a4e76c5ae3b028a4dd325481495d61caa6f21`이다.
- 코드·테스트·설정 변경 파일 91개 중 90개는 포함 PR의 blob과 그대로 일치한다. 유일한 자동 조합 파일 `scripts/run_ci_tests.py`에는 pair-carry, navigation, proxy-model 테스트가 모두 남아 있다. [확인 기록](ancestry-and-blobs.json).
- RGB 복구/접근/운반 추론과 실행기의 입력 경계, 사후 평가 분리, 카메라/물리 불변 조건, 동기화 freshness·epoch·timeout·resource release, 지도 주행의 opt-in 입력, 기존 port 기본값, 개인 프록시 격리 및 종료, CI 목록과 병합 후 절차를 검토했다. 발견된 통합 차단 사항은 없다. 전체 코드의 무결함을 보장하는 검토는 아니다.
- 통합 과정에서 정책·모델·카메라·물리를 재조정하지 않았다. 추가 수정은 README/실험 인덱스, 이 검증 프로토콜·실행기·기록이다. 초안 #41·#42는 포함하지 않았다.
- 기존 CSV 3개의 CRLF 원본과 해시를 보존했다. `git -c core.whitespace=blank-at-eol,blank-at-eof,space-before-tab,cr-at-eol diff --check origin/main HEAD`가 통과했다.

## 자동 검사와 새 물리 실행

오프라인 검사는 통합 SHA `fef011e27ada3b71bd0dadd47d4ef51df4df3aa7`의 69개 모듈에서 **595 passed, 154 subtests passed, 15.74초**였다. [원본 로그](offline-tests.txt). 이후 프로토콜과 문서를 커밋한 **`4dc01c12c24afbbb183d4618d733ecce7a96f270`**에서 아래 물리 실행·감사를 수행했다. 그 사이 production/test/config 소스는 바뀌지 않았다. 실행 중 source와 working tree가 고정됐음을 실행기가 확인했다.

커밋된 모델 ZIP을 SHA-256 검증 후 새 폴더에 복원했다. 기존 pair-carry 증거 55개·6,638파일도 각각 해시 검증하여 복원했다. 통합 코드의 저장 RGB/명령/보고/평가 재계산 감사는 **55/55 통과**, 물리 성공 판정은 원래와 같은 **49/55**였다. 이 55개는 개발 진단과 최종 비교를 포함하므로 최종 52개 코호트의 성공률과 합치지 않는다. 새 물리 시뮬레이션도 아니다.

| 새 실행 | 실제 결과 | 감사 | 사전 예상과 일치 |
|---|---|---|---|
| 직진 접근 heldout-01 | 접근·파지 성공 | 통과 | 예 |
| 다양한 시작 heldout-01 | 접근·파지 성공 | 통과 | 예 |
| 정상 pair sync | 운반·내려놓기 성공 | 통과 | 예 |
| r1 출발 1초 지연 baseline | RGB 범위 이탈로 중단, 운반 완료 실패 | 통과 | 예 |
| r1 출발 1초 지연 sync | 정렬 후 운반·내려놓기 성공 | 통과 | 예 |
| r3 중간 0.75초 지연 sync | 운반·내려놓기 성공 | 통과 | 예 |
| r3 보고 1초 누락 sync | 운반·내려놓기 성공 | 통과 | 예 |
| r1 slalom heading | 충돌 없이 도착, 142판단·56.35 SIM초 | 통과 | 예 |
| r1 narrow heading | 1판단 후 `no_map_route`, 진입 거부 | 통과 | 예 |

**9/9는 회귀 기대 결과 일치율**이다. 비교군의 의도된 실패와 안전 거부를 물리 성공률에 섞지 않는다. 모든 실험은 LLM 호출·API 비용 0이며, 학생과 지도 주행의 관측 경계를 각 실행별 감사로 확인했다. 운반/파지에는 weld가 없고, 지도 주행의 장애물 접촉·weld step은 0이다. 결과의 평가 좌표는 제어 종료 후 별도 기록에만 사용된다.

종료 코드만으로 성공을 판정하지 않았다. Mac의 mjpython 실행에서 baseline의 `result.json`에 `policy stopped: invalid RGB`와 `success:false`가 기록됐지만 프로세스 종료 코드는 0이었다. 실행기는 결과 JSON과 독립 감사까지 읽어 예상 실패를 확인했다.

## 영상 확인

실행 영상에서 접근의 시작·중간·들기 완료, 지연 비교군의 중단, sync의 운반·내려놓기, slalom의 우회와 도착, narrow의 정지를 확인했다. 정확한 프레임 번호와 fps는 각 `*-video-review.json` 및 `sync-motion-review.json`에 있다. 영상 전체를 매 프레임 검토했다는 뜻은 아니며 연속성 판정은 저장 물리/명령 기록의 검사와 함께 해석한다.

- [접근 6프레임](approach-review.jpg)
- [운반 지연·보고 누락 12프레임](carry-review.jpg)
- [출발 지연 sync의 운반·내려놓기 구간 6프레임](sync-motion-review.jpg)
- [지도 주행·진입 거부 8프레임](navigation-review.jpg)

## 기록과 한계

[summary.json](summary.json), [환경](environment.json), [실제 명령/시간/종료 코드](commands.json), [소스 해시](source-manifest.json)에 실행을 연결했다. 원본 결과·감사 19개를 gzip으로 저장하고 [records.json](records.json)에 원본 경로와 해시를 남겼다. gzip 해제한 바이트의 SHA-256을 검사할 수 있다.

새 RGB·영상·로그 원본 1,769파일(92,552,676 bytes)의 [파일별 해시](raw-files.jsonl.gz)와 실제 보관 위치는 `/Users/changmin/projects/ugrp-worktrees/integrate-reviewed-stack/outputs/integration-review/runtime`이다. 원본 영상/RGB 전체는 로컬에만 있고 새 원격 백업을 만들지 않았다. 이 PR에는 압축 결과/감사와 검토용 정지 화면을 포함한다. 기존 55개 증거의 RGB ZIP은 원래 pair-carry 실험 폴더에 이미 포함돼 있다.

이번 검사는 기존 조건의 조합 회귀 검증이다. 새 물체/장면 일반화, 다양한 시작의 운반 6/10 한계 해결, 실물 로봇, 실제 LLM 협력 또는 실제 네트워크 지연 성공을 검증하지 않았다. 지도 주행과 운반은 별도 실행 경로다. 학생 모델은 현재 고정 장면에 제한된다.

검증 세션 `pr-integration-20260914`는 종료됐고 status에서 stopped를 확인했다. 원래 main 및 기존 feature worktree는 변경하지 않았다. 최종 원격 SHA의 GitHub fresh-checkout CI 결과는 통합 PR의 Checks와 본문에서 확인한다. 로컬 테스트는 Ubuntu 설치/모델 프록시 로그인/실제 모델 응답을 대신하지 않는다.
