# 로컬 시뮬레이션 구성 검토 — 2026-09-22

기존 코드·지도·현재 안내·연구 TODO를 새 CLI/API와 대조했다. 점검 대상은 장면 선택,
물리 초기화/reset, 카메라/제어 입력, action·제어기 확장, 기록·종료, 설치·기존 연구 진입점이다.
전체 연구의 성능/완료 여부를 재평가한 문서가 아니다.

## 발견한 문제와 수정

| 문제 | 수정 |
|---|---|
| 기존 4개 엔진 예제만 선택 가능; 연구 맵 누락 | 기존 생성기를 연결한 `scenes` 목록 58항목. 미등록 이름은 오류로 종료 |
| 기본 화면/새 실험이 camera_team과 장애물/공 예제 | 무인자 실행·init·new는 기존 공동 출하장. 장난감 예제는 extensions-demo로 명시 |
| research reset을 생략하면 로봇/화물이 옛 창고 배치로 이동 | 연구 장면별 초기 배치를 매 reset에 적용; model/data 유지 |
| 연구 생성기가 새 추가 물체를 숨길 수 있음 | 연구 XML 변환 후 사용자 형상을 추가 |
| 자유 시점의 중심과 기록된 화물 목록이 옛 창고 기준 | 선택 지도의 경계와 실제 연구 화물 목록 사용 |
| 지도 출처·초기화·실제 컴파일 장면을 기록하지 않음 | scene.json/XML, model.mjb, 지도·프로토콜 원본/해시 저장 |
| 확장 초기화 오류 시 진입 소스가 남지 않음 | trusted Python 실행 전에 input-files를 저장, 실패 result 유지 |
| headless 시간 초과가 성공 종료 코드 | 미완료 wall limit은 종료 코드 2 |
| start/end 캡처만 제공 | ffmpeg MP4와 프레임별 에피소드/SIM 시각, 관찰용 카메라 선택 |
| 설치 안내와 로컬/클라우드 기본값이 서로 모순 | 로컬 시작 경로 통일, doctor로 현재 환경 확인 |
| 기존 연구 스킬·학습·평가 경로를 새 사용자가 찾기 어려움 | workflows에서 17개 주요 진입점·필요 자료·범위 제공 |

## 보존·연결한 자산

| 자산 | 새 로컬 진입점 | 원래 동작과 제한 |
|---|---|---|
| standard/mixed/arena/camera_team 4종 | 같은 scene.layout 이름 | 기존 설정 그대로 호환. 구버전 v1에서 layout 생략 시 camera_team |
| dispatch 5변형 | dispatch/open, shared_crossing, north_blocked, narrow_south, rough_south | 기존 빔/상자/도크·seed 배치·비공개 장애물 복원. 정책·물리 프로필은 명시적으로 선택 |
| 단독 navigation 9종 | navigation/<파일 이름> | 기존 벽·지형·고정 TOP. r1은 시작 구역, r2/r3는 코스 밖 |
| 공동 운반 지도 6종 | pair_navigation/<파일 이름> | 원래 pair 지도·목표 방향·footprint 보존. 장면 확인은 바닥 빔 상태; 실제 파지/운반은 기존 runner |
| ACT 지도 suite 22조건 | act/<case ID> | 새 16 + 기존 회귀 6. 분할·map_seed·원본 프로토콜 보존. 자동 학습/ACT 실행 아님 |
| 다중 물건 12조건 | multi_object/<case ID> | 기존 1~8개 빔/상자 복제·임무·목적 영역·초기화 보존 |
| 사용자 지도 | navigation/file 또는 pair_navigation/file + map_file | 기존 JSON 계약 검사. config 폴더 기준 경로. 고정 TOP 변경 거부 |
| 장애물/물체·정책·action 추가 | scene.builder / controllers / action_plugins | 기존 장면 위에 추가. 새 물체가 자동으로 운반 임무에 편입되지는 않음 |

지도 JSON 자체는 교체하거나 삭제하지 않았다. 정적 미리보기에 기존 코드가 정의한 배치를 사용한다.
58은 선택 가능한 항목 수이며 서로 독립된 58개 맵 또는 성공한 58개 운반 조건이라는 뜻이 아니다.
기존 nav/pair/ACT 회귀에는 같은 형상의 중복이 있다.

## 별도 연구 실행기의 역할

`bash scripts/open_simulation.command workflows`에서 소스·문서·필요 자료를 확인한다.
3대 계획·출하 스킬, RGB 공동 운반, 지도 주행, ACT 교사/학습/평가, Jev 비교, 다중 물건 실행,
통신 조건 구성/감사, 단계 동기화, 물리 장치 기록과 기존 워커를 보존한다.
새 로컬 실행기는 이들을 임의로 시작하지 않는다. 일부 경로에는 Git에 없는 모델·원시 기록과
개인 모델 프록시가 필요하다. 새 clone의 기본 native 장면은 그 자료 없이 열린다.

기존 연구 XML 변환은 공유 모듈 `sim/research_scene_xml.py`에서 재사용한다.
engine → 장면 XML/초기화 adapter → Simulation → CLI 순으로 구성하고,
제어기는 자기 RGB·TOP·자기 명령만 받는다. 지도/초기화/평가 원문은 실행기 기록에만 남는다.
과거 실행기의 global monkeypatch는 호환 경로로 유지하며, 새 Simulation은 인스턴스별 XML hook을 쓴다.

## 남은 연구·확장 범위

- 3대 MasterPi가 엔진 계약이다. 새 로봇 종류/대수·관절·센서는 별도 엔진 개발이 필요하다.
- 물체 추가와 LLM의 물체 식별·임무·스킬 연결은 별도 단계다. 미지원 조건을 성공 동작으로 대체하지 않는다.
- 기본 제어기 템플릿은 정지한다. 기존 ACT/운반 모델은 필요한 가중치·입력 계약을 갖춘 기존 실행기로 사용한다.
- 다중 물건/맵 일반화·통신 A/B/C의 최종 연구 성능은 [연구 TODO](research_todo.md)의 미완료 항목이다.
- MP4는 관찰 기록이다. 시뮬레이터·난수·제어기 기억을 모두 복원하는 checkpoint replay는 미구현이다.
- 사용자 Python은 trusted code이며 보안 격리가 아니다. 동기 호출이 멈추면 그 호출을 wall limit으로 강제 중단하지 못한다.
- viewer GUI의 실제 마우스/키 조작은 물리/API·렌더링·정상 종료 검증과 별도로 보고한다.

Mac·Ubuntu에서 58/58 장면 생성·reset을 통과했다. 실행·검증 범위와 소스 SHA는
[검증 기록](../experiments/2026-09-22-simulation-scenes/README.md)에 남겼다.
