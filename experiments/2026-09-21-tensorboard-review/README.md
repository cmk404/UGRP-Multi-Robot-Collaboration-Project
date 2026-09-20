# TensorBoard 기록 열람 검증 — 2026-09-21

로봇 실험이 아닌 읽기 전용 변환/뷰어 검증이다. 변환기 `9c45f24`를 커밋한 깨끗한 체크아웃에서 실행했다. 화면 검증 중 발견한 launcher의 scalar 표본 수 설정은 후속 커밋에서 0 → 10000으로 수정했다. TensorBoard data provider에서 0은 빈 결과를 반환해 그래프가 비었으며, 수정 뒤 같은 이벤트의 loss 곡선이 표시됨을 확인했다.

## 원본과 결과

- ACT 학습 2개, ACT/교사 공동 운반 12개, Jev/규칙/Gemini 접근 9개, 다중 물건 1개: 총 24개 기록, 변환 실패/경고 0.
- 위치: `/Users/changmin/projects/ugrp/outputs/tensorboard/20260921-review-v1`.
- TensorBoard 2.21.0, Pillow 12.3.0, 기존 Mac Python 환경. `pip check` 통과. TensorFlow/PyTorch 추가 설치 없음.
- 숫자 33,635개, 텍스트 7,274개, 이미지 364개. EventAccumulator로 전체 재열람. HParams 메타데이터를 포함한 tensor는 7,346개.
- 읽은 원본 JSON·이미지 397건 SHA256을 변환 뒤 재검증해 일치. 전체 manifest 및 event 해시는 `validation.json`에 기록했다. 원본 동영상은 복사/재인코딩하지 않고 기존 파일로 연결했다.
- 새 시뮬레이션·학습·모델 요청 없음. 서로 다른 실행 조건의 성공률을 새로 합산하거나 비교 우위를 주장하지 않는다.

## 검증

- 오프라인 회귀: **1003 passed, 1 skipped, 184 subtests passed**.
- 선택 의존성을 사용하는 변환/영상 검사: **15 passed**. 원본 불변·시간축·미기록 값·주장/평가 분리·물리 로봇 ID·이미지 해시·HTTP Range/경로 제한 포함.
- 실제 TensorBoard 화면: ACT loss 곡선, 24개 HParams 결과 행, 입력 카메라 이미지, JSON 텍스트, 원본 MP4 링크 확인.
- 다중 물건 영상의 브라우저 로딩: 960×720, 5.75초, readyState=4, 첫 프레임 시각 확인. 모든 원본 영상을 처음부터 끝까지 재검토한 것은 아니다.
- 사용한 `tensorboard-qa` 세션은 검증 종료 후 정리한다. 별도 원본 실험 세션은 변경하지 않았다.

## 남은 범위

실시간 JSON 감시는 없으며 새 스냅샷은 명시적으로 변환한다. 파일 mtime을 과거 실행 시작 시각으로 취급하지 않는다. 이벤트 WALL 시각은 변환 시각이며 학습 STEP·SIM 시각과 구분한다. 원본 영상 시각과 그래프를 자동 동기화하지 않는다. raw 자료와 이벤트는 로컬 보관이며 이 해시 기록만으로 원격 백업된 것이 아니다. 실행법은 [TensorBoard 안내](../../docs/tensorboard.md)를 따른다.
