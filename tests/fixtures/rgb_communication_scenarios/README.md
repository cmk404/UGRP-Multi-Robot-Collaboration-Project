# A2 fixture와 판정 범위

- catalog.json: 세 family 실행 규격. 두 pilot candidate는 선정 이유만 고정했고 실제 실행
  components/pins는 null이다. 모든 live gate는 현재 NO-GO.
- baseline_audit.json: 통합936bf821의 실제 소스 SHA-256와 오프라인 재현 기록.
  characterization green은 취약점/미지원 재현이며 최종 경로 compliance가 아니다.
- tests의 linked_fixture와 임시 review 파일: 순수 합성 metadata/JPEG 경계 마커. 실제
  카메라/물리 성공·독립 검토를 표현하지 않는다. 테스트 임시 디렉터리 밖에 발급하지 않는다.

기존 B pair-request/static context fixture를 다시 사용하며 endpoint는 fake다.
외부 모델/물리/renderer/remote job을 실행하지 않는다. 데이터/출처·환경 gate의 실제
evidence는 담당 D가 회수하고 A가 확정 소스와 영상을 검토한 뒤 별도 기록해야 한다.
fixture 성적은 공통 TensorBoard나 실제 연구 결과에 등록하지 않는다.
