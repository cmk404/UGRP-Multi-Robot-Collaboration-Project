# 기본 화물 식별판 제거

사용자 요청에 따라 기본 창고 장면의 판자·파이프·나무 상자에서도 QR 형태의 ArUco 식별판과 관련 텍스처 생성을 비활성화했다. 작은 상자의 기존 기본값도 유지된다. 색상 띠·목재 무늬·물체 물리 속성은 보존했다.

과거 비교용 XML은 `add_warehouse_mission_xml(include_fiducials=True)`로 명시적으로 생성할 수 있다. 기본 장면에는 표식이 없으므로 과거 ArUco 전용 관측기는 화물 ID를 검출할 수 없다. 새 인식 알고리즘이나 운반 성능 검증은 이번 변경에 포함하지 않았다.

검증: markerless XML 및 warehouse mission 검사 18개 통과. 필수 offline suite 189개 및 107 subtests 통과. MuJoCo에서 기본 3개 화물을 렌더링하고 식별판이 없는 것을 시각 확인했다. 미리보기는 `outputs/cargo-marker-removal/scene.png`에 로컬 보관하며 원격 백업은 아니다. 렌더러는 종료했다. 이미 실행 중인 다른 세션은 재시작하지 않았으므로 다음 장면 생성부터 적용된다.
