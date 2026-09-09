# Windows 3대 실행기 smoke 검증

- 코드: `f432b7b` (실행기 기능 커밋). 실행 시 environment.json에서 전체 SHA 확인 가능.
- 명령: `.venv-sim/Scripts/python.exe -m scripts.run_three_robot --condition rule --timeout 30 --max-calls 4`
- 환경: Windows, Python 3.12 가상환경, MuJoCo glfw, WinGet FFmpeg, 맑은 고딕.
- 사전 검사: 3대 카메라 렌더링·FFmpeg·글꼴 통과.
- 결과: `success=false`, `reason=INCOMPLETE`, 실행기 종료 코드 1.
- 정책 호출: r1=1, r2=1, r3=2. 에피소드 경과 시간 약 30.141초.
- 대화 영상: 79프레임, 실제 메시지 4개. 영상 첫 프레임 디코딩 통과.
- 검토 범위: 파일 생성, 임무 실패 표시, 영상 디코딩만 검증. 운반 성공·영상 전 프레임 시각 검토·Gemini 판단은 검증하지 않음.
- API 호출: 0. 비용: 외부 모델 비용 없음.
- 원본 위치: 로컬 `outputs/three-robot-20260909-184319-317858/`. 원격 백업 없음.

30초 제한 시험이므로 전체 연구 실험 성공률이나 다중 로봇 운반 성공의 근거로 사용하지 않는다. Gemini 모드에는 사용자의 유효한 프록시·키·모델 접근 권한이 필요하다.
