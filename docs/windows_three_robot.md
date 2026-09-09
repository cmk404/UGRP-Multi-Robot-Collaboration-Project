# Windows 3대 협력 실행

이 실행기는 `record_mixed_warehouse`의 같은 MuJoCo 공간에서 R1/R2/R3가 단독·공동 화물을 처리하는 배치 실험을 실행한다. 결과는 대화가 포함된 영상으로 확인한다. 실시간 TEAM 채팅 웹 서버는 이 실행기에 포함되지 않는다.

## 실행

PowerShell에서 저장소 루트로 이동한 후 프록시를 시작한다. API 키는 숨김 입력창에 입력한다.

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_gemini_proxy.ps1
```

프록시 창을 열어 두고, 다른 PowerShell 창에서 실행한다.

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_three_robot.ps1
```

모델을 바꾸려면 `-Model <사용 가능한 모델 ID>`를 추가한다. 기본 프록시 주소는 `http://127.0.0.1:8391/v1/chat/completions`이며 다른 서버는 `GEMINI_PROXY_URL` 환경변수로 지정한다.

API 없이 규칙 기반 협력 실험은 `-Condition rule`, 환경 검사만 하려면 `-CheckOnly`를 추가한다. Gemini 모드의 환경 검사는 짧은 실제 API 호출 한 번을 포함한다. `/health`의 configured 값만으로 키·모델·할당량의 유효성을 판단하지 않는다.

## 검사와 결과

- Python 가상환경, FFmpeg, Windows 한글 글꼴, 3대의 카메라 렌더링을 검사한다.
- FFmpeg가 PATH에 없으면 사용자 WinGet의 Gyan.FFmpeg 설치 경로를 찾는다.
- Gemini 모드는 지정한 모델에 실제 요청 후 실험을 시작한다.
- 기본 실험 제한은 240초, 로봇별 정책 호출 12회다. 준비·렌더링·종료 처리 시간은 별도로 필요하다.
- 매번 고유한 `outputs/three-robot-<시간>/` 폴더를 만든다.
- `environment.json`: 코드 SHA, 작업 폴더 변경 여부, Python, 실행 옵션.
- `console.log`: 실행 중 출력. `episode/result.json`: 임무 성공 여부와 사유.
- `episode/mixed-dialogue-1x.mp4`: 대화가 포함된 영상. 영상의 첫 프레임 디코딩도 확인한다.

임무 미완료와 프로그램 오류는 성공으로 표시하지 않는다. 결과 파일의 `success`, `reason`, `errors`와 영상을 함께 검토한다. 임무 성공은 정책과 환경 조건에 따라 달라진다. Ctrl+C로 중지하면 해당 실행기의 자식 프로세스를 정리한다. 기존에 별도로 실행한 프록시는 프록시 창에서 Ctrl+C로 종료한다.

raw 영상과 로그는 로컬에만 저장되며 GitHub에는 포함되지 않는다. Gemini 공식 API는 키의 사용량·할당량·요금 설정을 따른다.
