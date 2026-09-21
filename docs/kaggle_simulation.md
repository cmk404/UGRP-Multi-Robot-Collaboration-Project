# Kaggle CLI에서 UGRP 실행

Colab과 별도로 Kaggle의 비공개 Dataset·Python script kernel을 이용해 시뮬레이션을 배치 실행한다. 기본은 CPU·OSMesa이며 모델 계정/GitHub 토큰/Drive 연결은 필요 없다. 최초 Kaggle 계정 인증만 사용자가 수행하고 이후 소스 전송·제출·상태·결과 회수는 CLI로 진행한다.

## 1. 최초 인증

이 Mac에는 독립 uv tool로 `kaggle 2.2.4`가 설치되어 있다.

```sh
kaggle auth login
kaggle quota --format json
```

로그인 화면에서 인증을 완료한다. CLI가 코드를 요구하는 경우 터미널에 직접 입력한다. 코드·토큰·인증 파일은 채팅·Git·실험 ZIP에 넣지 않는다. 서버가 추가 계정 확인이나 권한 동의를 요구하면 사용자가 그 단계를 완료한다. 인증 확인과 원격 실행 성공은 별도 상태다.

## 2. 커밋한 소스 준비

저장소 루트에서:

```sh
python3 scripts/kaggle_simulation_cli.py prepare --output outputs/kaggle-demo-01
```

인증된 CLI의 `kernels init`으로 계정 이름을 읽는다. 필요하면 `--owner <Kaggle 사용자명>`을 명시할 수 있다. 준비 단계는 로컬에서 Linux/Python 3.12 wheel과 Ubuntu 22.04 렌더링 라이브러리를 다운로드하고 실행 자료를 생성한다. Kaggle로 업로드하지는 않는다. 로컬 Python에는 pip가 필요하다. 추적 소스가 수정되어 있으면 먼저 커밋해야 한다.

Colab과 같은 shallow sparse Git snapshot을 사용하여 실제 커밋 SHA를 보존한다. 실행 코드·설정·테스트·작은 실험 JSON만 기본 포함하고, 큰 모델/과거 증거는 제외한다. `source-manifest.json`에서 포함/제외 목록을 확인한다. 추가 추적 파일은 `--include <경로>`로 넣는다. 로컬 raw 모델·데이터를 임의로 전부 업로드하지 않는다.

생성물:

- `dataset/`: source `.bin` 압축본, source/dependency manifest, Linux wheel·deb 패키지, Dataset 메타데이터를 포함한다. 확장자는 Kaggle의 자동 압축 해제를 피하기 위해 `.bin`이며 내용은 SHA-256으로 확인하는 gzip tar다.
- `kernel/`: 원격 `run.py`와 비공개·CPU 메타데이터. 원격 인터넷은 OFF이며 검증된 입력 패키지만 설치한다.
- `job.json`: 고유 작업 ID·소스 SHA·Dataset/kernel 식별자·제출 단계. 기존 작업의 slug를 재사용하거나 덮어쓰지 않는다.

Dataset의 라이선스 필드는 `other`이며 설명에 기존 저작권·라이선스를 유지하고 추가 재배포 허락을 부여하지 않는 비공개 실행 사본임을 명시한다. 문서에 나열된 `copyright-authors`는 실제 서버에서 거부되어 사용하지 않는다. 공개 배포/CC0로 바꾸지 않는다.

## 3. 제출·상태 확인

```sh
python3 scripts/kaggle_simulation_cli.py submit --output outputs/kaggle-demo-01
python3 scripts/kaggle_simulation_cli.py status --output outputs/kaggle-demo-01
```

Dataset은 공식 CLI의 기본 private 생성으로 업로드하고 서버 메타데이터에서도 private인지 확인한 뒤 kernel을 제출한다. 소스 해시·driver·작업 ID·CPU/private 설정을 다시 검사하며 업로드 폴더에 예상하지 않은 파일이 있으면 거부한다.

Dataset 인덱싱 중이면 submit이 종료 코드 2와 안내를 출력한다. 잠시 후 같은 submit 명령으로 이어간다. 이미 생성 요청을 보낸 Dataset을 중복 생성하지 않는다. kernel 제출 요청 뒤에는 자동 재제출하지 않는다. Kaggle CLI는 오류 메시지를 출력하면서 종료 코드 0을 반환할 수 있어 성공 응답의 버전 번호도 확인한다. 오류/불확실 상태는 로컬 제출 로그와 Kaggle 상태를 먼저 조사한다.

기본 데모는 Kaggle의 Python 3.12/x86_64 런타임에서 독립 시뮬레이션 환경을 만들고 `scripts.sim_quickstart`를 실행한다. CPU 작업 최대 요청 시간은 1,800초다. 계정별 실제 할당·제한이 우선한다. GPU나 유료 자원을 자동 구매·전환하지 않는다.

다른 실행기는 prepare 단계에서 지정한다.

```sh
python3 scripts/kaggle_simulation_cli.py prepare \
  --output outputs/kaggle-traffic-01 --module scripts.run_rgb_traffic \
  -- --scenario crossing --out-dir '{output}'
```

`{output}`은 원격의 새로운 결과 디렉터리다. 임의 모델/데이터 의존성까지 자동 준비하지는 않는다. 실제 LLM은 원격에서 접근 가능한 인증된 endpoint가 있어야 하고 ACT는 별도 학습 환경을 사용해야 한다.

## 4. 결과 회수

```sh
python3 scripts/kaggle_simulation_cli.py collect --output outputs/kaggle-demo-01
```

종료 상태에서만 다운로드하며, 실행 중이면 그대로 두고 나중에 collect한다. 실패 작업도 다운로드를 시도해 `failure.txt`, `setup.log`, `remote-job.json`을 보존한다. setup이 실패해 결과 ZIP이 없으면 완료로 표시하지 않는다.

회수 시 서버의 kernel private 설정, 고유 작업 ID, 실행 SHA, ZIP 전체 해시, 모든 내부 결과 파일 해시와 종료 코드를 확인한다. 성공하면 로컬 `verified-result.json`과 `job.json`의 downloaded 경로를 남긴다. 프로세스 종료 코드 0은 로봇 운반 성공을 뜻하지 않는다. 물리/카메라 데모는 summary와 GIF를 따로 확인한다.

Linux wheel을 재사용하려면 `prepare --wheelhouse <폴더>`를 지정한다. Python 3.12/x86_64용 전체 의존성과 `pip==26.2.1` wheel이 필요하다. Ubuntu deb는 공식 archive URL과 고정 SHA-256으로 검증하고 `dpkg-deb -x`로 작업 폴더에만 푼다. 시스템 패키지·기존 Python을 변경하지 않는다. 실제 검증 대상은 Ubuntu 22.04/CPython 3.12이며 Kaggle 기본 이미지가 바뀌면 다시 확인한다.

원격 환경·checkout은 `/kaggle/temp`, 회수할 자료만 `/kaggle/working`에 둔다. setup과 시뮬레이션은 유한 실행이며 상시 서버·자동 재접속을 만들지 않는다. 로컬 CLI를 닫아도 이미 제출한 Kaggle 배치 작업은 별도 상태다. 이 실행기는 다른 작업을 종료하거나 Dataset/kernel을 삭제하지 않는다.

## 현재 검증 범위

로컬 24개 관련 테스트를 통과했다. 계정 `changmin2026` 인증과 private Dataset/kernel 제출을 확인했다. 첫 원격 실행은 패키지 서버 DNS 실패로 설치 단계에서 종료했다. 읽기 전용 런타임 진단으로 Python 3.12, Ubuntu 22.04 및 OSMesa 부재를 확인하여 오프라인 의존성 방식으로 수정했으며, 실제 물리·렌더링 재검증을 진행 중이다. 실패 로그와 소스는 로컬 outputs에 보존한다.

참고: [공식 kernels 명령](https://github.com/Kaggle/kaggle-cli/blob/main/docs/kernels.md), [Dataset 메타데이터와 라이선스](https://github.com/Kaggle/kaggle-cli/blob/main/docs/datasets_metadata.md), [kernel 메타데이터](https://github.com/Kaggle/kaggle-cli/blob/main/docs/kernels_metadata.md). 실제 플래그는 설치된 `kaggle <명령> --help`를 우선한다.
