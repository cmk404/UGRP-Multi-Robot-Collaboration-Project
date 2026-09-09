# Private GitHub 동기화

이 저장소는 `kcm0127-dotcom/ugrp` private repository와 수동으로 동기화한다. 시작할 때 working tree가 깨끗한 경우에만 다음 명령으로 fast-forward pull한다.

```sh
/Users/changmin/projects/main/scripts/project-sync start /Users/changmin/projects/ugrp
```

종료할 때는 검토한 코드·설정·연구 문서 경로와 commit 메시지를 직접 지정한다. trace, 영상, 환경 파일, 키, 로컬 에이전트 상태를 추가하지 않는다.

```sh
/Users/changmin/projects/main/scripts/project-sync finish /Users/changmin/projects/ugrp "설명" harness/file.py docs/note.md
```

다른 작업 공간의 변경은 pull request로 검토·merge한 후 로컬에서 받는다. background 자동 동기화는 설정하지 않는다.
