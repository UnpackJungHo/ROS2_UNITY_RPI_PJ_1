---
name: git_push_main
description: 변경사항을 개조식 3줄로 압축 요약하고, 커밋 메시지를 제안한 뒤 사용자 확인(OK) 시에만 add/commit/push를 수행한다.
---

## Inputs
- do_push: true/false (default: true)
- type_hint: feat|fix|refactor|chore|docs|test|perf|build|ci (optional)
- scope_hint: (optional) 예: ros2, unity, reinforcement, perception, ui, infra
- remote: (default: origin)
- branch: (default: main)

## Output format
### (A) 3-line bullet summary (필수)
- 한 줄당 “개조식”으로 작성 (서술형 금지, 종결어미 최소화)
- 최대 3줄, 각 줄 최대한 압축
- 형식 예:
  - FinishLineGate 통과 시 evaluator.NotifyFinishCrossed 연동
  - 에피소드 종료 로그에 마지막 충돌 오브젝트/상대속도 기록
  - 테스트 씬에 RL 패널/FinishController 추가 + Robot 레이어 반영

### (B) Commit message proposal (필수)
- Conventional Commits 유지:
  - `<type>(<scope>): <summary>`
- Body는 “개조식 bullet”로만 작성 (서술형 문장 금지)
- Body는 최대 3 bullets (가장 중요한 것만)

### (C) Confirmation prompt (필수)
- 마지막 줄에 사용자에게 아래 중 하나로 답하라고 명시:
  - `OK`  → add/commit/push 진행
  - `REWRITE: ...` → 사용자가 원하는 스타일로 다시 작성 후 다시 컨펌
  - `CANCEL` → 작업 중단

## Rules
- **사용자 컨펌 없이는 절대 add/commit/push를 수행하지 않는다.**
- do_push=false이면:
  - 요약 + 커밋 제안 + 실행 예정 명령만 출력하고 종료 (컨펌도 받지 않음)
- 커밋 메시지 작성 규칙:
  - Summary는 1줄, 50자 내외, “무엇을 했는지”에 집중
  - Body는 bullets만, 최대 3개, 중요도 순
- 변경 파일이 너무 많거나(예: 30+) 서로 성격이 다른 변경이 섞이면:
  - “커밋 분할 제안”을 먼저 하고, 사용자 컨펌을 받는다.
- push 전 체크:
  - `git fetch` 후 origin/main과의 ahead/behind 요약
  - behind/충돌 위험이면 해결 가이드 먼저 제시하고 컨펌 받기

## Steps
1) Git 변경사항 수집 및 요약
   - `git status` 확인
   - `git diff` 및 `git diff --staged` 요약
   - 변경 핵심을 “개조식 3줄”로 압축 작성 (최대 3줄)

2) 커밋 메시지 제안 생성
   - type은 변경 내용 기반 추론 (type_hint 있으면 우선)
   - scope는 scope_hint 있으면 사용, 없으면 대표 모듈 1개로 선정
   - `<type>(<scope>): <summary>` 작성
   - Body는 개조식 bullets 최대 3개로 작성

3) 사용자 컨펌 받기 (필수)
   - 출력:
     - (A) 3줄 요약
     - (B) 커밋 메시지(제목+bullets)
     - (C) 확인 요청 문구
   - 사용자가 `OK`라고 할 때만 다음 단계로 진행
   - 사용자가 `REWRITE: ...`라고 하면:
     - 지시를 반영해 (A)(B) 다시 작성
     - 다시 컨펌 받기
   - 사용자가 `CANCEL`이면 중단

4) add/commit 수행 (OK 받은 후)
   - `git add -A`
   - multi-line 커밋 메시지로 commit 수행 (제목+bullets)

5) push 수행 (OK 받은 후)
   - `git push <remote> <branch>`
   - commit hash 및 push 결과 출력

6) 마무리 리포트
   - 최종 `git status`
   - commit hash
   - push 결과
   - 3줄 요약 재출력
