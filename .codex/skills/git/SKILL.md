---
name: git_push_master
description: 변경사항을 요약하고 커밋 메시지를 생성한 뒤 add/commit 하고 main로 푸시한다.
---
## Inputs I expect
- 커밋/푸시 수행 여부(true/false)  (기본 true)
- (선택) 커밋 타입 힌트: feat | fix | refactor | chore | docs | test | perf | build | ci
- (선택) 푸시 대상 브랜치: 기본 main
- (선택) 커밋 범위(scope): 예) ros2, unity, perception, ui, infra

## Output format
- 요약 3줄(무엇/왜/어떻게)
- 생성된 커밋 메시지(Conventional Commit)
- 실행 결과(status, commit hash, push 결과)

## Rules
- 커밋/푸시 수행 여부가 false면, **커밋 메시지 생성과 실행 계획만** 출력하고 실제 add/commit/push는 하지 않는다.
- 커밋 메시지는 **Conventional Commits** 형식을 따른다:
  - `<type>(<scope>): <summary>`
  - 필요 시 본문(body)에 변경 이유/주의사항을 2~4줄로 추가
- 변경 파일이 너무 많으면(예: 30개 이상) 무조건 한 번에 밀지 말고,
  - 논리적으로 묶이는 단위로 “커밋 분할 제안”을 먼저 출력한다.
- main으로 push하기 전에 현재 브랜치가 main이 아니면:
  - (권장) main로 체크아웃/리베이스 또는 머지 방식을 제안하고,
  - 사용자가 “그대로 진행”을 명시하면 현재 브랜치에서 push한다.
- push 전에 원격 상태를 확인한다:
  - `git fetch` 후 upstream과의 차이를 요약하고,
  - 충돌/behind 상황이면 해결 방안을 먼저 제시한다.

## Steps
1) Git 변경사항을 확인한다
   - `git status`로 변경/스테이징 상태를 확인한다.
   - `git diff`(unstaged)와 `git diff --staged`를 모두 요약한다(있는 경우).
   - 변경사항을 "무엇을/왜/어떻게" 3줄로 정리한다.

2) 커밋 메시지를 생성한다
   - 변경 파일 경로/내용을 근거로 type을 추론한다(입력 힌트가 있으면 우선).
   - scope는 입력이 있으면 사용하고, 없으면 폴더/모듈 기준으로 1개만 선정한다.
   - summary(한 줄)는 50자 내외로 명확히 작성한다.
   - body에는 아래 중 해당되는 항목만 2~4줄로 작성한다:
     - 왜 필요한 변경인지(문제/목표)
     - 동작 변화(behavior change)
     - 테스트 방법(있으면)
     - 주의사항/호환성(있으면)

3) add/commit 수행(조건부)
   - 입력 `커밋/푸시 수행 여부`가 false면:
     - (1)~(2) 결과와, 실행 예정 명령어만 출력하고 종료한다.
   - true면:
     - `git add -A`로 변경사항을 스테이징한다.
     - `git commit -m "<type>(<scope>): <summary>"` 를 수행한다.
     - body가 있으면 multi-line commit 메시지로 커밋한다.

4) main로 push한다
   - `git fetch` 후 현재 브랜치와 origin/main의 차이를 요약한다.
   - 문제가 없으면 `git push origin main`를 수행한다.
   - 결과로 커밋 해시와 push 결과를 출력한다.

5) 마무리 리포트 출력
   - 최종 상태(`git status`), 커밋 해시, 푸시 대상 브랜치, 요약 3줄을 한 번에 정리한다.
