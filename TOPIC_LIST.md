# Project Rules

## Git / Release
- main push 전: 커밋 제안 → 사용자 OK 후에만 add/commit/push
- Conventional Commits 사용 (type(scope): summary)
- 커밋 본문은 개조식 bullet, 최대 3개

## Testing Gate (when available)
- Unity Editor.log / Player.log 에 Error/Exception 있으면 FAIL
- FAIL이면 push 금지, 원인/해결/재현 절차 기록

## Documentation
- 문제 해결 기록은 Notion에: 문제/원인/해결/적용파일/검증결과/커밋SHA 템플릿 유지

## Python
- 파이썬 코드 실행 시 conda activate driving 을 실행 후 테스트 (가상환경에서 실행)

