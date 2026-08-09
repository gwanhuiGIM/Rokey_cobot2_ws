<!-- meta
updated: 2026-08-10
status:  blocked — 팀원 확인 대기
owns:    voice_processing ↔ M0609_VLA_system 통합의 "원본이 무엇인가" 재확인
-->

# 음성/VLA 통합 보류 — 원본 재확인 필요 (2026-08-10)

## 왜 보류했나

[`2026-08-08-vla-integration.md`](2026-08-08-vla-integration.md)는 `~/M0609_VLA_system`을
`vla_command_node`의 상대 원본으로 전제하고 쓰였다. 2026-08-09 대화 중 그 repo를 직접 열어
대조해보니, 실제로는 **완전히 다른 두 계보가 섞여 있었다**:

| 우리 쪽 코드 | 실제 원본 |
|---|---|
| `voice_processing/get_keyword.py` (마이크·wakeword·STT·LLM) | `~/M0609_VLA_system`이 **아니다** — 이 ws에 있던 옛 `pick_and_place_voice` 패키지(원조는 `corecode/VoiceProcessing` 튜토리얼) |
| `voice_processing/vla_command_node.py` (`/vla/pick_command` JSON 어댑터) | `~/M0609_VLA_system`의 코드를 재사용한 게 아니라, 그 시스템의 **최종 판단 출력**("무엇을 집어라")만 받도록 새로 짠 얇은 어댑터. `agent_node`/`robot_node`/`vla_interfaces` 등은 한 줄도 안 가져왔다 |

즉 사용자가 통합 대상으로 제시한 `~/M0609_VLA_system`이 실제로 이 노드들의 "원본"이 맞는지
자체가 이번 대화에서 처음으로 의심됐다 — **팀원과 소통 없이 이 전제로 계속 진행하면 안 된다.**

## 팀원에게 확인할 것

- [ ] `voice_processing`의 진짜 원본이 `~/M0609_VLA_system`이 맞는지, 아니면 제3의
      repo/브랜치인지
- [ ] `get_keyword.py` 계열(`pick_and_place_voice` 원조)이 팀원 쪽에서 더 진행된 버전이
      있는지 — 있다면 이 ws가 뒤처진 스냅샷을 들고 있는 것
- [ ] `~/M0609_VLA_system`이 맞다면, 그 시스템의 `agent_node`가 갖고 있는 기능
      (대화형 재질의, 도중 취소, `pixel` 기반 개체 선택)을 이 ws의 안전장치
      (`WAIT_APPROVAL`, `allowed_classes` 검증, `request_id` 상관관계)와 어떻게
      합칠지 — 어댑터 계층만 유지할지, 일부 로직을 이식할지

## 지금 상태 (건드리지 않음)

- `voice_processing` 코드·빌드는 이번 세션에서 고친 dangling symlink 건 외엔 무변경.
- `md/plans/2026-08-08-vla-integration.md`의 §0~§7 설계는 **전제("원본 = M0609_VLA_system")가
  재확인 전까지는 잠정**으로 취급한다 — 폐기하지 않았고, 팀원 확인 후 맞으면 그대로 이어간다.
- 실기 통합 작업(추가 코드 작성·병합)은 팀원 답 나올 때까지 **보류**.
