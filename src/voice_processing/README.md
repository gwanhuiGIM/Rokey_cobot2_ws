# voice_processing

*`pick_fsm` 의 **지시 입력 층**. 사람 음성이든 외부 VLA 든, 결국 `/get_keyword`
(`std_srvs/Trigger`) 하나로 FSM 에 들어간다.*

> 🚨 여기서 지시를 보내면 **실기가 실제로 움직인다.** 이 노드는 `/pick/approve` 를
> **부르지 않는다** — 승인은 언제나 사람이 누른다. 최종 안전장치는 물리 비상정지 버튼이다.

| 노드 | 입력 | 추가 의존성 |
|---|---|---|
| `vla_command_node` | `/vla/pick_command` (외부 PC 의 VLA, JSON) | **없음** — 표준 ROS 2 만 |
| `get_keyword` | 마이크 → wakeword → Whisper STT → LLM | `openai` `langchain-openai` `python-dotenv` `pyaudio` `openwakeword` `sounddevice` + `resource/.env` |

⚠️ **둘 다 `/get_keyword` 를 제공한다. 동시에 띄우지 않는다.**

레퍼런스(JSON 스키마·파라미터·결과 계약·검증 상태)는
**[`src/PACKAGES.md`](../PACKAGES.md#voice_processing)**.
설계 배경(역할 경계·대역폭·좌표계)은 **[`md/plans/2026-08-08-vla-integration.md`](../../md/plans/2026-08-08-vla-integration.md)**.
