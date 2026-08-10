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

### 실행

```bash
ros2 run voice_processing get_keyword            # 마이크 경로 — launch 파일 없음
ros2 launch voice_processing vla_command.launch.py # VLA 경로
```

`get_keyword` 는 서비스 호출이 오디오 스트림을 열고 웨이크워드("hello_rokey")가 뜰 때까지
블로킹한다. 뽑힌 키워드는 노드 터미널의 `Detected tools: [...]` 로그로 보거나, 단독 트리거로
직접 확인한다:

```bash
ros2 service call /get_keyword std_srvs/srv/Trigger "{}"
```

응답 `message` 가 추출된 물체명(공백 join) — `task_manager` 는 이 중 **첫 단어만** target 으로 쓴다.
마이크 경로는 아직 실기 미검증.

레퍼런스(JSON 스키마·파라미터·결과 계약·검증 상태)는
**[`src/PACKAGES.md`](../PACKAGES.md#voice_processing)**.
설계 배경(역할 경계·대역폭·좌표계)은 **[`md/plans/2026-08-08-vla-integration.md`](../../md/plans/2026-08-08-vla-integration.md)**.
