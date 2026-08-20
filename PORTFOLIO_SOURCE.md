# cobot2_ws — Portfolio Source

## 0. Extraction boundary and version

- Source-only extraction: `/home/kimkh/cobot2_ws`, HEAD `7270293356a59d0600d0315520e3e98f3526aeea` (`final_commit`, 2026-08-11); generated `build/`, `install/`, `log/`, and `isaac_ros-dev/` are excluded.
- **49,817 LOC 🔵** of Python/C++/ROS interface source and **3,329 LOC 🔵** of `corecode/` tutorials were counted in this turn. These are source census figures, not lines personally authored or runtime coverage.
- **This workspace is a personal work-in-progress snapshot, not the team's submission.** The team's final submission is `cobot2/협동2 제출/m0609_vla_ws` (own `PORTFOLIO_SOURCE.md`/`SKILL_INVENTORY.md`/`PORTFOLIO_MAP.md` there). ✅ **검증됨** (2026-08-20 diff audit, re-verified independently against source/git): `m0609_vla_ws` is the more complete/integrated version — it has the full `vla_system`/`vla_interfaces` VLA agent+mission layer (11,421 LOC), `pick_fsm` pause/resume/stow/release_now services, matching `voice_processing` control forwarding, and tracked YOLO weight files, none of which are present on this workspace's HEAD `7270293` (`final_commit`, 2026-08-11). A VLA-integration branch (`personal/vla_integed`, likely a teammate's) exists in this repo's remotes but was never merged into `final_commit` and is itself thinner than `m0609_vla_ws`. The four C++ packages here (`depth_downsample_cpp`, `gripper_virtual_cpp`, `planned_tcp_path_cpp`, `robot_safety_cpp`) are **personal study started a few hours before this extraction (2026-08-20)** — hash-identical ports of the Python nodes that already run the actual project (also present in `m0609_vla_ws`). They are unbuilt/unintegrated and were never run on the project robot; not unique shipped content, not a gap, and not a project deliverable (see §8 Language/runtime optimization for the boundary label carried through the rest of this document). Conclusion: for portfolio purposes, treat `m0609_vla_ws` as authoritative; this workspace documents the in-progress path, not additional shipped capability.

## 1. 문제 정의 (Why)

Perception-originated target/grasp results need a typed, frame-safe handoff to a robot pick sequence; otherwise a pose can be interpreted as tool TCP instead of RG2 base, a stale VLA command can be consumed at the wrong FSM state, or an external client could silently bypass human approval. The source addresses these software failure modes through `ComputeGrasp` semantics (`pick_fsm_msgs/srv/ComputeGrasp.srv:17-42`), state-dependent VLA latching (`voice_processing/vla_command_node.py:795-843`), and a separate stop service (`robot_safety_cpp/src/robot_safety_node.cpp:101-118`). Physical production effectiveness is **⚠️ unverified** in this extraction.

## 2. 담당 역할 & 기여 범위

- The repository combines custom integration, learning ports/tutorials, and upstream Doosan/OnRobot sources. Source alone cannot prove individual/team attribution; each technical item below states the boundary instead of inferring authorship.
- Upstream driver, controller, message and OnRobot implementation under `src/cobot_rg2/doosan-robot2` and `onrobot-ros2` are dependency/vendor material. Custom-facing integration evidence includes the RG2 bringup utilities (`src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py:5-66`) and FSM/perception/voice packages.
- `corecode/bench.py:1-128`, `Calibration_Tutorial/`, `DRL_Tutorial/`, `GraspSelection/`, `OD_Tutorial/`, and `VoiceProcessing/` are **학습** material per task scope, not a claim of a production node or built system.

## 3. 시스템 아키텍처

`camera/depth → segmentation/scene capture → GraspGenX bridge → ComputeGrasp or legacy trigger → task_manager FSM → MoveIt IK/action → RG2/Doosan services`.

| Subsystem | Evidence | Function |
|---|---|---|
| Camera/bringup | `cobot_rg2/.../depth_downsample_node.py:46-78` | Publishes downsampled depth and correctly scaled intrinsics. |
| Perception/grasp | `graspgenx_perception/grasp_bridge_node.py:199-546`; `pick_fsm_msgs/srv/ComputeGrasp.srv:8-42` | Produces typed best grasp plus alternatives and widths. |
| FSM/motion | `pick_fsm/task_manager.py:263-401,988-1134`; `pick_fsm/moveit_bridge.py:132-300` | Gates command/grasp, calculates three poses, requests IK/MoveGroup. |
| Safety/visualization | `pick_fsm/robot_safety_node.py:70-186`; `pick_fsm/planned_tcp_path_node.py:35-110` | Separates HOLD stop interface and visualizes planned TCP from FK. |
| Human/VLA interface | `voice_processing/vla_command_node.py:337-496,795-878`; `approve_listener_node.py:88-208` | Latches VLA/voice input, retains human-only approval policy. |

### 3-1. 결합 지점 — quiet-failure map

| Coupling | Must match | Hardware/topology assumption | Evidence |
|---|---|---|---|
| Grasp frame → tool goal | Producer sends raw GraspGenX frame; consumer applies exactly one RG2-base rotation; `base_frame=base_link`. | Production pick code expects fixed eye-to-hand result already expressed in `base_link`; it does not establish a wrist camera. | `ComputeGrasp.srv:17-24`; `geometry.py:20-57`; `task_manager.py:897-908` |
| Calibration transform | Parent/child and optical/body convention must agree. | Utility supports either fixed camera eye-to-hand or wrist eye-in-hand; this is a utility capability, not evidence both are installed. | `calib_npy_to_tf.py:5-13,28-46` |
| Regrasp | `regrasp_enabled` stays false until camera/extrinsic and capture hook exist. | eye-in-hand is only scaffold: no camera or hand-eye calibration in that path. | `task_manager.py:124-128,1132-1148` |
| Target/place QoS | target/place producers and FSM use durable `TRANSIENT_LOCAL`; commands carry request identity. | ROS/DDS network topology only; no real multi-PC operation is demonstrated here. | `task_manager.py:83,324-330`; `vla_command_node.py:423-445` |
| Depth + CameraInfo | Same source image model and correctly scaled K/P must travel together. | RealSense topic names are defaults; camera hardware attachment/USB behavior is not verified in this turn. | `cobot_rg2/.../depth_downsample_node.py:41-117` |
| Motion authority | MoveGroup/Doosan stop services use the namespace/service contract. | Actual robot controller, RG2 and stop semantics require human-approved physical validation. | `pick_fsm/robot_safety_node.py:70-186`; `moveit_bridge.py:132-143` |
| Approval | VLA `approve` is rejected; FSM approval service is separate; voice listener only runs in WAIT_APPROVAL. | Voice input is intended as human speech, not external VLA autonomy; microphone contention remains a source-noted risk. | `vla_command_node.py:66-70,556-558`; `approve_listener_node.py:30-44,167-208` |

## 4. 핵심 기술 의사결정

1. [좌표계][안전] Raw grasp pose is not called TCP; transform it once at the FSM boundary, then reject unexpected frame IDs. Evidence: `ComputeGrasp.srv:17-24`; `task_manager.py:897-908`; `geometry.py:42-57`.
   - 라이브러리 ROS messages/SciPy가 pose storage·quaternion math를 한다 / 내가 한 건 원시 프레임 계약과 단일 변환 경계를 정한 것이다.
2. [상태관리][통신][HRI] VLA commands are latched until FSM pulls during LISTENING, and an external `approve` command is blocked. Evidence: `vla_command_node.py:24-29,192,337-496,795-843`.
   - 라이브러리 rclpy/DDS가 service·topic transport를 한다 / 내가 한 건 state-aware command consumption and human-gate policy를 통합한 것이다.
3. [모션플래닝][안전] Build pre-grasp/grasp/lift poses with local approach axis and constrain replan settings at the MoveGroup request. Evidence: `geometry.py:75-115`; `moveit_bridge.py:182-224`.
   - 라이브러리 MoveIt가 IK·trajectory planning을 한다 / 내가 한 건 pose sequence, planning options, and failure boundary를 구성한 것이다.
4. [비전][노드설계] Downsample depth with nearest-neighbor and scale CameraInfo K/P rather than publishing geometry with stale intrinsics. Evidence: `cobot_rg2/.../depth_downsample_node.py:41-117`.
   - 라이브러리 OpenCV/cv_bridge가 image conversion·resize를 한다 / 내가 한 건 camera-model-preserving relay 규칙을 구현한 것이다.
5. [안전][통신] Keep a controller-facing `MoveStop(HOLD)` service separate from FSM. Evidence: `pick_fsm/robot_safety_node.py:70-186`.
   - 라이브러리 Doosan controller가 HOLD를 수행한다 / 내가 한 건 ROS service boundary와 async request dispatch를 둔 것이다.
6. [노드설계][시뮬] Virtual gripper uses Reentrant callback group, mutex and MultiThreadedExecutor so a blocking command service does not prevent its timer from reaching target. Evidence: `cobot_rg2/.../gripper_virtual_node.py:25-106`.
   - 라이브러리 rclpy가 executor scheduling을 한다 / 내가 한 건 concurrency precondition and shared-state locking을 둔 것이다.

## 5. 문제 해결 사례 (source-grounded STAR)

- **S/T:** grasp pose can look valid but be a different end-effector convention. **A:** interface documents raw RG2-base origin and FSM performs one conversion/rejects a mismatched frame. **R:** prevents a silent double rotation/TCP interpretation at the code boundary; real motion result **⚠️ 미검증**. Evidence: `ComputeGrasp.srv:17-32`; `task_manager.py:899-908`.
- **S/T:** external VLA push timing does not align with FSM pull timing. **A:** command latch checks state, TTL and request identity before `/get_keyword` reply. **R:** source contains deterministic rejection/timeout paths; end-to-end VLA pick cycle **⚠️ 미검증**. Evidence: `vla_command_node.py:136-159,703-843`.
- **S/T:** a depth image resize without intrinsics update changes projection geometry without an error. **A:** node gates until CameraInfo, uses nearest-neighbor, rescales K/P. **R:** camera-model invariant is encoded; camera test **⚠️ 미검증**. Evidence: `cobot_rg2/.../depth_downsample_node.py:41-117`.

## 6. 정량 성과 & 한계

- **49,817 LOC 🔵** source census under requested extensions/exclusions; **3,329 LOC 🔵** tutorial-only `corecode/` census. These are not personal LOC or coverage metrics.
- `dynamic_avoid.launch.py` defaults replan frequency to **3.0 Hz ⚠️** (configuration value, not observed rate): `cumotion/launch/dynamic_avoid.launch.py:42,76`.
- Limitation: `object_detection` contains package shell/tests but no detector implementation; do not represent it as an in-workspace trained detector. Evidence: `object_detection/setup.py:1-31`.
- Limitation: reactive-replan material documents NVIDIA-example lineage and unverified preemption behavior. Evidence: `cumotion/goal_setter_replan.py:4-52`.

## 7. 역량 태그 요약

[시뮬] [비전] [좌표계] [통신] [노드설계] [모션플래닝] [상태관리] [안전] [인프라] [데이터] [HRI]

All 11 have source evidence; relative depth is summarized in the final inventory rather than treated as a measured skill score.

## 8. 기반 기술 요소 (§3-A)

| Element | Project contact | Why it matters |
|---|---|---|
| Sensor bandwidth / camera model | Depth decimation preserves depth edges and scales intrinsics: `cobot_rg2/.../depth_downsample_node.py:41-117`. | Pixel-to-ray geometry fails silently if image and CameraInfo diverge. |
| CPU/GPU/process boundary | Grasp worker/bridge source exists: `graspgenx_perception/graspgen_worker.py:168-286`; exact GPU/latency observation is ⚠️ unverified. | Model process placement affects controllability and recovery. |
| Hardware API/configuration | Modbus controller implementation exists in vendor package: `cobot_rg2/onrobot-ros2/onrobot_rg_control/.../comModbusTcp.py:1-114`; Doosan stop client is used by safety node. | Hardware service contract and unit range are safety-critical; vendor behavior is not personal implementation. |
| Language/runtime optimization | ⚠️ **개인 학습 — 실기 미투입.** Four C++ ports of the production Python nodes above (`depth_downsample_cpp`, `gripper_virtual_cpp`, `planned_tcp_path_cpp`, `robot_safety_cpp`) were started as personal study a few hours before this extraction (2026-08-20); hash-identical Python originals are what actually ran/runs on the project. The C++ code has not been built into the running system or exercised on the robot — treat as a study exercise, not a project deliverable. Evidence they are ports, not new capability: `depth_downsample_cpp/src/depth_downsample_node.cpp:1-6`, `gripper_virtual_cpp/src/gripper_virtual_node.cpp:1-10`, `planned_tcp_path_cpp/src/planned_tcp_path_node.cpp:1-14`, `robot_safety_cpp/src/robot_safety_node.cpp` (matches `pick_fsm/robot_safety_node.py:70-186`). | Executor/serialization/shared-state concerns surfaced while porting are a legitimate Python→C++ learning artifact — see §9 for what a real integration would still require (build, launch wiring, on-robot validation). |

## 9. Future Work / 심화 학습 계획

- Complete an eye-in-hand regrasp only after a real camera, flange-to-camera extrinsic and approved calibration validation; source currently marks it scaffold-only (`task_manager.py:1132-1148`). [좌표계][비전][안전]
- Establish an executable benchmark for the two replan paths; source calls out unverified MoveGroup preemption (`goal_setter_replan.py:45-52`). [모션플래닝][데이터]
- Add camera hardware/latency and real stop-service tests before claiming physical performance. [비전][인프라][안전]
- Resolve microphone stream ownership between keyword and approval nodes before deployment (`approve_listener_node.py:30-44`). [HRI][노드설계]
- Carry the personal-study C++ ports (§8 Language/runtime optimization) through to an actual on-robot A/B latency comparison against the Python originals — currently unbuilt/unintegrated, started 2026-08-20. [노드설계][인프라]

## 10. 전공 기반 매핑 (§3-B)

| 전공 | 코드에 실제로 닿은 과목 | 실제 지점 | 받치는 기능 축 | §0-1 경계 |
|---|---|---|---|---|
| 수학 | 선형대수/강체변환, quaternion | `geometry.py:20-57,75-138`; `calib_npy_to_tf.py:28-46` | [좌표계][모션플래닝] | SciPy/ROS가 rotation/message 연산을 한다 / 내가 한 건 frame convention, offset, application boundary를 정한 것이다. |
| CS / 컴퓨터공학 | OS 동시성, 네트워크(DDS), 자료구조(FSM) | `cobot_rg2/.../gripper_virtual_node.py:43-88`; `task_manager.py:83,263-401`; `vla_command_node.py:703-843` | [통신][노드설계][상태관리][HRI] | rclpy가 executor/DDS를 제공한다 / 내가 한 건 callback ownership, QoS, latch/FSM policy를 통합한 것이다. C++ concurrency port(§8)는 개인 학습이며 여기 근거로 쓰지 않았다. |
| 기계공학 | 로봇 기구학, TCP/end-effector geometry | `ComputeGrasp.srv:17-32`; `moveit_bridge.py:132-300` | [좌표계][모션플래닝] | MoveIt/URDF가 IK·collision model을 한다 / 내가 한 건 RG2-base/TCP semantic and pose sequence를 연결한 것이다. |
| 제어공학 | trajectory execution, stop mode | `reactive_replan.py:825-972`; `pick_fsm/robot_safety_node.py:144-186` | [모션플래닝][안전] | MoveIt/Doosan controller가 trajectory/PID/HOLD를 수행한다 / 내가 한 건 replan handover and stop-service integration을 둔 것이다. C++ port(§8)는 개인 학습이며 실기 미투입. |
| 전자공학 | camera sensor interface, Modbus TCP gripper | `cobot_rg2/.../depth_downsample_node.py:41-78`; `comModbusTcp.py:1-114` | [비전][인프라][안전] | RealSense/OnRobot hardware and vendor driver perform electrical/protocol work / 내가 한 건 source-level integration point를 사용한 것이며 회로 설계 주장은 하지 않는다. |

This lens is separate from §7: CS is source-thick; control/mechanical knowledge is integration-heavy with planner/controller boundaries; electronics is configuration/interface-level and therefore thin.
