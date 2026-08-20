# cobot2_ws — Source-to-Capability Map

## Version-pinned scope

- Extraction root: `/home/kimkh/cobot2_ws`; Git HEAD: `7270293356a59d0600d0315520e3e98f3526aeea` (`final_commit`, 2026-08-11).
- Same-turn source census: Python/C++/interface files under the stated exclusions total **49,817 LOC 🔵**; `corecode/` tutorial material is **3,329 LOC 🔵**. `cobot_rg2` includes a large Doosan/OnRobot vendor base; it is catalogued as an integration dependency, not personal authorship.
- Package roots covered: `cobot_rg2`, `cumotion`, `depth_downsample_cpp`, `graspgenx_perception`, `gripper_virtual_cpp`, `object_detection`, `pick_fsm`, `pick_fsm_msgs`, `planned_tcp_path_cpp`, `robot_safety_cpp`, `voice_processing`.
- Topology contrast from source: production pick path expects a fixed camera result in `base_link` (`task_manager.py:138,899-908`); calibration utility supports both eye-to-hand and eye-in-hand transforms (`calib_npy_to_tf.py:5-13`); regrasp eye-in-hand is explicitly a disabled scaffold without camera/extrinsic (`task_manager.py:124-128,1132-1148`).
- Sibling relationship (✅ 검증됨, 2026-08-20 diff audit): the team's actual final submission is `cobot2/협동2 제출/m0609_vla_ws`, not this workspace — this ws is a personal work-in-progress snapshot. `m0609_vla_ws` is the more complete/integrated version (full `vla_system`/`vla_interfaces` VLA layer, `pick_fsm` pause/resume/stow, matching `voice_processing` control forwarding, tracked YOLO weights — none present on this ws's `final_commit`).
- ⚠️ **C++ packages are personal study, not project deliverables**: `depth_downsample_cpp`, `gripper_virtual_cpp`, `planned_tcp_path_cpp`, `robot_safety_cpp` are hash-identical ports of the Python nodes that actually run the project, started as individual study a few hours before this extraction (2026-08-20). Unbuilt, not wired into any launch file, never run on the robot. Rows below that cite these C++ files describe standalone code behavior only — the matching Python file (same package tree, `.py` not `.cpp`) is what the project actually uses.

Legend: **●** central evidence, **○** real contact, blank = no source evidence claimed. Columns are the guide's fixed 11 axes.

| File / source unit | Sim | Vision | Frames | ROS | Node | Motion | State | Safety | Infra | Data | HRI |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `cobot_rg2/.../bringup.launch.py` | ○ | ● | ● | ● | ○ | ● |  | ○ | ● |  |  |
| `cobot_rg2/.../calib_npy_to_tf.py` |  | ○ | ● | ○ |  |  |  |  |  | ○ |  |
| `cobot_rg2/.../depth_downsample_node.py` |  | ● | ○ | ● | ● |  |  |  | ○ |  |  |
| `cumotion/reactive_replan.py`, `goal_setter_replan.py` | ○ |  | ○ | ● | ● | ● | ○ | ● | ● | ○ |  |
| `depth_downsample_cpp/...node.cpp` |  | ● | ○ | ● | ● |  |  |  | ○ |  |  |
| `graspgenx_perception/grasp_bridge_node.py` |  | ● | ● | ● | ● | ○ |  | ○ | ● | ● |  |
| `graspgenx_perception/capture_graspgenx_scene.py`, `yolo_seg_node.py` |  | ● | ● | ● | ● |  |  |  | ● | ● |  |
| `gripper_virtual_cpp/...node.cpp` | ● |  |  | ● | ● |  |  |  |  |  |  |
| `object_detection/setup.py` |  | ○ |  |  |  |  |  |  | ○ | ○ |  |
| `pick_fsm/task_manager.py`, `states.py` |  | ○ | ● | ● | ● | ● | ● | ● | ○ | ○ | ● |
| `pick_fsm/geometry.py`, `moveit_bridge.py` |  |  | ● | ● | ● | ● | ○ | ○ |  | ○ |  |
| `pick_fsm_msgs/srv/ComputeGrasp.srv` |  | ○ | ● | ● |  | ○ | ○ | ○ |  | ● |  |
| `planned_tcp_path_cpp/...node.cpp` |  |  | ● | ● | ● | ● |  |  |  | ○ |  |
| `robot_safety_cpp/...node.cpp` |  |  |  | ● | ● | ○ | ○ | ● |  |  |  |
| `voice_processing/vla_command_node.py` |  | ○ |  | ● | ● | ○ | ● | ● | ● | ○ | ● |
| `voice_processing/get_keyword.py`, `approve_listener_node.py` |  |  |  | ● | ● |  | ○ | ● | ○ |  | ● |
| `corecode/Calibration_Tutorial/*.py` (학습) |  | ● | ● | ○ |  |  |  |  |  | ● |  |
| `corecode/GraspSelection/*.py`, `OD_Tutorial/*.py` (학습) |  | ● | ○ |  |  |  |  |  |  | ● |  |
| `corecode/VoiceProcessing/*.py` (학습) |  |  |  | ○ | ○ |  |  |  |  |  | ● |

## File-by-axis detail and contribution boundary

| Source | Evidence and axis | Boundary |
|---|---|---|
| `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/calib_npy_to_tf.py:5-13,28-46` | eye-to-hand / eye-in-hand input choice and optical-to-body REP-103 conversion [좌표계]. | Library `scipy` rotation utilities perform matrix conversion / I configured the transform convention and CLI mapping; ownership of this custom script is not established by source alone. |
| `src/cobot_rg2/rg2/m0609_rg2_bringup/scripts/depth_downsample_node.py:46-78` | dual-resolution depth, CameraInfo gate and ROS pub/sub [비전][통신][노드설계]. | ROS/OpenCV transport and resize do the primitives / I cannot infer personal authorship from source. |
| `src/cumotion/cumotion/reactive_replan.py:825-972`; `goal_setter_replan.py:4-52` | action handover/replan experiment, explicit unverified preemption note [모션플래닝][안전][데이터]. | MoveIt/cuMotion solve planning / this code integrates and compares execution paths; comments say the material follows an NVIDIA example, so personal contribution is **⚠️ boundary unresolved**. |
| `src/depth_downsample_cpp/src/depth_downsample_node.cpp:29-50,68-130` | SensorDataQoS depth relay; nearest resize and K/P intrinsic scaling [비전][통신][노드설계]. | cv_bridge/OpenCV do conversion/resize / the C++ port preserves image and camera-model invariants. |
| `src/graspgenx_perception/graspgenx_perception/grasp_bridge_node.py:199-546` | grasp service/pose bridge [비전][좌표계][통신]; tests include round-trip/selection cases. | GraspGenX/YOLO provide model inference / bridge selects, transports and exposes the ROS contract; personal ownership must be attributed separately. |
| `src/graspgenx_perception/graspgenx_perception/capture_graspgenx_scene.py:166-830`; `yolo_seg_node.py:207-431` | scene capture and segmentation nodes [비전][노드설계][인프라]. | Camera/model libraries acquire and infer / node integration owns ROS-facing orchestration. |
| `src/gripper_virtual_cpp/src/gripper_virtual_node.cpp:155-159,192-217,260-334` | virtual gripper simulation uses Reentrant group, mutex, timer and MultiThreadedExecutor [시뮬][통신][노드설계]. | rclcpp schedules callbacks / the port specifies the concurrency and state-protection design. |
| `src/object_detection/setup.py:1-31` | package has setup/tests but no detector implementation [비전][인프라]. | No inference library or built system is evidenced / it is a share-path/package shell only. |
| `src/pick_fsm/pick_fsm/task_manager.py:83,214-239,263-401,897-908,1132-1148` | durable targets, service selection, Reentrant FSM I/O, frame rejection, disabled regrasp scaffold [통신][노드설계][상태관리][안전][HRI]. | MoveIt/ROS execute primitives / FSM design owns gates, contracts and failure routing. |
| `src/pick_fsm/pick_fsm/geometry.py:20-25,42-138`; `moveit_bridge.py:132-300` | raw grasp→RG2 base orientation, local-Z approach, `base_link` planning scene [좌표계][모션플래닝]. | SciPy/MoveIt perform math/IK / integration code defines conventions, offsets and requests. |
| `src/pick_fsm_msgs/srv/ComputeGrasp.srv:1-42` | typed grasp, width and alternatives contract [통신][좌표계][데이터]. | ROSIDL generates bindings / interface design preserves raw-frame and width semantics. |
| `src/planned_tcp_path_cpp/src/planned_tcp_path_node.cpp:1-13,48-54,138-185` | nonblocking FK fan-out/fan-in to TCP `LINE_STRIP` [통신][노드설계][모션플래닝]. | MoveIt FK answers requests / node batches responses and visualizes ordered TCP path. |
| `src/robot_safety_cpp/src/robot_safety_node.cpp:76-118,218-219` | robot state services plus `/safety/stop` → `MoveStop(HOLD)` [통신][노드설계][안전]. | Doosan controller enforces stop / node exposes an independent service path; actual actuation is untested here. |
| `src/voice_processing/voice_processing/vla_command_node.py:66-70,337-496,556-558,795-878` | VLA latch, QoS/status, command forwarding; external VLA cannot call approve [통신][상태관리][안전][HRI]. | ROS handles transport / integration defines one-command latch and human-approval boundary. |
| `src/voice_processing/voice_processing/get_keyword.py:38-94,169-209`; `approve_listener_node.py:88-208` | OpenAI-backed keyword/STT and state-scoped voice approval [HRI][통신][안전]. | OpenAI/STT model extracts speech / nodes constrain invocation and route result; live API behavior is unverified. |
| `corecode/Calibration_Tutorial/eye2hand_calibration.py:644`; `GraspSelection/grasp_selector.py:92`; `VoiceProcessing/keyword_extraction.py:28` | calibration, grasp selection and speech exercises [학습]. | Libraries/tutorial code demonstrate concepts / these are explicitly learning material, not production nodes or delivered systems. |

## Census completeness

All 11 requested package roots appear above with an anchor. `cobot_rg2` includes external driver code and custom RG2 bringup scripts; all `corecode/` evidence is deliberately labelled 학습. The source map makes no claim that every vendor implementation line was authored or production-integrated.
