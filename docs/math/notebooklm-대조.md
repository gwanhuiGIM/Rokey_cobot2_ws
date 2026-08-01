---
updated: 2026-07-31
source: NotebookLM "cobot2_ws — 강의자료 ↔ 내 코드"
---

# NotebookLM 대조 결과 (1차)

> 노트북: https://notebooklm.google.com/notebook/cd7cc3d9-757c-4e1d-9633-8402d3514693
> 소스 7개: coordinate-transform.md, 없는-것.md, 협동로봇2_강의자료.pdf,
> 협동2_AI_강의자료.pdf, README(pick_and_place_voice), README(cobot_rg2), CLAUDE.md
> **교재(Modern Robotics / DRL / RG2 매뉴얼)는 아직 없다.** 아래 "근거 없음"의 대부분이 그 탓이다.

## Q1 — 내 구현 vs 강의자료, 어긋나는 지점

| 항목 | 내 문서 | 강의자료 | 판정 |
|---|---|---|---|
| 변환 사슬 | `base2cam = base2gripper @ gripper2cam` | \(T^{target}_{base} = T^{gripper}_{base} T^{camera}_{gripper} T^{target}_{camera}\) | **일치.** 어긋난 건 튜토리얼 코드의 변수명(`T_cam2base`)뿐 |
| `T_gripper2camera` | 교육자료 제공 파일을 로드 | 체커보드 20장+ 직접 수집 후 `handeye_calibration.py`로 산출 | **현장 제약** (교육 환경). 구현 실수 아님 |
| 회전 표현 | ZYZ로 받아 즉시 행렬 변환, 짐벌락 회피 | 회전 규약·짐벌락 **언급 없음** | **하드웨어 제약**(두산 규약). 강의자료 누락 이유는 근거 없음 |
| `det` 검사 | "항상 1이라 무의미" | 해당 없음 | **튜토리얼 코드 논리 오류** — 내 지적이 맞다 |

→ 결론: 강의자료와 내 문서가 **수학적으로 어긋나는 지점은 없다.** 어긋난 건 전부 교육용 튜토리얼
코드의 결함(변수명, det 검사)이고, 그건 이미 `coordinate-transform.md` §6에 잡혀 있다.

## Q2 — `movel` 이후 컨트롤러가 하는 일

- 확인된 것: IK는 두산 펌웨어가 푼다. 6축은 closed-form 해가 있고 최대 8개(어깨 좌/우 × 팔꿈치 위/아래 × 손목 뒤집힘).
- **강의자료에 해 선택 기준이 없다.** 강의자료는 Vision/YOLO/캘리브/픽앤플레이스 응용에 집중되어 있고
  기구학 이론 챕터가 아예 없다. NotebookLM이 준 선택 기준(최단 관절 이동, 관절 한계, 특이점·자가충돌 회피,
  configuration flag)은 **소스 밖 일반 지식**이라 표시된 것 — 인용 근거 없음. 면접에서 이대로 말하면 안 된다.
- → **교재(Modern Robotics 6장 / Craig 4장)를 넣기 전까지 이 질문은 답이 안 나온다.**

## Q3 — 면접 질문 10개 (저장소 근거 유무)

**저장소에 근거 있음 (5개)**
1. 픽셀 → 베이스 3D 변환 — `coordinate-transform.md` §1·§3
2. ZYZ를 쓰는 이유 / 짐벌락 — §2
3. `det T = 1` 검사가 무의미한 이유 — §6①
4. 자세 추정 부재와 PCA 접근 — `없는-것.md` §3
5. 블로킹 서비스 호출 · busy-wait 웨이크워드 — `README(pick_and_place_voice)` 알려진 문제

**저장소에 근거 없음 → 교재 필요 (5개)**
6. IK 해 선택 기준 — `없는-것.md` §1에 **질문으로만** 존재
7. `movej` vs `movel` 궤적 차이 — §4에 질문으로만
8. 위치/힘 제어 동시 불가, 임피던스 vs 어드미턴스 — §5에 질문으로만
9. 특이점 회피(DLS) vs 유사역행렬 — §2에 질문으로만
10. (9와 같은 뿌리) 야코비안 rank 손실 → 속도 발산의 SVD 설명

## Q4 — 6~10번 재질의 (Modern Robotics 추가 후, 2026-07-31)

> 소스: `docs/modern_robotics_lynch_park.pdf` (Lynch & Park, Dec 2019 preprint) 추가 후 재질의.
> 노트북 대화: https://notebooklm.google.com/notebook/cd7cc3d9-757c-4e1d-9633-8402d3514693 (conversation `dd5e369e-cf61-4743-99e0-8e66fa1b2d48`)

| # | 질문 | 근거 | 판정 |
|---|---|---|---|
| 6 | IK 8개 해 중 선택 기준 | MR 6.2절(수치해는 초기 추정값에 가장 가까운 해로 수렴 → 이전 스텝 해를 다음 초기값으로 써서 최단 이동 유도), 10.1절(관절한계·충돌회피는 모션 플래너의 몫) | **부분 근거.** "최단 이동"은 수치 IK 알고리즘의 부작용으로 설명됨. Configuration flag(elbow-up/down 등)로 명시적 선택은 **소스 밖 일반 지식** |
| 7 | movej vs movel | MR 9.2.1절 — 관절공간 직선은 $\Theta_{free}$가 convex라 안전·단순하지만 말단이 직선으로 안 움직임. 작업공간 직선(movel)은 특이점 근처에서 관절속도 발산 위험 + 도달불가 지점 포함 가능 | **일치.** `coordinate-transform.md`/`없는-것.md` 서술과 부합 |
| 8 | 위치/힘 동시 제어 불가, 임피던스 vs 어드미턴스 | MR 1장 Preview + 11.7절 — "로봇이 모션을 강제하면 환경이 힘을 결정, 반대도 마찬가지"가 근본 이유. 임피던스=모션 측정→힘 명령, 어드미턴스=힘(손목 F/T센서) 측정→모션 명령 | **근거 있음.** 이론 챕터 그대로 있음 |
| 9 | DLS vs pseudoinverse | MR 6.2.2절엔 pseudoinverse($J^\dagger$) 수식만 있고 **DLS는 소스에 없음** | **소스 밖 일반 지식.** DLS는 오차항에 관절속도 크기(감쇠항)를 추가해 특이점 근처 속도 폭주를 막는 것 — 교재에 없으므로 인용 근거로 못 씀 |
| 10 | 야코비안 rank 손실 → SVD 설명 | MR 5.3·5.4절 — rank 손실 정의·manipulability ellipsoid가 선분으로 붕괴하는 정성적 설명만 있음. **SVD 전개는 소스에 없음** | **소스 밖 일반 지식.** $J=U\Sigma V^T$, $\dot\theta=V\Sigma^{-1}U^T\dot x$에서 $\sigma_{min}\to0$이면 발산하는 설명은 교재 밖 |

→ 5개 중 **7·8번은 교재 근거로 확보**, 6번은 절반(수치해 수렴 거동은 있음, configuration flag는 없음), **9·10번은 여전히 근거 없음** — MR이 rank 손실/특이점은 정성적으로만 다루고 DLS·SVD 전개는 아예 안 다룬다. 면접에서 9·10은 "Modern Robotics 5·6장이 정성적 근거, 정량적 SVD 전개는 일반 수치선형대수 지식"이라고 구분해서 말해야 한다.

## 다음
- [x] Modern Robotics PDF 확보 후 Q2·Q3의 6~10번 재질의 (Q4로 기록)
- [ ] 9·10번(DLS, SVD) 근거는 Craig 4장이나 별도 수치해석 자료가 있어야 채워짐 — 필요하면 추가 소스 검토
- [ ] `없는-것.md`가 "NotebookLM에 물을 것"으로 남겨둔 질문들이 그대로 미해결 5개와 일치 — 교재만 들어오면 바로 돌릴 수 있다

## 관련
- [[ws/cobot2/math/coordinate-transform|축 1 — 좌표 변환]]
- [[ws/cobot2/math/없는-것|축 2 — 없는 것]]
- [[ws/cobot2/state|state]]
