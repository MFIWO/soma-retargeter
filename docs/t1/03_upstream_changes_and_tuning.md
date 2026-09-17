# T1 retarget: NVIDIA 원본 대비 변경과 팔·발 보정 기록

작성·소스 대조: 2026-09-17. 사용자 회고의 증상, Git에 남은 변경, 코드로 해석한 작동 원리를 구분한다. 이번 문서화에서 BVH 변환·GPU 학습·실기 검증을 새로 수행하지 않았다.

## 1. 당시 원본과 최신 원본을 구분한다

| 기준 | Commit·의미 |
|---|---|
| 출발점 NVIDIA SOMA v0.1 계열 | [`b3ef2708`](https://github.com/NVIDIA/soma-retargeter/tree/b3ef2708d84bfd1314ddb52d0db6c9c211df1f57), 2026-03-25. G1 config와 pipeline 기반 |
| fork H2 추가 | [`99f166e9`](https://github.com/MFIWO/soma-retargeter/commit/99f166e9045cb2bf52a2bfc1d5cfa89d9cee3f30), 2026-04-25. 위 원본 commit의 직접 자식 |
| fork T1 추가 | [`b2d7ce7a`](https://github.com/MFIWO/soma-retargeter/commit/b2d7ce7a5584c2bd290042baed83cf0c69256873), 2026-07-27 |
| balanced T1 | [`aefd449f`](https://github.com/MFIWO/soma-retargeter/commit/aefd449f7b2a5bf27c2bff9ef608051dfa9c330b), 2026-08-24 |
| 이번 T1 코드 snapshot | [`93cded49`](https://github.com/MFIWO/soma-retargeter/tree/93cded49f8810adc181b1150f65182bb13727cfe), 2026-08-24 |
| 조사 당시 최신 NVIDIA | [`1733b820`](https://github.com/NVIDIA/soma-retargeter/commit/1733b820f3cdf6f74bbc81a10bda3201b38c7bcf), 2026-09-15, SOMA v0.2.0 multi-embodiment release |

원본 v0.1에 BVH/scaling/Newton IK/joint-limit objective/feet stabilization/CSV export 기반이 있었다. fork는 이를 T1의 asset·축·길이·목표에 맞게 확장했다. 최신 v0.2에는 이미 Booster T1과 H2 config가 있으므로 “NVIDIA SOMA는 현재도 G1만 지원한다”고 쓰지 않는다.

v0.2의 [T1 설정](https://github.com/NVIDIA/soma-retargeter/blob/1733b820f3cdf6f74bbc81a10bda3201b38c7bcf/soma_retargeter/assets/robotics/booster/t1/configs/soma_to_booster_t1_retargeter_config.json)은 경로·`ik_match_table`·mask 배열 등 schema가 다르다. 여기의 v0.1 fork 수치를 최신 파일에 그대로 덮어쓰면 안 된다. 이번 작업은 최신 버전으로의 migration이나 품질 비교를 실행한 것이 아니다.

## 2. 팔이 접히지 않았다는 문제의 증거 범위

**사용자 회고:** T1 팔 구조가 G1과 달라 G1식 retarget에서 팔이 잘 접히지 않았고, 조정 후 개선됐다. 사용자 표현은 팔당 모터5개였다.

**코드 확인:** 이 브랜치의 CSV/제어 대상은 23 DoF이고 팔당 shoulder pitch/roll + elbow pitch/yaw 네 축이다. 하드웨어 다섯 번째 모터의 유무·역할·비활성 여부는 이 코드만으로 확정하지 않는다. 현재 문서의 관절 수는 이 제어 모델 기준이다.

아래 수정들은 팔의 불충분한 굽힘·목표 도달·IK branch 불안정과 관련된 구현이다. 다만 같은 BVH의 수정 전후 영상·정량 ablation이 commit에 함께 있지 않아 **어느 한 값이 당시 증상을 해결한 유일한 원인**이라고 확정하지 않는다.

## 3. G1 → 초기 T1 → balanced T1 수치

출처: [원본 G1 JSON](https://github.com/NVIDIA/soma-retargeter/blob/b3ef2708d84bfd1314ddb52d0db6c9c211df1f57/soma_retargeter/configs/unitree_g1/soma_to_g1_retargeter_config.json), [초기 T1 JSON](../../soma_retargeter/configs/t1/soma_to_t1_retargeter_config.json), [balanced T1 JSON](../../soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json).

| 항목 | 원본 G1 v0.1 | 초기 T1 | balanced T1 |
|---|---:|---:|---:|
| 초기화 / 안정화 frames | 10 / 5 | 120 / 30 | 120 / 30 |
| IK iterations | 24 | 50 | 50 |
| joint-limit weight | 10 | 40 | 40 |
| smooth joint weight | 5.5 | 35 | 35 |
| Arm position / rotation | 1.5 / 0.15 | 1.4 / 0 | 1 / 0 |
| ForeArm position / rotation | 1 / 1 | 4.5 / 0.1 | 12 / 0.05 |
| Hand position / rotation | 2 / 1.2 | 6 / 0.05 | 14 / 0 |
| 손 endpoint offset | 해당 IK map에 없음 | 좌 `[0, 0.04, 0]`, 우 `[0, -0.04, 0]` m | 동일 |
| Hips position / rotation | 30 / 2 | 35 / 8 | 35 / 10 |
| Shin position / rotation | 1 / 1 | 10 / 1 | 12 / 1 |
| Foot position / rotation | 30 / 2 | 30 / 2 | 28 / 8 |
| 팔 chain scaling | 0.85 | 0.68 | 같은 scaler + 길이 기반 목표 보정 |

양팔·양다리에 대칭으로 적용된 값이다. `t_weight/r_weight`는 IK 목표의 가중치이며 PD gain이 아니다. T1은 G1식 손 orientation을 강제로 맞추기보다 팔꿈치·손 **위치**를 더 강하게 맞추도록 수정됐다. 손 offset은 손 link 원점과 목표 endpoint가 다른 문제를 다룬다.

정확한 소스 링크: [초기 config](../../soma_retargeter/configs/t1/soma_to_t1_retargeter_config.json), [balanced config](../../soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json), [scaler](../../soma_retargeter/configs/t1/soma_to_t1_scaler_config.json).

## 4. '부드럽게 하는 gain' 세 종류를 섞지 않는다

[원본 IKSmoothJointFilter](https://github.com/NVIDIA/soma-retargeter/blob/b3ef2708d84bfd1314ddb52d0db6c9c211df1f57/soma_retargeter/pipelines/ik_objectives.py)는 현재 joint 좌표와 관절 범위를 이용한 nonlinear penalty다. residual에 이전 frame의 `q`를 빼는 항은 없다. 따라서 `smooth_joint_filter_weight=35`를 “35 Hz low-pass”나 “관절 속도 smoothing gain”으로 설명하면 잘못이다.

| 제어 장치 | 실제 역할 | T1에서의 사용 |
|---|---|---|
| smooth joint objective + body mask | 특정 joint가 관절 범위 내에서 받는 penalty를 선택·조절 | `AL3/AR3` mask는 초기 T1부터 0. 이 objective가 팔꿈치 굽힘을 누르는 경로를 제외하되 별도 joint-limit objective/clamper는 남음 |
| `joint_delta_limits` | frame 간 joint 변화량 제한 | 초기 shoulder pitch/roll 0.06/0.08 → balanced 0.04/0.04; elbow pitch 0.34→0.06; hand-link 축 0.20→0.05 rad/frame |
| `temporal_smoothing` | 출력 trajectory 시간축 filtering | balanced에서 median5 + 4차 low-pass 8 Hz 추가; root translation/rotation과 joint 처리 |

frame 단위 증분 제한은 FPS를 바꾸면 초당 허용 속도도 달라진다. 8 Hz filtering은 offline `sosfiltfilt`이며 실시간 causal filter가 아니다. root quaternion 부호 연속화·정규화와 마지막 joint-limit clamp도 수행한다. 과도한 filtering이 빠른 동작을 둔화할 가능성은 별도 품질 평가 대상이다.

초기 설정의 `Left_Shoulder_Yaw` 이름은 body map에서 `left_hand_link`를 가리킨다. balanced에서는 이를 `Left_Elbow_Yaw`로 명시했다. **이 이름 변경만으로 물리 관절이 새로 생긴 것은 아니다.** default roll은 좌 -1.48/우 +1.48, yaw는 좌 -0.3/우 +0.3; preferred yaw는 -0.4/+0.4로 branch 선택을 보조한다. 관련 구현은 [newton_pipeline.py](../../soma_retargeter/pipelines/newton_pipeline.py), [ik_objectives.py](../../soma_retargeter/pipelines/ik_objectives.py)다.

## 5. 팔 길이와 굽힘 여유를 직접 넣은 변경

balanced의 `_apply_target_conditioning()`은 단순 전체 키 scaling 뒤에 T1 팔의 두 segment 길이를 반영한다.

- upper arm 0.105 m, forearm 0.1871 m.
- 손까지 최대 거리를 `(0.105 + 0.1871) × 0.94 = 0.274574 m`로 제한.
- 최소 도달 거리도 두 segment 길이 차이에 맞게 제한.
- source 팔꿈치 plane에서 굽힘 방향을 복원하고, 이전 방향과 부호를 맞춘 뒤 smoothing 0.2 적용.
- 새 손 거리와 segment 길이로 elbow 위치를 재구성; blend1로 목표에 반영.

코드 주석은 완전 최대 reach에서 elbow plane이 불안정해져 작은 noise에 반대 IK branch로 넘어가는 문제를 설명한다. 94% 제한은 굽힘 여유를 두는 구현상 조치다. “팔이 안 접힘” 회고와 관련성이 있지만 이를 입증한 단독 ablation 결과는 없다.

config의 `elbow_position_targets`, `hand_endpoint_targets`, `elbow_yaw_branch_guard` 세 boolean은 확인한 pipeline에서 독립적으로 읽는 switch가 아니다. 실제 작동 경로는 `ik_map`, `limb_length_conditioning.enabled`, preferred pose와 delta limit이다. 세 boolean만 변경하면 기능이 켜지고 꺼진다고 안내하지 않는다.

## 6. 발목·접촉과 CSV 동결 문제까지 함께 보정

| 변경 | 이유·구현 |
|---|---|
| Foot heading | Hips 기준 relative yaw 목표0°, 최대 보정45°, blend1. 기준 heading과 발 방향을 맞추는 target conditioning |
| stance flattening | 가장 낮은 발 대비 높이 0.04 m 안에서 smoothstep 가중치로 roll/pitch를 평탄화. 실제 force sensor contact 판정은 아님 |
| virtual toe | offset `[0.118, 0, -0.018]` m, position weight18; foot orientation 보정과 toe 위치 회전을 함께 수행 |
| feet postprocessing | [balanced feet stabilizer](../../soma_retargeter/configs/t1/t1_balanced_feet_stabilizer_config.json)의 knee 12/5, foot 14/8, root35/10. 본 IK의 weight와 별도 단계 |
| frame snapshot 복사 | `joint_q_data[env][frame] = data[env].copy()`로 CPU Warp 배열 재사용 시 CSV 전 frame이 최종 pose로 같아지는 문제 방지 |
| 품질 감사 | [audit_t1_retarget.py](../../app/audit_t1_retarget.py): 발 방향·stance·근사 COM·팔 목표 오차·속도/가속도 tail. 근사 COM만으로 동적 안정성을 보장하지 않음 |

마지막 `.copy()` 수정은 관절 gain과 별개의 데이터 저장 오류다. 정적인 CSV를 보고 policy 학습 부족으로 잘못 진단하지 않도록 변경 이력에 함께 남긴다.

## 7. 실행·재현성에서 추가한 부분

원본 G1 asset 경로에서 로봇별 MJCF 선택으로 확장했고, T1에는 [격리 workspace asset 해석](https://github.com/MFIWO/soma-retargeter/commit/8ce9740854c6bdf8f5a9f58d3467433c59070f84), [deterministic process sharding](https://github.com/MFIWO/soma-retargeter/commit/93cded49f8810adc181b1150f65182bb13727cfe), balanced override·batch 관련 converter 옵션을 추가했다. 대용량 motion 처리와 품질 보정은 별개의 변화다. shard 재실행의 기존 파일 skip은 config hash나 CSV 무결성 검증까지 대신하지 않는다.

실행 명령과 감사 절차는 [pipeline](01_retarget_pipeline.md), [검증·학습 인계](02_validation_handoff.md)를 따른다. 재현 시 같은 BVH를 G1식 초기값/초기 T1/balanced로 처리하고 elbow ROM, hand endpoint error, limit saturation, 발 미끄러짐·회전, spike를 함께 비교해야 한다. 설정 파일·asset hash·FPS·실패 clip·전후 영상/수치를 남겨야 다음 기체에 실제 튜닝 순서를 넘길 수 있다.

현재 Git은 일부 최종 config와 묶음 commit을 보존한다. 사용자가 시도했던 모든 중간 gain과 당시 성공 clip을 복원할 수 있는 것은 아니다. 이 문서는 확인된 차이까지 기록하고 그 빈칸을 추측으로 채우지 않는다.
