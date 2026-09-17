# T1 CSV 검증과 학습 인계

작성 기준: 2026-09-17 · `MFIWO/soma-retargeter` / `t1-retarget-support` / `93cded49f8810adc181b1150f65182bb13727cfe`.

이 문서는 해당 commit의 코드·설정·기존 문서를 대조한 안내서다. 문서 정리 과정에서 PPO, 데이터 변환, GPU 평가 또는 실기 제어를 실행하지 않았다. 아래 명령은 데이터·checkpoint·환경이 준비된 작업용 머신에서 사용하는 템플릿이다. 실행 경로가 존재한다는 것과 학습 성능이 검증됐다는 것은 구분한다.

## CSV 계약

[CSV schema](https://github.com/MFIWO/soma-retargeter/blob/93cded49f8810adc181b1150f65182bb13727cfe/soma_retargeter/assets/csv.py) 기준:

| 필드 | 의미 |
|---|---|
| `Frame` | frame 식별자 |
| `root_translateX/Y/Z` | root 위치; downstream에 단위를 명시 |
| `root_rotateX/Y/Z` | xyz Euler, radians |
| `*_dof` | 23개 관절, schema의 고정 순서, radians |

CSV 자체의 열 이름만으로 FPS를 복원할 수 없다. source BVH frame time과 출력 frame 수/시간 범위를 함께 보관한다. quaternion을 쓰는 downstream에서는 Euler 변환과 xyzw/wxyz 순서를 명시한다. `Frame`이나 root Euler를 관절 action으로 읽지 않는다.

T1은 머리 2축, 팔 각 4축, 허리 1축, 다리 각 6축이다. CSV/MJCF 순서와 policy 또는 hardware action 순서는 별도로 대조한다.

## 필수 확인

1. 예상 CSV 수와 성공/실패/누락 BVH key를 대조한다. 동일 stem이 다른 디렉터리에 중복되는지 확인한다.
2. 열 이름·순서·frame 수, NaN/Inf, 비정상적으로 짧은 결과를 검사한다.
3. stance 발 미끄러짐/지면 침투, root 높이, 관절 limit 및 순간적인 IK branch 전환을 확인한다.
4. mirror는 단순 관절 부호 반전으로 만들지 않는다. 좌우 permutation, root 방향, 관절 axis·contact 위상을 확인한다.
5. 원본·retarget·FK preview를 같은 시간으로 비교한다. 처음/중간/끝 frame을 포함한다.
6. smoothing·IK 설정을 바꿨다면 이전 CSV와 섞지 않고 별도 출력·manifest를 만든다.


## T1 전용 FK audit

```bash
python app/audit_t1_retarget.py \
  --bvh /absolute/path/to/source.bvh \
  --csv /absolute/path/to/t1.csv \
  --retarget-config soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json \
  --fps 120 --output artifacts/t1_audit.json
```

`--fps 120`은 실제 CSV 주기에 맞춘다. contact-foot heading/tilt, stance width, root lean, 손/팔꿈치 오차, 관절 속도·가속도 tail과 18rad/s 초과 frame 수를 확인한다. 근사 COM support margin은 quasi-static 기하학 진단으로 ZMP·capture-point·동역학적 안정성 증명은 아니다.

기존 README가 가리키는 `assets/t1_retarget_gl_*` 예제 일부는 이 commit tree에 없다. 없는 파일을 실행하기보다 로컬 config에 지원된 preview 필드를 지정한다.


## 학습 저장소로 넘길 자료

- BVH와 CSV의 key 대응 및 원본/출력 hash.
- 이 repository의 commit, retarget/scaler/feet 설정과 모델 asset hash.
- 단위, axis, joint order, FPS·기간, mirror/family metadata.
- 변환 실패·제외 목록과 품질 보고서.

robot motion PKL 변환, SMPL/SOMA pairing, G1 weight 초기화는 [GR00T WBC T1 데이터 가이드](https://github.com/MFIWO/GR00T-WholeBodyControl/blob/t1-transfer-dev/docs/t1/01_data_retarget.md)에서 진행한다. 정책이 추종할 수 있는지는 physics rollout으로 별도 판단한다. retarget preview의 성공을 곧바로 실기 성공으로 기록하지 않는다.
