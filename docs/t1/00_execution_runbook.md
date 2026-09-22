# SOMA → T1 retarget 실행 및 검증 절차

문서 개정: 2026-09-22 · 원격 대조: `10f722de85d1b9147d932b19f062758108cbad51`.

문서 기준: 2026-09-17 · `t1-retarget-support` · 검토 코드 `93cded49f8810adc181b1150f65182bb13727cfe`.

## 1. 기존 fork 재현 범위 및 준비 사항

본 절차는 사람 SOMA BVH를 T1 23-DoF CSV로 변환하는 offline 작업이다. 정지·걷기·회전·팔 굽힘을 포함한 소수 clip으로 설정을 확인한 후 전체 corpus로 확대한다. 명령은 Bash와 retarget 저장소 루트 기준이며 `/absolute/path/...`는 실제 경로로 교체한다.

| 준비 항목 | 내용 |
|---|---|
| Python 환경 | Python 3.12 이상, 해당 branch의 pinned dependencies |
| 원본 motion | SOMA BVH와 원본 Frame Time/FPS |
| 로봇 asset | WBC checkout의 `T1_23dof.xml` 및 참조 mesh |
| 출력 위치 | retarget 설정별 별도 디렉터리 |

```bash
set -euo pipefail
git clone --branch t1-retarget-support --single-branch \
  https://github.com/MFIWO/soma-retargeter.git soma-retargeter-t1
cd soma-retargeter-t1
git lfs install
git lfs pull
conda create -n soma-retargeter-t1 python=3.12 pip tk -y
conda activate soma-retargeter-t1
python --version
python -m pip install -e .
git rev-parse HEAD
```

이미 동일한 전용 환경이 있으면 `conda create`를 생략하고 해당 환경을 활성화한다. 기존 checkout에서는 clone을 생략한다. PPO용 Python 환경과 retarget 의존성을 동일 환경에 설치할 필요는 없다.

## 2. T1 모델 및 입력·출력 경로 지정

```bash
export GROOT_WBC_ROOT=/absolute/path/to/GR00T-WholeBodyControl-t1
export SOMA_T1_MJCF_PATH="$GROOT_WBC_ROOT/gear_sonic/data/assets/robot_description/mjcf/T1_23dof.xml"
export SOMA_BVH_DIR=/absolute/path/to/small_bvh_subset
export SOMA_CSV_DIR=/absolute/path/to/t1_csv_balanced_v1
mkdir -p artifacts

test -f "$SOMA_T1_MJCF_PATH"
test -d "$SOMA_BVH_DIR"
python - <<'PY'
from soma_retargeter.pipelines.utils import TargetType, get_robot_mjcf_path
print(get_robot_mjcf_path(TargetType.T1))
PY
```

**확인 결과:** 지정한 T1 MJCF 절대 경로가 출력된다. asset의 상대 mesh 경로도 해당 checkout에서 유효해야 한다.

converter용 로컬 JSON을 생성한다. `retargeter_config`는 T1 balanced 설정을 명시한다.

```bash
python - <<'PY'
import json, os
from pathlib import Path
config = {
    "import_folder": os.environ["SOMA_BVH_DIR"],
    "export_folder": os.environ["SOMA_CSV_DIR"],
    "batch_size": 4,
    "retargeter": "Newton",
    "retarget_source": "soma",
    "retarget_target": "t1",
    "retarget_source_facing_direction": "Mujoco",
    "retargeter_config": "soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json"
}
Path("artifacts/t1_local.json").write_text(json.dumps(config, indent=2) + "\n")
PY
python -m json.tool artifacts/t1_local.json
```

## 3. 소수 motion retarget 및 품질 검사

### 3.1 CSV 생성

```bash
python app/bvh_to_csv_converter.py \
  --config artifacts/t1_local.json --viewer null --device cpu \
  2>&1 | tee artifacts/t1_retarget.log
```

**산출물:** `SOMA_CSV_DIR` 하위 CSV와 실행 로그. CPU 예시는 소규모 기능 확인용이다. 기존 CSV가 있으면 converter가 건너뛸 수 있으므로 gain/config를 변경한 비교 실험은 새 출력 디렉터리로 수행한다.

### 3.2 파일 형식·관절 순서 검사

다음 검사는 생성된 CSV header를 코드의 T1 schema와 직접 대조하고 유한값 여부를 확인한다.

```bash
python - <<'PY'
import csv, os
from pathlib import Path
import numpy as np
from soma_retargeter.assets.csv import get_csv_config
files = sorted(Path(os.environ["SOMA_CSV_DIR"]).rglob("*.csv"))
assert files, "No CSV outputs"
expected = get_csv_config("t1").csv_header
for path in files:
    with path.open(newline="") as handle:
        header = next(csv.reader(handle))
    assert header == expected, f"Schema/order mismatch: {path}"
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    assert data.shape[0] > 1 and data.shape[1] == len(expected), path
    assert np.isfinite(data).all(), f"NaN/Inf: {path}"
    print(path.name, "frames=", len(data), "dof=", 23)
print("PASS: CSV schema, joint order and finite values")
PY
```

**확인 기준:** 23개 관절의 이름·순서, 유효 frame 수, NaN/Inf 부재. FPS는 BVH Frame Time에서 별도로 확인한다. 이 검사는 kinematic tracking 및 물리적 실행 가능성 판정과 구분한다.

### 3.3 팔·발·연속성 감사

동일 motion의 CSV와 원본 BVH 한 쌍을 지정한다. 아래 FPS120은 실제 원본 FPS와 일치할 때 사용한다.

```bash
SOMA_CSV_ONE=/absolute/path/to/t1_csv_balanced_v1/relative_motion.csv
SOMA_BVH_ONE=/absolute/path/to/small_bvh_subset/relative_motion.bvh

python app/audit_t1_retarget.py \
  --csv "$SOMA_CSV_ONE" --bvh "$SOMA_BVH_ONE" \
  --retarget-config soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json \
  --fps 120 --fk-batch-size 64 \
  --output artifacts/t1_retarget_audit.json
```

**산출물:** `balance`, `continuity`, `upper_body` 지표를 포함한 JSON. 팔꿈치·손 endpoint 오차, 발 방향·stance, 속도 spike를 기준 설정과 비교한다. 근사 COM 지표는 동적 균형 인증을 대신하지 않는다. gain별 변경 배경과 전후 수치는 [원본 대비 변경 및 튜닝 이력](03_upstream_changes_and_tuning.md)을 참조한다.

## 4. 대규모 변환 및 인계

소수 clip 확인 후 JSON의 입력·출력을 전체 corpus용으로 변경한다. 두 독립 터미널에서 같은 config와 입력 목록을 사용해 다음과 같이 분할할 수 있다.

```bash
# 터미널 1
python app/bvh_to_csv_converter.py --config artifacts/t1_local.json \
  --viewer null --device cpu --batch-size 32 --num-shards 2 --shard-index 0
```

```bash
# 터미널 2
python app/bvh_to_csv_converter.py --config artifacts/t1_local.json \
  --viewer null --device cpu --batch-size 32 --num-shards 2 --shard-index 1
```

`batch-size=32`는 retarget 동시 처리량 예시이며, 긴 clip에서 메모리가 부족하면 줄인다. 작업 중 입력 목록·shard 수를 유지하고 완료 후 누락·중복·실패 CSV를 확인한다.

설정과 asset hash를 보관한다.

```bash
sha256sum artifacts/t1_local.json \
  soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json \
  soma_retargeter/configs/t1/soma_to_t1_scaler_config.json \
  soma_retargeter/configs/t1/t1_balanced_feet_stabilizer_config.json \
  "$SOMA_T1_MJCF_PATH" > artifacts/t1_retarget_inputs.sha256
git rev-parse HEAD > artifacts/t1_retarget_commit.txt
```

CSV·BVH 대응표, 실제 FPS/단위, 로그·감사 JSON, 설정·asset 식별자를 함께 인계한다. 이후 [WBC T1 실행 절차](https://github.com/MFIWO/GR00T-WholeBodyControl/blob/t1-transfer-dev/docs/t1/00_execution_runbook.md)에서 robot PKL 변환과 학습을 진행한다. 추가 품질 기준은 [검증·인계 문서](02_validation_handoff.md)를 따른다.
