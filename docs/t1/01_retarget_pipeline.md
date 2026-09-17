# SOMA BVH → T1 CSV

작성 기준: 2026-09-17 · `MFIWO/soma-retargeter` / `t1-retarget-support` / `93cded49f8810adc181b1150f65182bb13727cfe`.

이 문서는 해당 commit의 코드·설정·기존 문서를 대조한 안내서다. 문서 정리 과정에서 PPO, 데이터 변환, GPU 평가 또는 실기 제어를 실행하지 않았다. 아래 명령은 데이터·checkpoint·환경이 준비된 작업용 머신에서 사용하는 템플릿이다. 실행 경로가 존재한다는 것과 학습 성능이 검증됐다는 것은 구분한다.

## 1. 환경과 asset

```bash
git clone --branch t1-retarget-support --single-branch \
  https://github.com/MFIWO/soma-retargeter.git soma-retargeter-t1
cd soma-retargeter-t1
git lfs install
git lfs pull
conda create -n soma-retargeter python=3.12 -y
conda activate soma-retargeter
python -m pip install -e .
```

이미 준비된 환경은 재생성할 필요가 없다. 이 브랜치 `pyproject.toml`은 Python ≥3.12, Newton 1.0.0, warp-lang 1.12.0 등을 고정한다. 원래 [설치 안내](../../README.md)와 [의존성](../../pyproject.toml)을 함께 확인한다. LFS pointer만 받아 실제 로봇 asset이 비어 있지 않은지 확인한다.

## 2. 입력과 출력의 로컬 설정

BONES-SEED의 SOMA BVH와 데이터 사용 조건을 준비한다. checked-in JSON에는 작성자의 절대 경로가 들어 있으므로 로컬 복사본으로 바꾼다. 실행 위치는 repository root다.

```bash
mkdir -p artifacts
cp assets/t1_balanced_bvh_to_csv_converter_config.json artifacts/t1_local.json
```

복사한 JSON을 다음 구조로 편집한다. `/absolute/path/...`는 실제 존재하는 경로로 바꾼다. converter는 import/export를 JSON에서 읽는다.

```json
{
  "import_folder": "/absolute/path/to/soma_uniform/bvh",
  "export_folder": "/absolute/path/to/t1_csv",
  "batch_size": 4,
  "retargeter": "Newton",
  "retarget_source": "soma",
  "retarget_target": "t1",
  "retarget_source_facing_direction": "Mujoco",
  "retargeter_config": "soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json"
}
```

## 3. 소수 clip부터 변환

```bash
python app/bvh_to_csv_converter.py \
  --config artifacts/t1_local.json --viewer null --device cpu
```

`--viewer gl`로 source/robot 포즈를 확인할 수 있다. CPU 예시는 기능 확인용이며 대규모 throughput 보장은 아니다. `batch_size`는 retarget batch 크기이고 PPO의 env 수가 아니다. 출력은 입력 상대 디렉터리 구조에 대응하는 CSV다.


## T1 balanced에서 추가한 조정

`soma_to_t1_balanced_retargeter_config.json`은 pelvis-relative foot heading, virtual toe, stance-foot flattening, 로봇 팔 길이 기준 hand/elbow target, elbow-plane/branch 연속성, 관절별 선호 자세, offline temporal smoothing을 구성한다. 코드 기준 IK 50회, joint-limit weight 40, smooth weight 35, collision weight 0이다. 이 수치는 시작 preset이지 모든 motion의 최적값이 아니다.

단순 G1 관절 이름 치환으로 T1 팔·발 reference가 만들어지지 않는다. T1 root/body `Trunk`, `Waist`, hand/foot link와 짧은 팔의 도달 범위를 확인한다. zero-phase smoothing은 offline 전처리이고 실시간 VR 필터와 동일하지 않다.

## 큰 corpus와 deterministic shard

```bash
# 두 터미널에서 동일한 입력 목록과 config로 각각 실행한다.
python app/bvh_to_csv_converter.py --config artifacts/t1_local.json \
  --viewer null --device cpu --batch-size 32 --num-shards 2 --shard-index 0
python app/bvh_to_csv_converter.py --config artifacts/t1_local.json \
  --viewer null --device cpu --batch-size 32 --num-shards 2 --shard-index 1
```

전체 BVH 상대 경로를 정렬한 뒤 `[shard_index::num_shards]`로 나누고, 그 뒤 기존 CSV 존재 여부를 검사한다. 입력 목록·shard 수를 작업 중 바꾸지 않는다. 현재 CSV는 파일이 존재하면 건너뛰므로 retarget 설정 변경 시 새 출력 폴더가 필요하다. 존재 여부가 완료·품질·동일 설정의 증명은 아니므로 중단된 CSV를 별도로 검사한다.


## 4. 다음 단계

[CSV 검증과 학습 인계](02_validation_handoff.md)를 통과한 자료만 학습 manifest에 포함한다. 이 단계에서는 RL 학습이나 실기 명령을 실행하지 않는다.

## 구현 근거

- [app/bvh_to_csv_converter.py](https://github.com/MFIWO/soma-retargeter/blob/93cded49f8810adc181b1150f65182bb13727cfe/app/bvh_to_csv_converter.py)
- [soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json](https://github.com/MFIWO/soma-retargeter/blob/93cded49f8810adc181b1150f65182bb13727cfe/soma_retargeter/configs/t1/soma_to_t1_balanced_retargeter_config.json)
- [soma_retargeter/pipelines/newton_pipeline.py](https://github.com/MFIWO/soma-retargeter/blob/93cded49f8810adc181b1150f65182bb13727cfe/soma_retargeter/pipelines/newton_pipeline.py)
