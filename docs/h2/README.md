# SOMA → H2 retarget 안내

작성 기준: 2026-09-17 · `MFIWO/soma-retargeter` / `h2-retarget-support` / `b2d7ce7a5584c2bd290042baed83cf0c69256873`.

이 문서는 해당 commit의 코드·설정·기존 문서를 대조한 안내서다. 문서 정리 과정에서 PPO, 데이터 변환, GPU 평가 또는 실기 제어를 실행하지 않았다. 아래 명령은 데이터·checkpoint·환경이 준비된 작업용 머신에서 사용하는 템플릿이다. 실행 경로가 존재한다는 것과 학습 성능이 검증됐다는 것은 구분한다.

| 단계 | 문서 |
|---|---|
| 설치·입력·로봇 설정·CSV 생성 | [retarget pipeline](01_retarget_pipeline.md) |
| CSV 품질·시간/좌표 계약·학습 인계 | [검증과 training handoff](02_validation_handoff.md) |
| NVIDIA 원본 대비 변경·팔/손/발목 튜닝 | [변경과 시행착오](03_upstream_changes_and_tuning.md) |

이 브랜치는 `h2-retarget-support`다. `main` 또는 다른 로봇 브랜치와 파일 존재/기능이 다를 수 있다. H2 브랜치에도 초기 T1 등록이 포함되지만 최신 T1 balanced/sharding 구현은 T1 브랜치를 사용한다.

retarget는 사람 모션을 로봇의 기하학적 reference로 만드는 단계다. 이 저장소에서 SONIC PPO를 학습하는 것은 아니다. 학습·G1 weight transfer·continual/evaluation은 [GR00T WBC의 H2 문서](https://github.com/MFIWO/GR00T-WholeBodyControl/blob/h2-transfer-dev/docs/h2/README.md)에서 이어진다. 해당 저장소 접근 권한이 필요할 수 있다.

공개 retarget 문서는 이 저장소의 구현·설정과 일반적인 인계 계약만 기록한다. 학습 서버의 경로, 비공개 checkpoint와 개별 실험 결과는 학습 저장소의 문서에서 관리한다.
