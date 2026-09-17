# SOMA → T1 retarget 안내

작성 기준: 2026-09-17 · `MFIWO/soma-retargeter` / `t1-retarget-support` / `93cded49f8810adc181b1150f65182bb13727cfe`.

이 문서는 해당 commit의 코드·설정·기존 문서를 대조한 안내서다. 문서 정리 과정에서 PPO, 데이터 변환, GPU 평가 또는 실기 제어를 실행하지 않았다. 아래 명령은 데이터·checkpoint·환경이 준비된 작업용 머신에서 사용하는 템플릿이다. 실행 경로가 존재한다는 것과 학습 성능이 검증됐다는 것은 구분한다.

| 단계 | 문서 |
|---|---|
| 설치·입력·로봇 설정·CSV 생성 | [retarget pipeline](01_retarget_pipeline.md) |
| CSV 품질·시간/좌표 계약·학습 인계 | [검증과 training handoff](02_validation_handoff.md) |

이 브랜치는 `t1-retarget-support`다. `main` 또는 다른 로봇 브랜치와 파일 존재/기능이 다를 수 있다. T1 balanced·deterministic process sharding은 이 T1 브랜치의 기능이다.

retarget는 사람 모션을 로봇의 기하학적 reference로 만드는 단계다. 이 저장소에서 SONIC PPO를 학습하는 것은 아니다. 학습·G1 weight transfer·continual/evaluation은 [GR00T WBC의 T1 문서](https://github.com/MFIWO/GR00T-WholeBodyControl/blob/t1-transfer-dev/docs/t1/README.md)에서 이어진다. 해당 저장소 접근 권한이 필요할 수 있다.

공개 retarget 문서는 이 저장소의 구현·설정과 일반적인 인계 계약만 기록한다. 학습 서버의 경로, 비공개 checkpoint와 개별 실험 결과는 학습 저장소의 문서에서 관리한다.
