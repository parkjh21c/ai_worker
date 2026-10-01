# poc_codex

카메라 이미지와 로봇 상태를 관측하고, 모델이 선택한 도구를 통해 SG2 로봇을 제어하는 하네스다. 아래 내용은 2026-10-01의 로컬 소스와 설정을 기준으로 확인했다.

## 실행 환경 선택

ROS 환경을 불러온 `ai_worker` 컨테이너에서 실행한다. 로봇 bringup과 Cyclo MoveL 컨트롤러가 먼저 실행되어 있어야 한다.

| 환경 | 진입점 | 기본 설정 | 관측 계층 | 모델 문서 원본 |
|---|---|---|---|---|
| 실기 | `run_codex.py` | `config.yaml` | `robot_io.py`, `observation_guard.py` | `agent_template/` |
| Gazebo | `run_codex_gazebo.py` | `config_gazebo.yaml` | `robot_io_gazebo.py`, `observation_guard_gazebo.py` | `agent_template_gazebo/` |

```bash
cd /root/ros2_ws/src/ai_worker/poc_codex

# 실기
python3 -u run_codex.py --task "pick up the blue block"

# Gazebo에서 실행할 때는 위 명령 대신 사용
python3 -u run_codex_gazebo.py --task "pick up the blue block"
```

`--config`는 설정 파일을 바꾸며, 실기/Gazebo 구현 선택은 진입점이 결정한다. 두 진입점 모두 `--run-id`로 실행 폴더 이름을 지정할 수 있고, 생략하면 UTC 시각을 쓴다. 이미 사용한 실행 폴더는 재사용하지 않는다. 한 번에 에피소드 하나를 실행한다.

실기는 카메라 3개, 양쪽 그리퍼·head·lift 관절 상태, `base_link`에서 양손까지의 TF를 최대 30초 기다린다. `/clock`은 필요하지 않다. Gazebo는 시뮬레이션 시계 `/clock`도 필요하며 기본 준비 대기는 10초다. 시간은 각 설정의 `io.ready_timeout_s`에서 정한다.

## 관측과 명령 순서

```text
robot_start → 관측 O1 + request_id R1
모델 판단 → 명령(목표, reason, R1)
하네스의 사전 검증 → 명령 발행 → 완료 판정
사후 관측 수집 → 실행 결과 + 관측 O2 + request_id R2 반환
모델이 O2를 보고 다음 명령 결정
```

동작 후 관측은 명령 도구 안에서 자동으로 수집한다. 정상적으로 반환되면 `robot_observe`를 별도로 부를 필요가 없다. `reason`은 실행 전에 모델이 제출한 관측 근거와 동작 목적이며, 응답 패킷의 `reason`은 하네스의 결과 설명이다.

로봇 도구는 한 번에 하나씩 처리한다. 양팔 동시 실행 도구는 아직 없다. `action_executed`는 명령 발행 여부이며 실제 변위나 작업 성공을 뜻하지 않는다. 사후 관측 실패 시 `observation`은 null이고 다시 관측해야 한다. timeout이나 실행 결과가 불확실한 오류는 에피소드를 종료하지만 이미 보낸 로봇 명령을 취소하지 않는다.

동작 범위, lift와 EE의 관계, 결과 상태의 의미는 [실기 계약](agent_template/context/robot_contract.md)과 [Gazebo 계약](agent_template_gazebo/context/robot_contract.md)을 참고한다.

## 설정과 계약 문서 검사

이 디렉터리에서 실행한다. Python과 PyYAML이 필요하며 ROS나 로봇 연결은 필요 없다.

```bash
python3 check_contract_config.py
```

실기 `config.yaml`의 수치와 `agent_template/context/robot_contract.md`의 17개 문구를 비교한다. 그리퍼 궤적 시간은 `robot_io.py`의 명령·대기 함수 기본값을 AST로 읽어 대조한다. 불일치는 종료 코드 1로 보고한다. Gazebo 계약과 동작의 의미까지 검사하지는 않으며, 자동 실행에 연결되어 있지 않아 설정·문서 변경 후 직접 실행해야 한다.

## 결과와 뷰어

`runs/<run_id>/result.json`에서 종료 이유와 모델의 완료 근거를 확인한다. `task_complete`는 모델의 완료 선언이며, 사람이 사진과 상태를 확인해 `verdict`와 `verdict_note`를 기입한다. `public/observations/`는 저장된 관측, `rollout/`은 상세 행동 기록이다.

```bash
python3 export_viewer.py runs/<run_id> --output run_viewer.html
```

내보낸 HTML은 브라우저에서 열 수 있다. 실행 추적과 포트 7085의 라이브 카메라는 [뷰어 안내](viewer/README.md)를 참고한다. 라이브 영상과 모델이 사용한 저장 관측은 서로 다른 시점일 수 있다.

Ctrl+C는 새 행동을 막고 진행 중인 도구의 처리가 끝나기를 기다린다. 도착 대기 외에 관측 수집과 프로세스 정리 시간도 들므로 전체 종료 시간이 15초 이내라고 보장되지는 않는다.
