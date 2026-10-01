# poc_codex 시스템 구조도 초안

[구조도 열기](architecture-draft.html) · [편집용 JSON](architecture-draft.json) · [이전 한국어 PNG](architecture-draft.visual-check.2048x1320.light.png)

2026-10-01 로컬 소스의 실기 구성을 기준으로 작성했다. 첨부 예시의 사용자 → PC 실행 환경 → AI Worker 구도를 따라 구성요소와 ROS 연결을 정리했다. 경계는 논리적 역할을 뜻하며 PC, 컨테이너, 로봇 내 실제 프로세스 배치는 확정하지 않는다. Codex app-server는 로컬 자식 프로세스이며, 이 블록은 모델 판단 역할까지 추상화한다. 모델 서비스의 네트워크 배치는 이 그림의 범위 밖이다.

핵심 흐름은 `사용자 → Host → RobotRollout → ObservationGuard / RobotTools → RobotIO → Cyclo / 로봇 제어`다. Codex app-server와 Host는 JSON-RPC/stdio로 도구 호출과 응답을 주고받는다. 그림의 화살표는 호출·명령의 주요 방향이며, 실행 결과와 관측은 호출 경로의 역방향으로 반환된다. 카메라와 상태의 수신 경로는 별도 화살표로 표시했다.

## 블록과 코드 근거

| 블록 | 근거 | 역할 |
|---|---|---|
| 진입점 | [run_codex.py](../run_codex.py), [workspace.py](../workspace.py) | 설정과 작업 지시 로드, 실행 폴더·모델 문서 준비, ROS 준비 대기 |
| Host / app-server | [codex_host.py](../codex_host.py), [transport.py](../transport.py) | app-server 자식 프로세스, dynamic tools, 한 번에 하나의 도구 처리 |
| RobotRollout | [rollout.py](../rollout.py), [plan_executor.py](../plan_executor.py) | 도구 라우팅, 행동 예산, 관측 저장·응답, 최대 5단계 순차 계획 |
| 관측 검증 / RobotTools | [observation_guard.py](../observation_guard.py), [robot_tools.py](../robot_tools.py) | request_id, 관측 신선도·상태 변화, 이동·관절 범위 검사, 실행 후 관측 |
| RobotIO | [robot_io.py](../robot_io.py), [transforms.py](../transforms.py), [stall.py](../stall.py) | ROS 발행·구독, 좌표 변환, 시간 정렬 관측, 도착·정지 판정 |
| Cyclo MoveL | [robot_io.py](../robot_io.py)의 publisher 정의와 명령 함수 | 외부 의존성. MoveL과 그리퍼 궤적을 받아 최종 팔 궤적을 생성한다는 호출 측 계약 |
| controller manager | [SG2 controller 설정](../../ffw_bringup/config/ffw_sg2_rev1_follower/ffw_sg2_follower_ai_hardware_controller.yaml) | update_rate 100 Hz, arm_l_controller, arm_r_controller, head_controller, lift_controller |
| 실행 기록 / Viewer | [run_codex.py](../run_codex.py), [rollout.py](../rollout.py), [export_viewer.py](../export_viewer.py) | result.json, public/observations, rollout, HTML 내보내기 |

실행 기록 블록은 Host가 운영하는 에피소드의 전체 산출물을 묶은 것이다. 관측·행동 기록의 실제 작성자는 RobotRollout이며, result.json은 진입점에서 작성한다.

## ROS 인터페이스

정확한 이름은 [config.yaml](../config.yaml)과 [robot_io.py](../robot_io.py)를 기준으로 한다.

| 방향 | 데이터 | 토픽 |
|---|---|---|
| 카메라 → RobotIO | 머리 RGB | `/zed/zed_node/rgb/image_rect_color` |
| 카메라 → RobotIO | 왼손목 RGB | `/camera_left/camera_left/color/image_rect_raw` |
| 카메라 → RobotIO | 오른손목 RGB | `/camera_right/camera_right/color/image_rect_raw` |
| 로봇 → RobotIO | 관절 상태 / TF | `/joint_states`, `/tf`, `/tf_static` |
| RobotIO → Cyclo | MoveL | `/l_goal_move`, `/r_goal_move` |
| RobotIO → Cyclo | 그리퍼 JointTrajectory | `/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory`, `/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory` |
| RobotIO → 외부 제어 경로 | head JointTrajectory | `/leader/joystick_controller_left/joint_trajectory` |
| RobotIO → 외부 제어 경로 | lift JointTrajectory | `/leader/joystick_controller_right/joint_trajectory` |

Cyclo 이후 팔 궤적 전달 방식과 head/lift 중계 노드는 이 프로젝트 내부만으로 확정하지 않았다. 로봇 controller manager까지의 화살표는 중간 전달 경로를 축약한 것이다. `FollowJointTrajectory`라고 단정하지 않았다. 참조 이미지의 cuMotion/Nvblox/Depth Image 경로는 poc_codex의 확인된 직접 인터페이스에 포함하지 않았다.

`100 Hz`는 SG2 controller manager 설정이며 모델 판단이나 poc_codex 루프 주기가 아니다. 기본 READ 화면은 간단한 설명을 표시하고, 상세 정보에서 각 블록의 태그를 확인할 수 있다.

## 실기와 Gazebo

| 항목 | 실기 | Gazebo |
|---|---|---|
| 진입점 | run_codex.py | run_codex_gazebo.py |
| 설정 | config.yaml | config_gazebo.yaml |
| I/O | robot_io.py | robot_io_gazebo.py |
| 관측 검증 | observation_guard.py | observation_guard_gazebo.py |
| 모델 문서 | agent_template/ | agent_template_gazebo/ |
| 시계 | system timestamp | /clock 필요 |

## 열기와 수정

HTML은 단독으로 브라우저에서 열 수 있다. 구조도의 제목, 설명, 라벨, 고정 메뉴와 HTML lang은 모두 영어다. Export 메뉴에서 이미지 또는 SVG 내보내기를 제공한다. 수정은 JSON 원본에서 하고 Archify의 validate와 deliver를 다시 실행한다.

## 검증 기록

영문 버전은 Archify의 validate, deliver, check, browser-check를 통과했다. 최신 결과는 [영문 최종 검증 기록](architecture-english-review/architecture-draft.finalize-summary.json)에 있다. 기존 visual-check PNG는 이전 한국어 버전의 캡처다.

- diagram_type: architecture
- specification_sha256: 55d008f252011ce2de841a820a00b5221e157ad71249f0372ef2b521e9b6aed0
- artifact_sha256: c6619b68140a5320520b1a0a0a098adad8eb3cdec9722c27863dc6bcfdcd2535
- browser_evidence: passed
- visual_review: not_requested (영문 모듈 구조도)

새로운 예시 스타일의 영문 도식은 [Agent Control Loop](control-loop-draft.html)와 [PNG](control-loop-draft.png)를 참고한다.
