# ffw_vision

AI Worker 카메라 영상에서 **픽셀 관측**(원본 영상 기준 u, v와 그 영상의 header)을 만들어 발행하는 패키지다.
3차원 좌표 계산, 파지, 모션은 이 패키지의 범위가 아니다. 그 작업은 후속 좌표계 노드가 맡는다.

- `ffw_vision_msgs`: 관측 메시지 정의 (`PixelObservation`)
- `ffw_vision`: Vision 노드. 현재는 패키지 골격과 카메라 설정 틀만 있다.

## 관측 메시지 `ffw_vision_msgs/msg/PixelObservation`

| 필드 | 의미 |
|---|---|
| `header` | 사용한 원본 영상의 header 를 그대로 복사한다. `stamp` 는 촬영 시각, `frame_id` 는 원본 영상의 광학 좌표계다. 수신·발행 시각으로 바꾸지 않는다. |
| `camera_id` | `head`, `left_wrist`, `right_wrist` 중 하나 (설정 `cameras.yaml` 의 키) |
| `image_topic` | 좌표의 기준이 된 영상 토픽. raw 인지 rectified 인지를 구분하는 데 쓴다. |
| `image_width`, `image_height` | 원본 영상 크기(픽셀) |
| `target_label` | 대상 종류 또는 요청된 대상. 수동 선택에서는 비어 있을 수 있다. |
| `point_type` | (u, v) 의 의미. 수동 선택은 `selected_point` 다. |
| `source_mode` | `SOURCE_MANUAL`(1) 수동 선택, `SOURCE_AUTO`(2) 자동 검출 |
| `status` | `STATUS_VALID`(1), `STATUS_NOT_DETECTED`(2), `STATUS_AMBIGUOUS`(3) |
| `u`, `v` | 원본 영상 기준 픽셀 좌표 |

### 좌표 규칙

- u 는 열(오른쪽이 +), v 는 행(아래쪽이 +)이다.
- 정수 (u, v) 는 그 픽셀의 **중심**이다. `sensor_msgs/CameraInfo` 의 K 행렬과 같은 기준이다.
  유효 범위는 `-0.5 <= u < image_width - 0.5`, `-0.5 <= v < image_height - 0.5` 이다.
- 화면에서 확대·축소·잘라내기를 했더라도 발행하는 값은 항상 원본 영상 좌표로 환산한 값이다.

### 수신 측 사용 규칙

1. **`status` 를 먼저 확인한다.** `STATUS_VALID` 일 때만 u, v 를 사용한다.
2. `STATUS_VALID` 가 아니면 u, v 는 NaN 이다. 미검출을 (0, 0) 으로 표현하지 않는다.
3. `status` 또는 `source_mode` 가 0(`UNSET`)이면 값을 채우지 않은 메시지이므로 버린다.
4. 3차원 변환에는 `header.stamp` 시각의 TF 와, `header.frame_id`·`image_topic`·크기가 일치하는 camera_info 를 쓴다.
5. `point_type == "selected_point"` 는 영상에서 고른 지점일 뿐이다. 물체 중심이나 파지점으로 해석하지 않는다.
6. 이 지점의 depth 는 보이는 표면까지의 거리다. 물체 중심이나 파지점이 필요하면 대상 형상에 맞는 처리를 따로 한다.

### 노드 상태

입력 끊김 같은 노드 상태는 관측 메시지에 넣지 않는다. 별도 토픽으로 발행한다(4단계에서 정의).

### 확장 예정 (5단계)

회전, 검출 점수, 다중 검출(배열 메시지)은 검출기와 실제 출력이 정해진 뒤
의미와 유효성 표현(값이 있는지 나타내는 표시)을 정의하고 추가한다.

## 카메라 설정 `config/cameras.yaml`

카메라마다 영상 토픽, raw/rectified 구분, camera_info 토픽, 정렬된 depth 토픽, frame_id 를 적는다.
빈 문자열은 "아직 확인하지 않음"을 뜻한다. 로봇 출력으로 확인한 값만 채운다.

### head (ZED Mini, 2026-10-23 로봇 출력으로 확인)

| 항목 | 값 |
|---|---|
| 원본 영상 | `/zed/zed_node/left/image_rect_color` (왼쪽 rectified, 672×376, bgra8) |
| camera_info | `/zed/zed_node/left/camera_info` (plumb_bob, D = 0) |
| frame_id | `zed_left_camera_optical_frame` |
| depth | `/zed/zed_node/depth/depth_registered` (32FC1, 미터). 아래 depth 설정으로 실행했을 때만 있음 |

- raw 영상과 rectified 영상의 frame_id 가 같다. 스트림 구분은 `image_topic` 으로 한다.
- 좌표계 노드는 rectified 영상에 맞는 camera_info 의 투영 파라미터(P, D = 0)를 쓴다. raw camera_info(equidistant 왜곡)를 섞어 쓰지 않는다.
- depth 를 연결할 때는 영상 정합, 촬영 시각, 단위를 확인한 뒤 사용한다.

### ZED depth 설정 `config/zed_depth_override.yaml`

bringup 의 ZED 설정(`depth_mode: NONE`)은 바꾸지 않는다. 카메라만 따로 실행할 때 이 파일로 덮어쓴다.

```bash
ros2 launch ffw_bringup camera_zed.launch.py camera_model:=zedm \
  ros_params_override_path:=$(ros2 pkg prefix ffw_vision)/share/ffw_vision/config/zed_depth_override.yaml
```

- `depth_mode: ULTRA`. NEURAL 계열은 이 컨테이너에 TensorRT 가 없어서 쓸 수 없다.
- `depth_stabilization: 0` 이라 위치 추적이 켜지지 않는다.

## 입력 확인 노드 `input_monitor`

설정한 카메라의 영상, camera_info, depth(설정된 경우)를 구독해서 주기적으로 한 줄씩 로그를 남긴다. 아무것도 발행하지 않는다.
영상을 받은 뒤 100 ms 가 지나면, 같은 stamp 의 camera_info 와 depth 가 있는지 판정한다.

```bash
ros2 run ffw_vision input_monitor --ros-args \
  --params-file $(ros2 pkg prefix ffw_vision)/share/ffw_vision/config/cameras.yaml
```

| 상태 | 의미 |
|---|---|
| `OK` | 영상이 들어오고 frame_id, 크기, camera_info 짝이 모두 맞음 |
| `NO_INPUT` | `timeout_sec` 동안 영상이 없음 |
| `NO_CAMERA_INFO` | 영상은 오지만 이번 구간에 camera_info 가 없음 |
| `NO_DEPTH` | depth_topic 이 설정됐지만 이번 구간에 depth 가 없음 |
| `MISMATCH` | frame_id·크기·depth 인코딩이 다르거나, 같은 stamp 의 camera_info·depth 가 없는 영상이 있음 |

## depth 확인 도구 `depth_probe`

영상과 depth 를 stamp 가 같은 것끼리 `frames` 개 모은 뒤, 영역 `roi` 의 유효 depth 비율과 거리 통계를 출력하고 PNG 를 저장한 다음 종료한다.

```bash
ros2 run ffw_vision depth_probe --ros-args \
  --params-file $(ros2 pkg prefix ffw_vision)/share/ffw_vision/config/cameras.yaml \
  -p roi:="[0, 0, 0, 0]" -p frames:=30
```

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `roi` | `[0, 0, 0, 0]` | `[u_min, v_min, u_max, v_max]` 원본 픽셀, max 는 포함 안 함. 모두 0 이면 전체 영상 |
| `frames` | 30 | 모을 짝 개수 |
| `min_depth`, `max_depth` | 0.1, 10.0 | 유효 범위(m). zedm.yaml 과 같음 |
| `output_dir` | `/workspace/vision_ws/captures` | PNG 저장 위치 |
| `timeout_sec` | 10.0 | 이 시간 안에 다 모으지 못하면 실패로 종료 |

- 유효 depth: 유한하고 `min_depth` 이상 `max_depth` 이하인 값.
- 출력: `valid_ratio`(전체), 프레임별 최소·최대 비율, 중앙값, p10, p90, `median_std`(프레임별 중앙값의 흔들림).
- PNG: `<stamp>_<camera>_color.png`(컬러), `_depth.png`(depth 색 표시, 검정 = 무효), `_overlay.png`(둘을 반씩 겹친 것).
  모두 50 px 격자와 좌표 숫자, roi 사각형이 그려져 있다.

## 빌드 (ai_worker 컨테이너 안)

`./docker/container.sh enter` 로 컨테이너에 들어간 뒤 실행한다.
기존 underlay(`/root/ros2_ws/install`)는 컨테이너 bashrc 가 자동으로 source 한다.

```bash
cd /workspace/vision_ws
colcon build --symlink-install --base-paths \
  /root/ros2_ws/src/ai_worker/ffw_vision_msgs \
  /root/ros2_ws/src/ai_worker/ffw_vision
source /workspace/vision_ws/install/setup.bash
```

- 빌드 결과는 `/workspace/vision_ws/{build,install,log}` 에만 생긴다.
- 컨테이너 안에서 `cb`(`/root/ros2_ws` 전체 빌드)는 쓰지 않는다. 이 패키지들이 underlay 에 섞인다.
