# Local run viewer

Open `../run_viewer.html` directly in a browser (double-click, or open with Chrome).
No server, ROS runtime, network connection, CDN, or Python installation is needed
for viewing the exported HTML. It includes the selected run's records and JPEGs.

- Select a call in the timeline or drag the slider.
- Switch between the observation used for the decision and the saved observation
  after execution. Missing observations are explicitly shown as missing; an older
  frame is never substituted for a missing post-action image.
- Click any camera to enlarge it. Escape closes the enlarged image.
- Play uses the intervals between recorded call timestamps, divided by the chosen
  speed. Left/right arrows change calls; Space toggles playback.
- **Open local folder** loads another single run directory containing `run.json` through
  the browser's local folder picker. Re-select it to load newer records. Folder
  loading does not modify the HTML or upload files; export again to make it portable.

To export another run, from the `ai_worker` Git repository root (use an existing run ID):

```bash
python3 poc_codex/export_viewer.py \
  poc_codex/runs/20260928_003947 \
  --output poc_codex/run_viewer.html
```

The exporter uses only Python's standard library. The page template is
`template.html`; regenerate the exported HTML after editing it.

## Follow an active robot run over SSH

Open the viewer through the robot's Live Server URL (for example, forwarded
`http://localhost:5500/poc_codex/run_viewer.html`). Enter a **Run folder** name,
such as `20260928_003947`, and click **Follow run**. The page reads
`poc_codex/runs/<run_id>/` using the same Live Server connection. No new
server or ROS node is needed for run records. The run folder must be under
`poc_codex/runs` next to `run_viewer.html`, and the browser must be able to
fetch its `run.json` through that URL.

The page checks for new call requests, public history, observations, and
results about every 1.2 seconds. The HTML intentionally uses optional closing
tags to prevent Live Server from reloading the page whenever a run file changes. A pending call shows its recorded action
reason and the observation it used; when execution completes, the result and
new observation appear. The three small **Model observation** images beneath
the rationale show the selected call's newest saved snapshot even while the
three large camera panels show the live ROS stream. These views are not
time-synchronized.

**Following latest** moves to new calls automatically. Selecting an older
call pauses automatic movement while updates continue; **Resume latest**
returns to the newest call. **Stop following** ends the file checks.

The browser's **Open local folder** picker opens folders on the browser
computer. With SSH port forwarding it cannot browse the robot's filesystem,
so use **Run folder** to follow a robot run. Its existing offline replay
behavior remains available.

## Live camera mode (Cyclo)

Open **Live cameras**, enter the existing Cyclo video bridge URL (default
`http://localhost:7085`), confirm the three topics, and click **Connect / Reconnect**.
Use the Cyclo PC IP instead of localhost when it runs on another computer.
The real-robot and Gazebo topic presets match this project's configuration files.
If you change those files, also update the fields in the viewer.

- Raw Image: `/stream?quality=50&type=mjpeg&default_transport=raw&topic=/...`
- CompressedImage: the same `type=ros_compressed&default_transport=compressed`
  path used by Cyclo's `ImageGridCell.js`. A trailing `/compressed` is removed
  from the base topic automatically. The compressed publisher must actually exist.

Live cameras can be used from a directly opened HTML file; follow mode still needs HTTP access to run records. The existing
ROS `web_video_server` bridge must be running and able to receive the camera
ROS topics. Cyclo's orchestrator bringup starts it on port 7085 by default. If
Cyclo is not running, the equivalent camera-only bridge can be started from the
correct sourced ROS environment (with `web_video_server` installed):

```bash
ros2 run web_video_server web_video_server --ros-args -p port:=7085
```

Click a live camera to expand it without opening another streaming connection.
**Disconnect**, or either recorded-observation button, stops live streams and
returns to snapshots. Changing timeline calls does not reconnect live streams.
Browser local-network permission may be required. Endpoint availability and ROS
camera reception depend on the local ROS setup.

Live camera mode only changes the three camera panels. The decision and command
cards still describe the selected saved run call, explicitly labeled in the UI.
MJPEG has no per-frame timestamp here: a decoded frame is not proof the camera
is still fresh; the page does not invent FPS, latency, or stale-frame detection.

## Data semantics

The default replay mode displays saved observation snapshots. Live mode connects
to the explicitly configured camera bridge.
`arguments.reason` is the recorded action explanation, displayed verbatim.
It does not reconstruct internal model reasoning. Targets and arrival measurements
are shown separately; gripper values use this project's 0=open / 1=closed convention.
The call counter includes observations and commands, and is not a task completion
percentage. The footer describes the whole run, while cards describe the selected
call. ROS timestamps label images; the player uses wall-clock record timestamps.

Data is limited to run/result summaries, rollout JSON, call requests, public history,
and public observation JSON/JPEGs. Session transcripts and prompts are not included.

A local `file://` page cannot subscribe to ROS topics or silently monitor arbitrary
files. This version supports offline replay, explicit folder reloads, live video
through an existing Cyclo-compatible camera bridge, and follow mode through
the existing Live Server HTTP connection.
