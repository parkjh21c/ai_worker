// Uses Cyclo's existing web_video_server. No page server or third-party scripts.
(() => {
  const cameras = ['head', 'wrist_left', 'wrist_right'];
  const fields = ['topicHead', 'topicLeft', 'topicRight'];
  const presets = {
    robot: ['/zed/zed_node/rgb/image_rect_color', '/camera_left/camera_left/color/image_rect_raw', '/camera_right/camera_right/color/image_rect_raw'],
    gazebo: ['/head_camera/image', '/wrist_left_camera/image', '/wrist_right_camera/image'],
  };
  const snapshotNotice = $('viewNotice').textContent;
  let streams = [], monitor = null;

  function buildStreamUrl(base, topic, transport) {
    const url = new URL(base);
    if (!['http:', 'https:'].includes(url.protocol) || url.username || url.password || url.search || url.hash)
      throw new Error('영상 브리지 주소는 http://PC주소:7085 형태로 입력하세요.');
    topic = topic.trim();
    if (!/^\/[A-Za-z0-9_/]+$/.test(topic))
      throw new Error('카메라 토픽은 /로 시작하는 ROS 토픽 이름이어야 합니다.');
    if (transport === 'compressed' && topic.endsWith('/compressed')) topic = topic.slice(0, -11);
    // Cyclo's server expects literal slashes in the topic parameter.
    const type = transport === 'compressed' ? 'ros_compressed' : 'mjpeg';
    return `${url.href.replace(/\/+$/, '')}/stream?quality=50&type=${type}&default_transport=${transport}&topic=${topic}`;
  }
  // Exposed for a small, dependency-free URL contract check.
  window.buildCameraStreamUrl = buildStreamUrl;

  window.updateLiveLabels = () => {
    text('observationLabel', '카메라 영상');
    text('observation', 'LIVE SOURCE · 하단 판단은 선택한 기록 기준');
    text('viewNotice', '상단은 현재 카메라 스트림, 하단 판단·명령은 선택한 run의 저장 기록입니다. 서로 같은 시점이 아닙니다. 영상 스트림은 프레임 시각을 제공하지 않아 최신 프레임 여부를 판정하지 않습니다.');
    text('connectionMode', 'LOCAL · LIVE CAMERA');
  };

  function disconnect() {
    clearInterval(monitor); monitor = null;
    for (const s of streams) {
      s.img.onload = null; s.img.onerror = null;
      s.img.src = '';
      // Replace a streaming IMG to cancel its open multipart HTTP request.
      const replacement = document.createElement('img');
      replacement.alt = s.img.alt;
      s.img.replaceWith(replacement);
      s.el.classList.remove('expanded');
      s.empty.textContent = '저장된 이미지 없음';
    }
    streams = [];
    $('disconnectLive').disabled = true;
    $('liveView').disabled = true;
    $('liveView').classList.remove('active');
    $('liveView').setAttribute('aria-pressed', 'false');
    text('connectionMode', 'LOCAL · OFFLINE');
    text('observationLabel', '표시 중인 관측');
    text('viewNotice', snapshotNotice);
    text('liveStatus', '연결 전');
    if (state.view === 'live') state.view = 'before';
  }
  window.disconnectLiveCameras = disconnect;

  function refreshStatus() {
    let seen = 0, failed = 0;
    for (const s of streams) {
      // MJPEG load events vary by browser; naturalWidth also detects decoded data.
      if (!s.failed && s.img.naturalWidth > 0) {
        s.seen = true; s.empty.hidden = true;
      }
      if (s.seen && !s.failed) seen++;
      if (s.failed) failed++;
      s.stamp.textContent = s.failed ? 'STREAM ERROR' : s.seen ? 'LIVE SOURCE · 프레임 수신됨' : 'LIVE SOURCE · 영상 대기';
    }
    text('liveStatus', `프레임 수신 ${seen}/3${failed ? ` · 연결 오류 ${failed}` : ''}`);
  }

  function connect(event) {
    event?.preventDefault();
    let urls;
    try {
      urls = fields.map(id => buildStreamUrl($('streamBase').value.trim(), $(id).value, $('streamTransport').value));
    } catch (error) { text('liveError', error.message); return; }
    disconnect(); stop(); text('liveError', '');
    if ($('lightbox').open) $('lightbox').close();
    $('largeImage').removeAttribute('src');
    state.view = 'live';
    cameras.forEach((camera, i) => {
      const el = document.querySelector(`[data-camera="${camera}"]`);
      const old = el.querySelector('img');
      old.onload = null; old.onerror = null;
      const img = document.createElement('img'); img.alt = `${camera} 실시간 영상`;
      old.replaceWith(img);
      const stream = { el, img, empty: el.querySelector('.empty'), stamp: el.querySelector('.stamp'), seen: false, failed: false };
      streams.push(stream);
      stream.empty.textContent = '영상 수신 대기'; stream.empty.hidden = false;
      img.onload = refreshStatus;
      img.onerror = () => {
        stream.failed = true;
        stream.empty.textContent = '영상 연결 실패'; stream.empty.hidden = false;
        img.removeAttribute('src');
        text('liveError', '브리지 주소·토픽·Raw/Compressed 방식을 확인하세요. 브라우저가 로컬 네트워크 접근 권한을 요청하면 허용한 뒤 다시 연결하세요.');
        refreshStatus();
      };
      img.src = `${urls[i]}&t=${Date.now()}`;
    });
    $('disconnectLive').disabled = false;
    $('liveView').disabled = false;
    refreshStatus();
    monitor = setInterval(refreshStatus, 500);
    render();
  }

  $('liveSettingsButton').onclick = () => { $('liveSettings').open = !$('liveSettings').open; };
  $('topicPreset').onchange = () => presets[$('topicPreset').value].forEach((topic, i) => { $(fields[i]).value = topic; });
  $('liveForm').onsubmit = connect;
  $('disconnectLive').onclick = () => { disconnect(); render(); };
  $('liveView').onclick = () => { state.view = 'live'; render(); };
  // Expand the existing live image rather than opening a second streaming request.
  document.querySelectorAll('.camera').forEach(el => el.addEventListener('click', event => {
    if (state.view !== 'live') return;
    event.stopImmediatePropagation();
    el.classList.toggle('expanded');
  }, true));
  document.addEventListener('keydown', event => {
    if (event.key === 'Escape') document.querySelectorAll('.camera.expanded').forEach(el => el.classList.remove('expanded'));
  });
  window.addEventListener('pagehide', disconnect);
})();
