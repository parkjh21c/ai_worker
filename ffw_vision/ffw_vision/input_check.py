from bisect import bisect_left
from collections import deque

NS_PER_SEC = 1_000_000_000
NS_PER_MS = 1_000_000


def stamp_to_ns(stamp):
    return stamp.sec * NS_PER_SEC + stamp.nanosec


def nearest_gap_ns(sorted_stamps, stamp):
    index = bisect_left(sorted_stamps, stamp)
    neighbors = sorted_stamps[max(index - 1, 0):index + 1]
    return min((abs(stamp - other) for other in neighbors), default=None)


class InputMonitor:

    def __init__(self, config, timeout_ns, info_history=300):
        self.config = config
        self.timeout_ns = timeout_ns
        self.info_history = info_history
        self.info_stamps = deque(maxlen=info_history)
        self.info = None  # (frame_id, width, height) of the latest camera_info
        self.info_frame_id_errors = 0
        self.last_image_rx_ns = None
        self.pending = []
        self.window_start_ns = None
        self._reset_window()

    def _reset_window(self):
        self.info_count = 0
        self.image_stamps = []
        self.latencies_ns = []
        self.frame_id_errors = 0
        self.size_errors = 0
        self.encodings = set()
        self.sizes = set()

    def on_camera_info(self, stamp_ns, frame_id, width, height):
        self.info_stamps.append(stamp_ns)
        self.info_count += 1
        self.info = (frame_id, width, height)
        if frame_id != self.config.frame_id:
            self.info_frame_id_errors += 1

    def on_image(self, stamp_ns, frame_id, width, height, encoding, rx_ns):
        if self.window_start_ns is None:
            self.window_start_ns = rx_ns
        self.last_image_rx_ns = rx_ns
        self.image_stamps.append(stamp_ns)
        self.latencies_ns.append(rx_ns - stamp_ns)
        self.encodings.add(encoding)
        self.sizes.add((width, height))
        if frame_id != self.config.frame_id:
            self.frame_id_errors += 1
        if self.info is not None and (width, height) != self.info[1:]:
            self.size_errors += 1

    def report(self, now_ns):
        if self.window_start_ns is None:
            self.window_start_ns = now_ns
        window_sec = max(now_ns - self.window_start_ns, 1) / NS_PER_SEC

        stamps = sorted(self.pending + self.image_stamps)
        info_sorted = sorted(self.info_stamps)
        latest_info = info_sorted[-1] if info_sorted else None
        info_set = set(info_sorted)
        paired = [s for s in stamps if s in info_set]
        pending = [s for s in stamps if latest_info is None or s > latest_info]
        unpaired = [s for s in stamps if s not in info_set and s not in pending]
        unpaired_gaps = [nearest_gap_ns(info_sorted, s) for s in unpaired]
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]

        if self.last_image_rx_ns is None or now_ns - self.last_image_rx_ns > self.timeout_ns:
            state = 'NO_INPUT'
        elif self.info_count == 0:
            state = 'NO_CAMERA_INFO'
        elif self.frame_id_errors or self.size_errors or self.info_frame_id_errors or unpaired:
            state = 'MISMATCH'
        else:
            state = 'OK'

        result = {
            'state': state,
            'images': len(self.image_stamps),
            'camera_infos': self.info_count,
            'rate_hz': len(self.image_stamps) / window_sec,
            'paired': len(paired),
            'unpaired': len(unpaired),
            'pending': len(pending),
            'max_unpaired_gap_ms': max(unpaired_gaps, default=0) / NS_PER_MS,
            'max_interval_ms': max(intervals, default=0) / NS_PER_MS,
            'latency_avg_ms': (sum(self.latencies_ns) / len(self.latencies_ns) / NS_PER_MS
                               if self.latencies_ns else 0.0),
            'latency_max_ms': max(self.latencies_ns, default=0) / NS_PER_MS,
            'frame_id_errors': self.frame_id_errors,
            'size_errors': self.size_errors,
            'info_frame_id_errors': self.info_frame_id_errors,
            'encodings': sorted(self.encodings),
            'sizes': sorted(self.sizes),
            'info': self.info,
        }
        self.pending = pending[-self.info_history:]
        self.info_frame_id_errors = 0
        self.window_start_ns = now_ns
        self._reset_window()
        return result


def format_report(result):
    sizes = ','.join(f'{w}x{h}' for w, h in result['sizes']) or '-'
    info = result['info']
    info_text = f'{info[0]} {info[1]}x{info[2]}' if info else '-'
    return (
        f"state={result['state']} images={result['images']} infos={result['camera_infos']} "
        f"rate={result['rate_hz']:.1f}Hz "
        f"paired={result['paired']} unpaired={result['unpaired']} pending={result['pending']} "
        f"max_unpaired_gap={result['max_unpaired_gap_ms']:.1f}ms "
        f"max_interval={result['max_interval_ms']:.1f}ms "
        f"latency_avg={result['latency_avg_ms']:.1f}ms latency_max={result['latency_max_ms']:.1f}ms "
        f"frame_id_err={result['frame_id_errors']} size_err={result['size_errors']} "
        f"info_frame_id_err={result['info_frame_id_errors']} "
        f"size={sizes} enc={','.join(result['encodings']) or '-'} info={info_text}"
    )
