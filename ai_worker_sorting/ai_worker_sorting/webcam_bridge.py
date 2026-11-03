"""Windows webcam HTTP bridge. YOLO runs in the ROS2 container, not here."""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
import sys


class Frames:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.stamp = 0
        self.sequence = 0
        self.result = None
        self.result_sequence = 0
        self.result_time = 0.0


def handler_for(state, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass  # Never log authorization headers or camera content.

        def authorized(self):
            value = self.headers.get('X-Bridge-Token', '')
            if not hmac.compare_digest(value.encode(), token.encode()):
                self.send_error(401)
                return False
            self.connection.settimeout(2.0)
            return True

        def do_GET(self):
            if not self.authorized():
                return
            if self.path != '/frame.jpg':
                self.send_error(404)
                return
            with state.lock:
                jpeg, stamp, seq = state.jpeg, state.stamp, state.sequence
            if jpeg is None:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Capture-Nanoseconds', str(stamp))
            self.send_header('X-Frame-Sequence', str(seq))
            self.end_headers()
            try:
                self.wfile.write(jpeg)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_POST(self):
            if not self.authorized():
                return
            if self.path != '/annotated.jpg':
                self.send_error(404)
                return
            try:
                import cv2
                import numpy as np
                size = int(self.headers.get('Content-Length', '0'))
                seq = int(self.headers.get('X-Frame-Sequence', '0'))
                if not 0 < size <= 2_000_000 or seq <= 0:
                    raise ValueError('Invalid result headers')
                data = self.rfile.read(size)
                if len(data) != size:
                    raise ValueError('Incomplete result image')
                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None or frame.shape[0] > 2160 or frame.shape[1] > 3840:
                    raise ValueError('Invalid result image')
                with state.lock:
                    if seq > state.sequence:
                        raise ValueError('Result refers to a future frame')
                    if seq > state.result_sequence:
                        state.result, state.result_sequence = frame, seq
                        state.result_time = time.monotonic()
                self.send_response(204)
                self.end_headers()
            except (ValueError, TimeoutError):
                self.send_error(400)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--output', type=Path, default=Path('results/webcam-ros2'))
    parser.add_argument('--no-display', action='store_true')
    args = parser.parse_args()
    token = args.token_file.read_text(encoding='utf-8').strip()
    if len(token) < 32 or args.width <= 0 or not 0 < args.fps <= 60:
        parser.error('Invalid bridge token, width or FPS')
    if args.stop_file and args.stop_file.exists():
        parser.error('Stop file already exists')
    import cv2
    state = Frames()
    server = ThreadingHTTPServer((args.bind, args.port), handler_for(state, token))
    server.daemon_threads = True
    cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if sys.platform == 'win32' else cv2.CAP_ANY)
    if not cap.isOpened():
        server.server_close()
        cap.release()
        raise RuntimeError('Cannot open webcam')
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    title = 'ROS2 Docker YOLO - Laptop Camera | Q: quit | S: save'
    last_log = 0.0
    failures = 0
    args.output.mkdir(parents=True, exist_ok=True)
    print(f'BRIDGE_READY: {args.bind}:{args.port}; webcam source only', flush=True)
    try:
        if not args.no_display:
            cv2.namedWindow(title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(title, args.width, 480)
        while not (args.stop_file and args.stop_file.exists()):
            began = time.monotonic()
            ok, frame = cap.read()
            capture_stamp = time.time_ns()
            if not ok or frame is None:
                failures += 1
                if failures >= 10:
                    raise RuntimeError('Webcam disconnected')
                time.sleep(0.1)
                continue
            failures = 0
            height = max(1, round(frame.shape[0]*args.width/frame.shape[1]))
            frame = cv2.resize(frame, (args.width, height))
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise RuntimeError('JPEG encoding failed')
            with state.lock:
                state.jpeg, state.stamp = encoded.tobytes(), capture_stamp
                state.sequence += 1
                result, received = state.result, state.result_time
                sequence, result_sequence = state.sequence, state.result_sequence
            result_age = time.monotonic()-received if received else float('inf')
            preview = result.copy() if result is not None and result_age < 2.0 else frame.copy()
            if result_age >= 2.0:
                cv2.putText(preview, 'Waiting for ROS2 YOLO...', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if time.monotonic()-last_log >= 5:
                print(f'frames={sequence} ros2_result_sequence={result_sequence} '
                      f'result_age={result_age:.2f}s', flush=True)
                last_log = time.monotonic()
            if not args.no_display:
                cv2.imshow(title, preview)
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), ord('Q'), 27):
                    break
                if key in (ord('s'), ord('S')):
                    path = args.output / f'snapshot-{time.time_ns()}.jpg'
                    if not cv2.imwrite(str(path), preview):
                        raise RuntimeError('Snapshot write failed')
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
            time.sleep(max(0, 1/args.fps-(time.monotonic()-began)))
    finally:
        cap.release()
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        if not args.no_display:
            cv2.destroyAllWindows()
        print('BRIDGE_STOPPED', flush=True)


if __name__ == '__main__':
    main()
